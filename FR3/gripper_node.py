"""
[NEW] Robotiq 2F-140 gripper node: Modbus RTU master + ROS 2 bridge.
(Started from Omar's hand-tracker teleoperation node; every part that differs from his code is marked [NEW].)

Communication
-------------
Physical layer : RS-485 serial, through a USB <-> RS-485 adapter (/dev/ttyUSB0); gripper powered at 24 V apart
Protocol       : Modbus RTU, 115200 baud 8N1, 1 master (this node) / 1 slave (gripper, ID 9)
                 FC3  = read holding registers   (status, 2000-2002)
                 FC16 = write multiple registers (commands, 1000-1002)
ROS 2 side     : DDS / RTPS over UDP (topics below)
Reference      : Robotiq 2F-85 & 2F-140 Instruction Manual, sections 4.3-4.7

Naming: r... = request (written by us), g... = gripper status (read from the gripper)

Command registers (written, FC16)
---------------------------------
1000 high byte : action request
                 rACT (bit 0) activate: 0 = reset, 1 = activate
                 rGTO (bit 3) go to   : 1 = move toward rPR, 0 = stop
1000 low byte  : reserved
1001 high byte : reserved
1001 low byte  : rPR  position request, 0 = open ... 255 = closed
1002 high byte : rSP  speed, 0-255
1002 low byte  : rFR  force, 0-255 (relative, not in newton; 0 disables re-grasp)

Status registers (read, FC3)
----------------------------
2000 high byte : gripper status
                 gACT (bit 0)    activation echo
                 gGTO (bit 3)    go-to echo
                 gSTA (bits 4-5) 0 = reset, 1 = activating, 3 = activation completed
                 gOBJ (bits 6-7) 0 = moving, 1 = stopped on object while opening,
                                 2 = stopped on object while closing, 3 = at target (no object / object lost)
2000 low byte  : reserved
2001 high byte : fault status, gFLT (bits 0-3, 0 = no fault), kFLT (bits 4-7, controller fault)
2001 low byte  : echo of rPR
2002 high byte : gPO  actual position, 0 = open ... 255 = closed
2002 low byte  : gCU  motor current (~10 mA per unit)

ROS 2 topics
------------
sub /hand/finger_distance     std_msgs/Float64         target opening [m] in [min_width, max_width] (0.0 = ignore)
pub /gripper/current_distance std_msgs/Float64         actual opening [m], 10 Hz
pub /gripper/object_detected  std_msgs/Bool            [NEW] True if gOBJ = 1 or 2, gGTO = 1 and rPR echo = last
                                                       command (Robotiq manual 4.6 / 4.7.6), 10 Hz
pub /gripper/status           std_msgs/UInt8MultiArray [NEW] [gSTA, gOBJ, gFLT, gCU], 10 Hz
sub /gripper/stop             std_msgs/Bool            [NEW] True = stop (rGTO=0) + reject commands, False = release

Node name: robotiq_gripper (e.g. `ros2 param get /robotiq_gripper force`)
Parameters (fixed at start-up, defaults in __init__): port, command_cooldown_seconds, speed (64), force (30),
                                min_width (0.001), max_width (0.140), activation_timeout, watchdog_timeout
Safety: commands outside [min_width, max_width] or NaN are rejected (not saturated); software stop
        (/gripper/stop); watchdog (no valid status for watchdog_timeout s, or gSTA != 3 -> commands disabled).

Run:  python3 gripper_node.py                                      (real gripper on /dev/ttyUSB0)
      python3 gripper_node.py --ros-args -p port:=/tmp/ttyGRIPPER  (fake gripper, see tools/fake_robotiq.py)
"""

import math  # [NEW] math.isfinite for the command validation
import time

import rclpy
import rclpy.duration  # [NEW]
from pymodbus.client import ModbusSerialClient
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, UInt8MultiArray  # [NEW] Bool, UInt8MultiArray


class RobotiqGripperNode(Node):
    def __init__(self):
        super().__init__("robotiq_gripper")
        self.ready = False  # [NEW] True only when connected and activated (checked in main)

        # --- Configurations ---
        # [NEW] Serial port of the gripper as a ROS parameter (override with: --ros-args -p port:=/tmp/ttyGRIPPER)
        self.usb_port = (
            self.declare_parameter("port", "/dev/ttyUSB0")
            .get_parameter_value()
            .string_value
        )
        self.slave_id = 9
        self.width_limit = 0.140  # Robotiq 2F-140 max width is 14cm
        self.command_cooldown_seconds = (
            self.declare_parameter("command_cooldown_seconds", 0.1)   #mininum time between two commands)
            .get_parameter_value()
            .double_value
        )
        
        self.speed = max(0, min(self.declare_parameter("speed", 64).value, 255))  # [NEW] Speed and force of the fingers, 0-255 each (relative, not calibrated): fixed at start-up.
        self.force = max(1, min(self.declare_parameter("force", 30).value, 255))  # 0 disables re-grasp

        # [NEW] Accepted command range [m]: commands outside are REJECTED (not saturated)
        self.min_width = self.declare_parameter("min_width", 0.001).value  # 0.0 is the tracker "hand lost"
        self.max_width = self.declare_parameter("max_width", 0.140).value

        # [NEW] Watchdog: no valid status for this long -> gripper considered lost [s]
        self.watchdog_timeout = self.declare_parameter("watchdog_timeout", 1.0).value
        self.stopped = False     # [NEW] software stop latched by /gripper/stop
        self.gripper_ok = True   # [NEW] cleared by the watchdog

        # [NEW] Max time to wait for gSTA = 3 after the activation request [s]
        self.activation_timeout = (self.declare_parameter("activation_timeout", 10.0).get_parameter_value().double_value)
        self.last_width = 0.0
        self.last_registers = None  # [NEW] last 3 registers written (also used for the rPR echo check)

        # [NEW] Start already outside the cooldown(mininum time between two commands), so the first command is not dropped
        self.last_width_time = self.get_clock().now() - rclpy.duration.Duration(
            seconds=self.command_cooldown_seconds
        )

        # --- Modbus Connection ---
        self.get_logger().info(f"Connecting to gripper on {self.usb_port}...")  #return node's logger, better than a simple print
        self.client = ModbusSerialClient(port=self.usb_port, baudrate=115200, timeout=0.1) # Valori da datasheet?

        if not self.client.connect():
            self.get_logger().error(
                "Failed to connect! Check USB and 'dialout' permissions."
            )
            return

        # [NEW] Initialize and calibrate the hardware: stop here if the gripper does not answer
        if not self.activate_gripper():
            return

        # --- ROS 2 Publishers & Timers ---
        self.state_pub = self.create_publisher(Float64, "/gripper/current_distance", 1)
        #                                      ───┬───  ────────────┬────────────    ┬─
        #                                 tipo del msg       nome del topic        depth og QoS

        self.obj_pub = self.create_publisher(Bool, "/gripper/object_detected", 1)          # [NEW] True when the fingers stopped on an object (gOBJ = 1 or 2): used by the pick & place policy
        self.status_pub = self.create_publisher(UInt8MultiArray, "/gripper/status", 1)     # [NEW] Raw status for debug / logging: [gSTA, gOBJ, gFLT, gCU]
        self.last_flt = 0                                                                  # [NEW] Last fault code, to log a warning only when it changes

        # Read gripper state at 10Hz==0.1 sec 
        self.last_good_read = time.monotonic()  # [NEW] watchdog reference for thestart, otherwise it is activated at the beggining
        self.timer = self.create_timer(0.1, self.publish_gripper_state)   


        # --- ROS 2 Subscribers ---
        self.subscription = self.create_subscription(Float64, "/hand/finger_distance", self.listener_callback, 10)
        self.stop_sub = self.create_subscription(Bool, "/gripper/stop", self.stop_callback, 10)         # [NEW] Software stop: True = stop the fingers (rGTO = 0) and reject commands, False = release
        self.get_logger().info("Gripper active. Listening on /hand/finger_distance")
        self.ready = True  # [NEW]

    def activate_gripper(self):
        """Reset + activate (Modbus FC16), then poll gSTA (Modbus FC3) until activation is completed.

        [NEW] Returns True if the gripper is activated, False on timeout or communication error
        (Old's version waited a fixed sleep(3.0) and always reported "Activation complete").
        """
        self.get_logger().info("Activating Gripper (Clearing faults & calibrating)...")
        try:  # [NEW] communication errors handled
            # FC16 on 1000: all zero = reset (rACT = 0 and clears faults)
            self.client.write_registers(
                1000, [0x0000, 0x0000, 0x0000], device_id=self.slave_id
            )
            time.sleep(0.5)     # let the gripper see rACT = 0 before the change  0 -> 1 of rACT
                                # (manual 4.7.6: clear, then set rACT; 0.5 s is a conservative choice)

            # FC16 on 1000: rACT = 1 (bit 0 of the high byte) -> activation / calibration stroke
            self.client.write_registers(
                1000, [0x0100, 0x0000, 0x0000], device_id=self.slave_id
            )
        except Exception as e:
            self.get_logger().error(
                f"Activation request failed ({e}): check gripper power, cable and port."
            )
            return False

        # [NEW] Modbus RTU: the slave never speaks first, so we poll the status until gSTA = 3
        # (Robotiq manual 4.7.6, step 2)
        if not self.wait_for_activation(self.activation_timeout):
            self.get_logger().error(
                f"gSTA never reached 3 within {self.activation_timeout:.0f} s: "
                "check gripper power, cable and faults."
            )
            return False
        self.get_logger().info("Activation complete.")

        msg = Float64()
        msg.data = self.max_width  # [NEW]
        self.listener_callback(msg)  # Fully open after calibration
        # last_width / last_width_time are updated by listener_callback on a successful write
        return True

    # [NEW] whole method
    def wait_for_activation(self, timeout_s):
        """Read register 2000 (FC3) every 0.1 s until gSTA = 3. Returns True/False."""
        t_start = time.monotonic()
        while time.monotonic() - t_start < timeout_s:
            try:
                result = self.client.read_holding_registers(
                    2000, count=1, device_id=self.slave_id
                )
                if not result.isError():
                    status = (result.registers[0] >> 8) & 0xFF  # high byte of 2000
                    g_sta = (status >> 4) & 0x03                # bits 4-5
                    if g_sta == 3:  #named, activation
                        return True
            except Exception:
                pass  # no answer yet (e.g. gripper still booting): keep polling until timeout
            time.sleep(0.1)
        return False

    def publish_gripper_state(self):
        """Read the status (Modbus FC3, regs 2000-2002) at 10 Hz=0,1s, publish it, run the watchdog.
            Publish: object_detected (gGTO), status(gSTA, gOBJ, gFLT, gCU) and current_distance
        """
        read_ok = False  # [NEW] for the watchdog
        try:
            #FC3 request for the registers 2001 2002 e 2002
            result = self.client.read_holding_registers(
                2000, count=3, device_id=self.slave_id
            )
            #Se la lettura dei registri è risucita senza errori decodifichiamo i byte
            if not result.isError():   
                read_ok = True                          # [NEW]
                self.last_good_read = time.monotonic()  # [NEW] istant time of the last good read of Modbus FC3 of the registratio 2000 2001 and 2002

                # [NEW] Decodification of the bytes
                # Robotiq packs two bytes in each register: high byte = first, low byte = second. OLD's version read only gPO from register 2002)
                reg_2000 = result.registers[0]  # high: gripper status | low: reserved
                reg_2001 = result.registers[1]  # high: fault status   | low: echo of rPR
                reg_2002 = result.registers[2]  # high: gPO (position) | low: gCU (current)

                # [NEW] High byte of 2000: | gOBJ (7-6) | gSTA (5-4) | gGTO (3) | - (2-1) | gACT (0) |
                status = (reg_2000 >> 8) & 0xFF
                g_gto = (status >> 3) & 0x01  # echo of rGTO: 1 = go-to active, 0 = stopped
                g_sta = (status >> 4) & 0x03  # 0 reset, 1 activating, 3 activated
                g_obj = (status >> 6) & 0x03  # 0 moving, 1/2 object (opening/closing), 3 at target
                
                # [NEW] High byte of 2001 = fault byte: | kFLT (7-4) | gFLT (3-0) |
                g_flt = ((reg_2001 >> 8) & 0xFF) & 0x0F  # gripper fault, 0 = no fault
                r_pr_echo = reg_2001 & 0xFF  # low byte of 2001: echo of the last rPR received by the gripper
                actual_pos_raw = (reg_2002 >> 8) & 0xFF  # gPO, 0 = open ... 255 = closed
                g_cu = reg_2002 & 0xFF  # [NEW] gCU, low byte (~10 mA per unit)

                # [NEW] Watchdog on activation (gSTA): the gripper must stay activated means gSTA==3
                if g_sta != 3:
                    self.watchdog_trip(f"gSTA={g_sta}, gripper no longer activated")

                # [NEW] Status valid for the LAST command only when the gripper echoes the rPR we sent:
                # right after a new command the status may still describe the previous one
                # (Robotiq manual 4.6 "Wait for: gPR == position requested in previous step", 4.7.6 step 5).
                echo_ok = self.last_registers is None or r_pr_echo == self.last_registers[1]  

                # [NEW] Object detection: gOBJ is meaningful only while gGTO = 1 (manual 4.5.3: "Only valid if gGTO = 1")
                obj_msg = Bool()
                obj_msg.data = self.gripper_ok and g_gto == 1 and echo_ok and g_obj in (1, 2)  #true solo se valgono tutte e quattro le condizioni: mano disponibile, movimento attivo (non in stop), stato riferito all’ultimo comando (eco), dita ferme su un oggetto.
                self.obj_pub.publish(obj_msg)

                # [NEW] Raw status vector
                status_msg = UInt8MultiArray()
                status_msg.data = [g_sta, g_obj, g_flt, g_cu]
                self.status_pub.publish(status_msg)

                # [NEW] Fault: warn once when it appears or changes, info when it clears
                if g_flt != self.last_flt:
                    if g_flt != 0:
                        self.get_logger().warn(f"Gripper fault gFLT=0x{g_flt:X}")
                    else:
                        self.get_logger().info("Gripper fault cleared")
                    self.last_flt = g_flt

                # Linear mapping 0-255 -> 0.14-0.0 m
                msg = Float64()
                msg.data = self.width_limit * (1.0 - (actual_pos_raw / 255.0))
                self.state_pub.publish(msg)
            else:
                self.get_logger().warn("Failed to read Modbus registers.", throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().warn(f"Modbus read error: {e}", throttle_duration_sec=1.0)

        # [NEW] Watchdog: no valid status for watchdog_timeout seconds -> gripper lost
        if not read_ok and time.monotonic() - self.last_good_read > self.watchdog_timeout:
            self.watchdog_trip(f"no valid status for {self.watchdog_timeout:.1f} s")
            obj_msg = Bool()
            obj_msg.data = False
            self.obj_pub.publish(obj_msg)

    def listener_callback(self, msg):
        target_width = msg.data

        # 1. "Hand lost" signal of the hand tracker (exactly 0.0): ignore
        if target_width == 0.0:
            return

        # 2. [NEW] Safety: reject (do not saturate) commands when stopped / lost / invalid.
        # OLD's version used a threshold (> 0.1 m -> 0.08 m, otherwise 0.001 m): negative or NaN = full closing.
        if self.stopped or not self.gripper_ok:
            self.get_logger().warn(
                "Command rejected: gripper stopped or not available", throttle_duration_sec=1.0
            )
            return
        if not math.isfinite(target_width) or not (self.min_width <= target_width <= self.max_width):  # if number is NaN or ifnite or not in the range

            self.get_logger().warn(
                f"Command rejected: {target_width} m outside "
                f"[{self.min_width}, {self.max_width}] m"
            )
            return

        # 3. Map distance [m] -> rPR (0 = open ... 255 = closed)
        if target_width <= self.min_width + 1e-9:
            # [NEW] Full closing exactly as in the Robotiq manual (4.5.3 / 4.7.6 step 4): rPR = 0xFF.
            # The grasp is stopped by the force limit and reported by gOBJ = 2.
            mapped_position = 255
        else:
            mapped_position = int((1.0 - (target_width / self.width_limit)) * 255.0)
            mapped_position = max(0, min(mapped_position, 255))

        # 4. Modbus FC16 registers: rACT=1 + rGTO=1 | rPR | speed, force
        # [NEW] speed / force from the parameters (Omar: fixed 0xFF7F)
        registers = [0x0900, mapped_position, (self.speed << 8) | self.force]

        try:
            # [NEW] Send only if one of the 3 registers changed (Omar: only if the width changed),
            # at most once per cooldown (FC16 1000 1001 E 1002)
            if registers != self.last_registers:
                if (
                    self.get_clock().now() - self.last_width_time < rclpy.duration.Duration(seconds=self.command_cooldown_seconds)):
                    return

                self.client.write_registers(1000, registers, device_id=self.slave_id)

                # Only update if the write was successful
                self.last_width = target_width
                self.last_registers = registers  # [NEW]
                self.last_width_time = self.get_clock().now()

        except Exception as e:
            self.get_logger().warn(f"Modbus write failed: {e}")

    # [NEW] whole method
    def stop_callback(self, msg):
        """/gripper/stop: True = stop the fingers where they are and latch, False = release."""
        if msg.data:
            # Modbus FC16: rACT = 1, rGTO = 0 -> the gripper stops immediately (no cooldown)
            r_pr = self.last_registers[1] if self.last_registers else 0
            registers = [0x0100, r_pr, (self.speed << 8) | self.force]
            try:
                self.client.write_registers(1000, registers, device_id=self.slave_id)
                self.last_registers = registers
            except Exception as e:
                self.get_logger().error(f"STOP write failed: {e}")
            if not self.stopped:
                self.get_logger().warn("SOFTWARE STOP: fingers stopped, commands rejected")
            self.stopped = True
        elif self.stopped:
            self.stopped = False
            self.get_logger().info("Software stop released: commands accepted again")

    # [NEW] whole method
    def watchdog_trip(self, reason):
        """Disable the commands once the gripper is lost (latched: restart the node to recover)."""
        if self.gripper_ok:
            self.gripper_ok = False
            self.get_logger().error(
                f"GRIPPER LOST ({reason}): commands disabled, check the gripper and restart the node"
            )


def main(args=None):
    rclpy.init(args=args)
    node = RobotiqGripperNode()
    # [NEW] spin only if connected and activated; clean exit on Ctrl+C
    if node.ready:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    else:
        node.get_logger().error("Gripper not ready: exiting.")

    if hasattr(node, "client"):
        node.client.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

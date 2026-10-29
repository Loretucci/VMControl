"""Simulated FR3 plant: receive motor torques and publish measured joint states."""

import argparse
from pathlib import Path
import time

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = [f"fr3_joint{i}" for i in range(1, 8)]
# Limits used for this offline test, in Nm, following the homework setup.
TORQUE_LIMITS = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])


def main():
    parser = argparse.ArgumentParser(description="Simulated FR3 for the VMC ROS 2 test")
    parser.add_argument("--state-topic", default="/robot/joint_states")
    parser.add_argument("--torque-topic", default="/mujoco/robot/torques")
    parser.add_argument("--state-rate", type=float, default=100.0, help="JointState publication rate in Hz")
    parser.add_argument("--timestep", type=float, default=0.001, help="MuJoCo step in seconds")
    parser.add_argument("--duration", type=float, default=20.0, help="Wall-clock seconds; <= 0 runs indefinitely")
    parser.add_argument("--torque-timeout", type=float, default=0.1, help="Use zero VMC torque if no command arrives for this many seconds")
    args = parser.parse_args()
    if args.state_rate <= 0 or args.timestep <= 0 or args.torque_timeout <= 0:
        parser.error("state-rate, timestep and torque-timeout must be positive")

    # This model represents the simulated physical robot, independently of
    # the virtual model used by h1_vmc_ros.py.
    scene_path = Path(__file__).resolve().parent / "torque_control" / "model" / "scene1.xml"
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    model.opt.timestep = args.timestep
    data = mujoco.MjData(model)

    # Start with a visibly bent elbow while preserving the model's home pose.
    start_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "bent_start")
    if start_id < 0:
        raise ValueError("The FR3 model has no 'bent_start' keyframe")
    mujoco.mj_resetDataKeyframe(model, data, start_id)
    mujoco.mj_forward(model, data)

    print(f"nq: {model.nq}, nv: {model.nv}, nu: {model.nu}")
    print(f"Initial qpos: {data.qpos}")

    # The motor controls and the JointState arrays follow fr3_joint1..7 order.
    if (model.nq, model.nv, model.nu) != (7, 7, 7):
        raise ValueError("This test expects exactly seven FR3 joint coordinates and motors")
    for i, name in enumerate(JOINT_NAMES):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if joint_id < 0 or actuator_id != i or model.jnt_qposadr[joint_id] != i or model.jnt_dofadr[joint_id] != i:
            raise ValueError(f"Unexpected joint or motor ordering for {name}")

    rclpy.init()
    node = Node("simulated_fr3_plant")
    state_pub = node.create_publisher(JointState, args.state_topic, 10)
    latest_vmc_torque = np.zeros(7)
    last_torque_time = None

    def on_torque(message: Float64MultiArray):
        nonlocal last_torque_time
        candidate = np.asarray(message.data, dtype=float)
        if candidate.shape != (7,) or not np.all(np.isfinite(candidate)):
            node.get_logger().warn("Ignoring a torque command without seven finite values")
            return
        latest_vmc_torque[:] = candidate
        last_torque_time = time.perf_counter()

    torque_sub = node.create_subscription(Float64MultiArray, args.torque_topic, on_torque, 10)

    def publish_state():
        state = JointState()
        state.header.stamp = node.get_clock().now().to_msg()
        state.name = JOINT_NAMES
        state.position = data.qpos.tolist()
        state.velocity = data.qvel.tolist()
        state_pub.publish(state)

    print(f"[SIM] Publishing joint states on {args.state_topic}")
    print(f"[SIM] Receiving VMC torques on {args.torque_topic}")
    publish_state()

    start = time.perf_counter()
    next_step = start
    next_state_pub = data.time + 1.0 / args.state_rate
    try:
        while rclpy.ok() and (args.duration <= 0 or time.perf_counter() - start < args.duration):
            # Handle incoming torques in this thread, then advance the plant.
            rclpy.spin_once(node, timeout_sec=0.0)
            now = time.perf_counter()
            vmc_torque = latest_vmc_torque if last_torque_time is not None and now - last_torque_time < args.torque_timeout else 0.0

            # qfrc_bias compensates gravity and velocity-dependent bias terms.
            # Joint damping and friction remain part of the simulated dynamics.
            mujoco.mj_forward(model, data)
            data.ctrl[:] = np.clip(data.qfrc_bias + vmc_torque, -TORQUE_LIMITS, TORQUE_LIMITS)
            mujoco.mj_step(model, data)

            if data.time >= next_state_pub:
                publish_state()
                next_state_pub += 1.0 / args.state_rate

            # Keep simulation time close to real time without a runaway catch-up.
            next_step += model.opt.timestep
            delay = next_step - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.1:
                next_step = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[SIM] Finished at t={data.time:.2f} s; final qpos={np.round(data.qpos, 3)}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

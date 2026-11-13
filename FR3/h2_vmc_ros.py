"""Compute FR3 virtual-mechanism joint torques from measured ROS joint states.

MuJoCo supplies forward kinematics, site velocities, and Jacobians here. The
robot's joint trajectory comes from ROS; this file never advances it with
``mj_step``. The optional passive viewer displays that measured configuration.
"""

import argparse
from pathlib import Path
import time

import mujoco
from mujoco import viewer
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Bool, UInt8MultiArray
import numpy as np
from scipy.spatial.transform import Rotation
from threading import Lock

from mujoco_vmc import (
    LinearDamper,
    MuJoCoSite,
    ObjectState,
    RobotROSBridge,
    RobotStateSynchronizer,
    TanhSpring,
    VirtualConnection,
    VirtualMechanism,
    StaticObject,
    resolve_menagerie_path,
)

def build_ee_link_model(
    spec,
    link_length: float = 0.4,
):
    """Add three rigid control sites in the Robotiq base frame around the TCP."""
    ee_spec = spec.body("robotiq_base_link")
    tcp_site = spec.site("tcp")
    tcp_pos = tcp_site.pos
    L = float(link_length)

    # The existing TCP site is the desired contact point. These three sites
    # provide separated lever arms for the virtual connections around it.
    # Positions passed to add_site are in robotiq_base_link coordinates:
    # TCP +x (jaw closing direction) is robotiq_base_link -y.
    ee_spec.add_site(name="ee_top_spring", pos=[tcp_pos[0], tcp_pos[1], tcp_pos[2] - L], size=[0.012, 0.0, 0.0], rgba=[1.0, 0.0, 0.0, 0.8], group=0)
    ee_spec.add_site(name="gripper_left_finger", pos=[tcp_pos[0], tcp_pos[1] - 0.3, tcp_pos[2]], size=[0.012, 0.0, 0.0], rgba=[1.0, 0.0, 0.0, 0.8], group=0)
    ee_spec.add_site(name="gripper_right_finger", pos=[tcp_pos[0], tcp_pos[1] + 0.3, tcp_pos[2]], size=[0.012, 0.0, 0.0], rgba=[1.0, 0.0, 0.0, 0.8], group=0)

def build_positions(camera_pose, link_length):
    """Create three points TCP position in world coordinates."""
    L = float(link_length)
    p_marker, q_marker, received_at = camera_pose

    rotation_flat = np.empty(9)
    mujoco.mju_quat2Mat(rotation_flat, q_marker)  # q_marker è [w, x, y, z]
    R_base_aruco = rotation_flat.reshape(3, 3)

    x_aruco_in_base = R_base_aruco[:, 0]
    z_aruco = R_base_aruco[:, 2]
    h = 0.0
    p_target = p_marker - h * z_aruco
    p_left = p_target + 0.30 * x_aruco_in_base
    p_right = p_target - 0.30 * x_aruco_in_base
    # Match the simulation's top target: it lies along target +z.
    p_top = p_target + L * z_aruco
    
    return p_top,p_left,p_right, R_base_aruco

def build_ref_objects(p_top,p_left,p_right,rotation_matrix):
    """Create three targets around the desired TCP position in world coordinates."""

    
    # The top target is L above the TCP; the lateral targets are at world +/-x.
    target_top_spring = StaticObject(ObjectState(position=p_top, rotation=rotation_matrix, linear_velocity=np.zeros(3)),name = "target_attachment_site")
    target_l = StaticObject(ObjectState(position=p_left, rotation=rotation_matrix,linear_velocity=np.zeros(3)),name ="target_left_finger")
    target_r = StaticObject(ObjectState(position=p_right, rotation=rotation_matrix,linear_velocity=np.zeros(3)),name = "target_right_finger")
    return target_top_spring,target_l, target_r

def update_positions(p_top, p_left, p_right, rotation_matrix, target_top_spring, target_left, target_right):

    target_left.set_state(ObjectState(position=p_left, rotation=rotation_matrix, linear_velocity=np.zeros(3)))
    target_right.set_state(ObjectState(position=p_right, rotation=rotation_matrix, linear_velocity=np.zeros(3)))
    target_top_spring.set_state(ObjectState(position=p_top, rotation=rotation_matrix, linear_velocity=np.zeros(3)))

def orientation_from_points(attachment, left, right):
    """Build a world-frame orientation from the virtual finger and tool axes."""
    x_axis = np.asarray(left, dtype=float) - right
    x_axis /= np.linalg.norm(x_axis)
    z_axis = (np.asarray(left, dtype=float) + right) / 2.0 - attachment
    z_axis -= np.dot(z_axis, x_axis) * x_axis
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))

def main():
    """Build the virtual robot, receive joint states, and publish VMC torques."""
    # The topic names must match those used by the physical ROS controller.
    parser = argparse.ArgumentParser(description="FR3 virtual-mechanism torque controller")
    parser.add_argument("--sub-topic", "--topic", dest="topic", type=str, default="/robot/joint_states", help="Topic to receive real robot states (default: /robot/joint_states)")
    parser.add_argument("--torque-pub-topic", type=str, default="/mujoco/robot/torques", help="Topic to publish computed joint torques (default: /mujoco/robot/torques)")
    parser.add_argument("--pub-rate", type=float, default=1000.0, help="Publish rate in Hz for outgoing ROS topics (default: 1000.0)")
    parser.add_argument("--link-length", type=float, default=0.4, help="Distance from TCP to the upper virtual spring site in meters (default: 0.4)")
    parser.add_argument("--stiffness", type=float, default=2000.0, help="VMC spring stiffness for each endpoint (N/m)")
    parser.add_argument("--damping", type=float, default=50.0, help="VMC damper coefficient for each endpoint (Ns/m)")
    parser.add_argument("--max-force", type=float, default=20.0, help="Maximum saturating VMC force per spring (N)")
    parser.add_argument("--timestep", type=float, default=0.001, help="Controller loop pacing interval in seconds (default: 0.001); no mj_step is called")
    parser.add_argument("--state-timeout", type=float, default=0.2, help="Publish zero VMC torque if no new robot state arrives for this many seconds")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode without passive viewer")
    parser.add_argument("--duration", type=float, default=60.0, help="Run duration in seconds if headless (<= 0 runs indefinitely)")
    parser.add_argument("--camera-topic", type = str, default = "/vision/aruco_pose", help="Topic on which receiving ArUco pose")
    #parser.add_argument("--scene", type = str, default = "mujoco/")
    args = parser.parse_args()
    if args.state_timeout <= 0:
        parser.error("state-timeout must be positive")

    phase = "APPROACH"
    position_tolerance = 0.01 # 1 cm, errore del TCP
    connection_tolerance = 0.01 # 1 cm, massimo dei tre collegamenti

    # The combined scene contains the robot, gripper, and world. Add the three
    # control sites to the Robotiq base before compiling the complete model.
    menagerie = resolve_menagerie_path()
    scene_xml = menagerie / "fr3_robotiq_2f_140" / "scene_fr3_robotiq.xml"
    if not scene_xml.is_file():
        raise FileNotFoundError(f"Modello non trovato: {scene_xml}")


    spec = mujoco.MjSpec.from_file(str(scene_xml))
    build_ee_link_model(spec, args.link_length)
    model = spec.compile()
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION
    # The model stores geometry/parameters; MjData stores its current state and
    # computed quantities. This timestep only paces our Python loop below.
    model.opt.timestep = args.timestep
    data = mujoco.MjData(model)
    sync = RobotStateSynchronizer(model)


    # The synchronizer maps joint names to MuJoCo qpos/qvel addresses. The ROS
    # bridge receives states and publishes torques on its own executor thread.
    bridge = RobotROSBridge(
        synchronizer= sync,
        sub_topic=args.topic,
        pub_torque_topic=args.torque_pub_topic,
        pub_rate_hz=args.pub_rate,
    )

    bridge.start()
    camera_lock = Lock()
    latest_camera_pose = None

    def camera_callback(msg):
        nonlocal latest_camera_pose

        if msg.header.frame_id != "base":
            return

        p = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)

        q = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ], dtype=float)

        norm = np.linalg.norm(q)
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)) or norm < 1e-6:
            return

        with camera_lock:
            latest_camera_pose = (p, q / norm, time.perf_counter())     

    camera_sub = bridge.node.create_subscription(PoseStamped, args.camera_topic, camera_callback, 1)

    first_camera_pose = None
    while first_camera_pose is None:
        with camera_lock:
            first_camera_pose = latest_camera_pose

        if first_camera_pose is None:
            time.sleep(0.05)

    # Wait for the first measured pose before evaluating any virtual spring.
    initial_qpos = None
    initial_qvel = None
    while initial_qpos is None:
        qpos, qvel = bridge.get_latest_state()
        if qpos is not None:
            initial_qpos = qpos
            initial_qvel = qvel
            print(f"[ROS2] Received initial real robot joint state: {np.round(initial_qpos, 3)}")
            break

        time.sleep(0.05)
    

    # Inject the first measured positions and velocities. The damper uses
    # velocity immediately, so dropping the first dq would be misleading.
    # Subsequent messages update both arrays via apply_incoming_state().
    sync.set_state(data, initial_qpos, initial_qvel, forward=True)

    target_top_position, target_left_position, target_right_position, R_base_aruco  = build_positions(first_camera_pose, args.link_length)
    target_top_spring, target_left, target_right = build_ref_objects(target_top_position, target_left_position, target_right_position, R_base_aruco)

    # The TCP is the contact point; the other three sites anchor the springs.
    tcp_id = model.site("tcp").id
    top_spring_id = model.site("ee_top_spring").id
    left_grip_id = model.site("gripper_left_finger").id
    right_grip_id = model.site("gripper_right_finger").id
    # MuJoCoSite wraps a site so the VMC library can read its world position,
    # linear velocity, and force-to-joint mapping from the current MjData.
    r_top_site = MuJoCoSite(top_spring_id, "top_site")
    r_left_site = MuJoCoSite(left_grip_id, "left_finger")
    r_right_site = MuJoCoSite(right_grip_id, "right_finger")

    # Each connection pulls a robot site toward its fixed target with a
    # saturating spring and opposes relative velocity with a linear damper.
    # max_force limits each spring's force components independently; it is
    # not a bound on total force or joint torque. The damper is not saturated
    # by that spring parameter either.
    conn_att = VirtualConnection(
        source=r_top_site,
        target=target_top_spring,
        components=[
            TanhSpring(args.stiffness, max_force=args.max_force),
            LinearDamper(args.damping),
        ],
        name="link_att_spring",
    )
    conn_left = VirtualConnection(
        source=r_left_site,
        target=target_left,
        components=[
            TanhSpring(args.stiffness, max_force=args.max_force),
            LinearDamper(args.damping),
        ],
        name="link_left_spring",
    )
    conn_right = VirtualConnection(
        source=r_right_site,
        target=target_right,
        components=[
            TanhSpring(args.stiffness, max_force=args.max_force),
            LinearDamper(args.damping),
        ],
        name="link_right_spring",
    )
    mechanism = VirtualMechanism([conn_att,conn_left, conn_right])

    # Ensure the initial site positions and Jacobians reflect initial_qpos.
    # Timeouts use a monotonic clock; time.time() is only for human-readable
    # logging frequency.
    mujoco.mj_forward(model, data)
    last_telemetry_log = time.time()
    last_state_time = time.perf_counter()
    state_timeout_logged = False
    dt = model.opt.timestep

    gripper_pub = bridge.node.create_publisher(Float64, "/hand/finger_distance", 10)

    last_gripper_detection = None
    gripper_lock = Lock()

    def gripper_obj_det_cb(msg: Bool):
        nonlocal last_gripper_detection
        with gripper_lock:
            last_gripper_detection = (msg.data, time.perf_counter())

    gripper_sub = bridge.node.create_subscription(Bool, "/gripper/object_detected", gripper_obj_det_cb, 10)

    last_gripper_status = None
    def gripper_status_cb(msg: UInt8MultiArray):
        nonlocal last_gripper_status
        if len(msg.data) != 4:
            return
        with gripper_lock:
            last_gripper_status = (tuple(msg.data), time.perf_counter())

    gripper_status_sub = bridge.node.create_subscription(UInt8MultiArray, "/gripper/status", gripper_status_cb, 10)
    
    t_arrival = None
    last_camera_time = first_camera_pose[2]
    camera_timeout = 0.8 #in seconds
    closing_timeout = 5.0  # valore iniziale, da regolare misurando la chiusura

    def step_iteration():

        """Update the virtual scene from ROS and buffer one torque command.

        No physics step happens here: the physical robot is the source of q and
        dq. ``mechanism.apply`` computes generalized forces from the virtual
        components at that measured state.
        """
        nonlocal last_telemetry_log, last_state_time, state_timeout_logged, latest_camera_pose, phase, position_tolerance, connection_tolerance, t_arrival, last_camera_time
        # Consume the most recent JointState, if any. With forward=True, the
        # synchronizer calls mj_forward to refresh site poses and velocities.
        state_updated = bridge.apply_incoming_state(data, forward=True)
        with camera_lock:
            camera_pose = latest_camera_pose
            latest_camera_pose = None
        
        if camera_pose is not None:
            last_camera_time = camera_pose[2]
            new_p_top, new_p_left, new_p_right, new_rotation = build_positions(camera_pose, args.link_length)
            update_positions(new_p_top, new_p_left, new_p_right, new_rotation, target_top_spring, target_left, target_right)

        if state_updated:
            last_state_time = time.perf_counter()
            state_timeout_logged = False

        # The active StaticObjects are the source of truth for both errors.
        target_att_pos = target_top_spring.state(model, data).position
        target_left_pos = target_left.state(model, data).position
        target_right_pos = target_right.state(model, data).position
        desired_tip_pos = (target_left_pos + target_right_pos) / 2.0
        error_xyz = desired_tip_pos - data.site_xpos[tcp_id]

        target_rotation = orientation_from_points(target_att_pos, target_left_pos, target_right_pos)
        measured_rotation = orientation_from_points(
        data.site_xpos[top_spring_id],
        data.site_xpos[left_grip_id],
        data.site_xpos[right_grip_id],
        )

        connection_errors = [
            np.linalg.norm(target_att_pos - data.site_xpos[top_spring_id]),
            np.linalg.norm(target_left_pos - data.site_xpos[left_grip_id]),
            np.linalg.norm(target_right_pos - data.site_xpos[right_grip_id]),
        ]

        if state_updated and (time.time() - last_telemetry_log > 1.0):

            # The rotation vector gives the signed correction about the target
            # frame's X, Y and Z axes (the ArUco axes after a camera update).
            relative_rotation_target = target_rotation.T @ measured_rotation
            orientation_error_xyz_deg = -np.degrees(
            Rotation.from_matrix(relative_rotation_target).as_rotvec()
            )

            print(
                f"Errore punta xyz: {np.round(error_xyz, 4)} m; "
                f"modulo: {np.linalg.norm(error_xyz):.4f} m; "
                f"errore orientazione target xyz: {np.round(orientation_error_xyz_deg, 2)}°; "
                f"massimo sui 3 collegamenti: {max(connection_errors):.4f} m"
            )
            last_telemetry_log = time.time()


        within_tolerance = (np.linalg.norm(error_xyz) < position_tolerance and max(connection_errors) < connection_tolerance)
        close_requested = False

        t_actual = time.perf_counter()
        robot_state_recent = t_actual - last_state_time <= args.state_timeout
        camera_data_recent = t_actual - last_camera_time <= camera_timeout
        if phase == "APPROACH":
            if within_tolerance and robot_state_recent and camera_data_recent:
                if t_arrival is None:
                    t_arrival = t_actual

                elif t_actual - t_arrival >= 0.5:
                    phase = "CLOSING"
                    close_requested = True
                    t_arrival = None
            else:
                t_arrival = None
        
        with gripper_lock:
            detection = last_gripper_detection
            status = last_gripper_status

        if phase == "CLOSING" and t_close_request is not None:
            if status is not None and detection is not None:
                values, t_status = status
                g_sta, g_obj, g_flt, g_cu = values
                detected, t_detection = detection
                if t_status > t_close_request and t_detection > t_close_request:
                    if detected and g_sta == 3 and g_obj == 2 and g_flt == 0:
                        phase = "GRASPED"

            if phase == "CLOSING" and t_actual - t_close_request >= closing_timeout:
                phase = "FAILED"


        # Clear forces from the previous iteration; otherwise they would
        # accumulate in qfrc_applied. MuJoCoSite.apply_force maps each world
        # force to generalized joint forces via the relevant Jacobian.
        data.qfrc_applied[:] = 0.0
        # When telemetry stops, request zero *additional VMC torque*. This
        # Python timeout does not replace a watchdog in the robot controller.
        if time.perf_counter() - last_state_time <= args.state_timeout:
            mechanism.apply(model, data)
            vmc_torques = np.array([data.qfrc_applied[adr] for adr in sync.qvel_indices], dtype=float)
        else:
            vmc_torques = np.zeros(sync.num_joints)
            if not state_timeout_logged:
                print("[ROS2] Joint state timed out; publishing zero VMC torque")
                state_timeout_logged = True
        # Extract the seven robot DOFs in name order. No qfrc_bias or gravity
        # term is added here; the real controller must define that convention.
        required_torques = vmc_torques

        # publish() fills a thread-safe buffer. A ROS timer sends its latest
        # value at --pub-rate; this call does not directly publish a message.
        bridge.publish(data, torques=required_torques) 

        return close_requested

    # 6. Execution loop. step_iteration() is the only reader of new ROS joint states.
    t_start = time.perf_counter()
    next_tick = t_start
    close_msg = Float64()
    close_msg.data = 0.001
    t_close_request = None
    try:
        if args.headless:
            while args.duration <= 0 or time.perf_counter() - t_start < args.duration:
                close_request = step_iteration()
                if close_request:
                    gripper_pub.publish(close_msg)
                    t_close_request = time.perf_counter()
                next_tick += dt
                remaining = next_tick - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    next_tick = time.perf_counter()
        else:
            next_viewer_sync = t_start
            with viewer.launch_passive(model, data) as v:
                print("[INFO] Passive viewer active. Double-right-click and drag the robot end-effector!")
                while v.is_running() and (args.duration <= 0 or time.perf_counter() - t_start < args.duration):
                    close_request = step_iteration()
                    if close_request:
                        gripper_pub.publish(close_msg)
                        t_close_request = time.perf_counter()

                    now = time.perf_counter()
                    if now >= next_viewer_sync:
                        v.sync()
                        next_viewer_sync = now + 1.0 / 30.0
                    next_tick += dt
                    remaining = next_tick - time.perf_counter()
                    if remaining > 0:
                        time.sleep(remaining)
                    else:
                        next_tick = time.perf_counter()
    except KeyboardInterrupt:
        print("\n[INFO] Control interrupted by user.")
    finally:
        bridge.stop()

     
if __name__ == "__main__":
    main()

    

    
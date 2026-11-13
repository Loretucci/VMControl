"""Drive the real FR3 with virtual connections anchored to an ArUco pose.

The camera supplies a target pose in the robot base frame. ROS supplies the
measured joint positions and velocities. MuJoCo computes kinematics and maps
the virtual spring/damper forces to joint torques. The robot trajectory comes
from ROS; this file does not advance the robot with ``mj_step``. The optional
passive viewer displays the measured configuration.
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
    """Attach three virtual-force anchors rigidly to the gripper body.

    These sites are added to MjSpec before compilation. Their coordinates are
    local to robotiq_base_link; MuJoCo later computes their world positions
    from the measured robot configuration. They do not add robot DOFs.
    """
    ee_spec = spec.body("robotiq_base_link")
    tcp_site = spec.site("tcp")
    tcp_pos = tcp_site.pos
    L = float(link_length)

    # The existing TCP is the point intended to meet the marker. The top and
    # lateral sites supply distinct lever arms for the three force connections.
    # In this gripper model, TCP +x is robotiq_base_link -y. The top site is
    # displaced by -L along the gripper body's local z; the two lateral sites
    # are displaced by +/-0.30 m along its local y.
    ee_spec.add_site(name="ee_top_spring", pos=[tcp_pos[0], tcp_pos[1], tcp_pos[2] - L], size=[0.012, 0.0, 0.0], rgba=[1.0, 0.0, 0.0, 0.8], group=0)
    ee_spec.add_site(name="gripper_left_finger", pos=[tcp_pos[0], tcp_pos[1] - 0.3, tcp_pos[2]], size=[0.012, 0.0, 0.0], rgba=[1.0, 0.0, 0.0, 0.8], group=0)
    ee_spec.add_site(name="gripper_right_finger", pos=[tcp_pos[0], tcp_pos[1] + 0.3, tcp_pos[2]], size=[0.012, 0.0, 0.0], rgba=[1.0, 0.0, 0.0, 0.8], group=0)

def build_positions(camera_pose, link_length):
    """Build the three target points from one measured base-frame marker pose.

    camera_pose contains (position, quaternion, receive_time). The position is
    in metres in the FR3 base frame; the quaternion uses MuJoCo order wxyz.
    The returned points are also in base/world coordinates, so they can be
    compared directly with MuJoCo's site_xpos values.
    """
    L = float(link_length)
    p_marker, q_marker, received_at = camera_pose

    # Convert the marker orientation to a rotation matrix. Its columns are
    # the marker's x, y and z axes expressed in the base frame.
    rotation_flat = np.empty(9)
    mujoco.mju_quat2Mat(rotation_flat, q_marker)  # MuJoCo quaternion order: w, x, y, z.
    R_base_aruco = rotation_flat.reshape(3, 3)

    x_aruco_in_base = R_base_aruco[:, 0]
    z_aruco = R_base_aruco[:, 2]
    # With h=0, the desired TCP coincides with the marker origin. The side
    # targets lie along marker x and the top target along marker z.
    h = 0.0
    p_target = p_marker - h * z_aruco
    p_left = p_target + 0.30 * x_aruco_in_base
    p_right = p_target - 0.30 * x_aruco_in_base
    # This is the position the upper gripper site is pulled toward.
    p_top = p_target + L * z_aruco
    
    return p_top,p_left,p_right, R_base_aruco

def build_ref_objects(p_top,p_left,p_right,rotation_matrix):
    """Represent the three target points as non-MuJoCo virtual objects.

    StaticObject supplies position, orientation and zero linear velocity to
    the virtual mechanism. The camera callback will update these targets
    later; they are not physical bodies in the compiled robot model.
    """

    
    # Their initial positions come from build_positions(). The target's
    # orientation follows the detected marker, although the three connections
    # below use only translational springs and dampers.
    target_top_spring = StaticObject(ObjectState(position=p_top, rotation=rotation_matrix, linear_velocity=np.zeros(3)),name = "target_attachment_site")
    target_l = StaticObject(ObjectState(position=p_left, rotation=rotation_matrix,linear_velocity=np.zeros(3)),name ="target_left_finger")
    target_r = StaticObject(ObjectState(position=p_right, rotation=rotation_matrix,linear_velocity=np.zeros(3)),name = "target_right_finger")
    return target_top_spring,target_l, target_r

def update_positions(p_top, p_left, p_right, rotation_matrix, target_top_spring, target_left, target_right):
    """Move all virtual targets to the latest camera-derived positions.

    The target velocity is currently set to zero on each update. Thus the
    damper does not use an estimate of marker velocity between camera frames.
    """

    target_left.set_state(ObjectState(position=p_left, rotation=rotation_matrix, linear_velocity=np.zeros(3)))
    target_right.set_state(ObjectState(position=p_right, rotation=rotation_matrix, linear_velocity=np.zeros(3)))
    target_top_spring.set_state(ObjectState(position=p_top, rotation=rotation_matrix, linear_velocity=np.zeros(3)))

def orientation_from_points(attachment, left, right):
    """Reconstruct a right-handed frame from the three connection points.

    x points from right to left. z points from the top point toward the
    midpoint of the lateral points, after removing any x component. y=z cross
    x completes the orthogonal frame. Each returned column is a frame axis
    expressed in world/base coordinates.
    """
    x_axis = np.asarray(left, dtype=float) - right
    x_axis /= np.linalg.norm(x_axis)
    z_axis = (np.asarray(left, dtype=float) + right) / 2.0 - attachment
    z_axis -= np.dot(z_axis, x_axis) * x_axis
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))

def main():
    """Set up ROS, the virtual mechanism, and the torque computation loop."""
    # Startup prerequisite: gripper_node.py must complete its activation and
    # return to the open position before this script is launched. The waits
    # below cover the first camera pose and robot state, not gripper readiness.
    # Configuration is kept on the command line so the ROS topic names and
    # mechanism gains can be changed without editing the model file. Topic
    # names must match the physical controller and camera publishers.
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
    # A nonpositive state timeout would make every measurement immediately
    # stale and would suppress all virtual mechanism torque.
    if args.state_timeout <= 0:
        parser.error("state-timeout must be positive")

    # Only APPROACH can issue a close request. The two thresholds must both be
    # satisfied before entering CLOSING: TCP-to-target distance and the worst
    # of the three individual connection distances.
    phase = "APPROACH"
    position_tolerance = 0.01 # 1 cm TCP position error.
    connection_tolerance = 0.01 # 1 cm maximum connection error.

    # Resolve the Menagerie root inside the VMC container, then load the
    # combined FR3/Robotiq scene. MjSpec is editable; MjModel is its compiled
    # form. The three control sites must be attached before spec.compile().
    menagerie = resolve_menagerie_path()
    scene_xml = menagerie / "fr3_robotiq_2f_140" / "scene_fr3_robotiq.xml"
    if not scene_xml.is_file():
        raise FileNotFoundError(f"Modello non trovato: {scene_xml}")


    spec = mujoco.MjSpec.from_file(str(scene_xml))
    build_ee_link_model(spec, args.link_length)
    model = spec.compile()
    # XML actuators are disabled because this script supplies joint torques
    # from the virtual connections instead of using the model's own controls.
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION
    # MjModel holds compiled geometry and parameters; MjData holds the current
    # robot state and kinematic results. Since no mj_step is called here, this
    # timestep only supplies the pacing interval for the Python loop.
    model.opt.timestep = args.timestep
    data = mujoco.MjData(model)
    sync = RobotStateSynchronizer(model)


    # The synchronizer maps ROS joint names to MuJoCo qpos/qvel addresses. The
    # bridge receives JointState messages and publishes torque messages on its
    # own ROS executor thread; the main loop reads its latest state.
    bridge = RobotROSBridge(
        synchronizer= sync,
        sub_topic=args.topic,
        pub_torque_topic=args.torque_pub_topic,
        pub_rate_hz=args.pub_rate,
    )

    bridge.start()

    # --- ROS callback state and locks ---
    # Callback functions execute on the bridge's ROS thread. Each lock protects
    # a short update/read of the most recent message; MuJoCo calculations stay
    # in the main thread outside the locks.
    camera_lock = Lock()
    latest_camera_pose = None
    last_gripper_detection = None
    gripper_lock = Lock()
    last_gripper_status = None

    # --- ROS callback functions ---
    def camera_callback(msg):
        """Store the newest finite marker pose received in the robot base frame."""
        nonlocal latest_camera_pose

        # Camera coordinates are usable here only after zivid_camera_h2.py has
        # transformed them into the same base frame used by the MuJoCo model.
        if msg.header.frame_id != "base":
            return

        p = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ], dtype=float)

        # geometry_msgs stores x,y,z,w; MuJoCo's quaternion routines use
        # w,x,y,z, so reorder while constructing the array.
        q = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
        ], dtype=float)

        # Reject NaN/Inf and a nearly zero quaternion. Normalization prevents
        # scale errors when converting the quaternion to a rotation matrix.
        norm = np.linalg.norm(q)
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)) or norm < 1e-6:
            return

        with camera_lock:
            # The monotonic receive time is later used to check camera data
            # freshness; it is separate from the ROS header timestamp.
            latest_camera_pose = (p, q / norm, time.perf_counter())

    def gripper_obj_det_cb(msg: Bool):
        """Store object-detection feedback and its local receive time."""
        nonlocal last_gripper_detection
        with gripper_lock:
            last_gripper_detection = (msg.data, time.perf_counter())

    def gripper_status_cb(msg: UInt8MultiArray):
        """Store the four raw gripper status fields when the message is valid."""
        nonlocal last_gripper_status
        if len(msg.data) != 4:
            return
        with gripper_lock:
            last_gripper_status = (tuple(msg.data), time.perf_counter())

    # --- ROS publishers and subscribers owned by this script ---
    # The bridge itself already subscribes to joint states and publishes the
    # computed joint torques. These additional endpoints handle the camera
    # target and the gripper command/feedback.
    gripper_pub = bridge.node.create_publisher(Float64, "/hand/finger_distance", 10)
    camera_sub = bridge.node.create_subscription(PoseStamped, args.camera_topic, camera_callback, 1)
    gripper_sub = bridge.node.create_subscription(Bool, "/gripper/object_detected", gripper_obj_det_cb, 10)
    gripper_status_sub = bridge.node.create_subscription(UInt8MultiArray, "/gripper/status", gripper_status_cb, 10)

    # --- Wait for initial measured inputs ---
    # The first camera pose defines the initial virtual target; starting from
    # arbitrary placeholder target coordinates would create misleading forces.
    first_camera_pose = None
    while first_camera_pose is None:
        with camera_lock:
            first_camera_pose = latest_camera_pose

        if first_camera_pose is None:
            time.sleep(0.05)

    # The virtual robot must also begin at the measured physical robot state.
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


    # Copy the first measured q and dq into MjData. The damper needs joint
    # velocities from the beginning. forward=True refreshes MuJoCo kinematics
    # after this assignment; later messages go through apply_incoming_state().
    sync.set_state(data, initial_qpos, initial_qvel, forward=True)

    # Only now can the camera pose be converted into initial virtual targets.
    # The three target objects are held in Python, not inserted into the XML.
    target_top_position, target_left_position, target_right_position, R_base_aruco  = build_positions(first_camera_pose, args.link_length)
    target_top_spring, target_left, target_right = build_ref_objects(target_top_position, target_left_position, target_right_position, R_base_aruco)

    # Resolve compiled site IDs once. The TCP is the contact/error point; the
    # other three sites are where the virtual spring/damper forces act.
    tcp_id = model.site("tcp").id
    top_spring_id = model.site("ee_top_spring").id
    left_grip_id = model.site("gripper_left_finger").id
    right_grip_id = model.site("gripper_right_finger").id
    # MuJoCoSite wraps each compiled site so the VMC library can obtain its
    # world position/velocity and map a Cartesian force to generalized joint
    # force through the site's Jacobian transpose.
    r_top_site = MuJoCoSite(top_spring_id, "top_site")
    r_left_site = MuJoCoSite(left_grip_id, "left_finger")
    r_right_site = MuJoCoSite(right_grip_id, "right_finger")

    # Each connection pairs one robot site with one camera-derived target.
    # TanhSpring uses a smooth, component-wise force saturation; LinearDamper
    # opposes their relative linear velocity. max_force limits each spring
    # component, not the sum of the three forces or any resulting joint torque.
    # The damper contribution is not limited by that spring parameter.
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

    # Refresh site poses/Jacobians from the initial robot state. perf_counter()
    # is monotonic and is used for age/timeout checks; time.time() only paces
    # the human-readable diagnostic printout.
    mujoco.mj_forward(model, data)
    last_telemetry_log = time.time()
    last_state_time = time.perf_counter()
    state_timeout_logged = False
    dt = model.opt.timestep

    # --- Grasp decision thresholds and freshness timers ---
    # A close request requires both geometric errors to remain small for
    # 0.5 s with recent robot and camera messages. These timers are based on
    # the local monotonic clock, not on ROS message timestamps.
    t_arrival = None
    last_camera_time = first_camera_pose[2]
    camera_timeout = 0.8 # Seconds without a new camera pose before it is stale.
    closing_timeout = 5.0 # Initial closing timeout; tune from measured gripper motion.

    def step_iteration():

        """Read measured inputs, update virtual targets, and buffer torques.

        The physical robot is the source of q and dq. No mj_step is called:
        mechanism.apply evaluates the three virtual connections at the latest
        measured state. The returned bool requests one gripper close command
        from the outer loop after the arrival condition has held long enough.
        """
        nonlocal last_telemetry_log, last_state_time, state_timeout_logged, latest_camera_pose, phase, position_tolerance, connection_tolerance, t_arrival, last_camera_time
        # --- Read the newest ROS measurements ---
        # Consume the most recent JointState, if available. With forward=True,
        # the synchronizer refreshes MuJoCo site poses and velocities using
        # mj_forward after it copies the measured joint positions/velocities.
        state_updated = bridge.apply_incoming_state(data, forward=True)
        # Copy and clear the one-message camera buffer under the lock. Keeping
        # the lock short avoids holding up the ROS callback thread.
        with camera_lock:
            camera_pose = latest_camera_pose
            latest_camera_pose = None
        
        # Update the three Python-side targets only for a newly received pose.
        # No camera message means they retain their last positions; the
        # freshness check below can still reject an old target for grasping.
        if camera_pose is not None:
            last_camera_time = camera_pose[2]
            new_p_top, new_p_left, new_p_right, new_rotation = build_positions(camera_pose, args.link_length)
            update_positions(new_p_top, new_p_left, new_p_right, new_rotation, target_top_spring, target_left, target_right)

        if state_updated:
            # This receive time is used for the controller's stale-state gate.
            last_state_time = time.perf_counter()
            state_timeout_logged = False

        # --- Compare measured gripper sites with the virtual targets ---
        # The active StaticObjects are the source of truth for both errors.
        target_att_pos = target_top_spring.state(model, data).position
        target_left_pos = target_left.state(model, data).position
        target_right_pos = target_right.state(model, data).position
        desired_tip_pos = (target_left_pos + target_right_pos) / 2.0
        # The lateral target midpoint is the desired TCP. site_xpos is already
        # expressed in MuJoCo world coordinates, aligned here with "base".
        error_xyz = desired_tip_pos - data.site_xpos[tcp_id]

        # The three points also define a target frame and a measured gripper
        # frame for diagnostics. No orientation controller is computed here;
        # the three translational connections influence orientation indirectly.
        target_rotation = orientation_from_points(target_att_pos, target_left_pos, target_right_pos)
        measured_rotation = orientation_from_points(
        data.site_xpos[top_spring_id],
        data.site_xpos[left_grip_id],
        data.site_xpos[right_grip_id],
        )

        # Distances for the individual spring endpoints are checked as well
        # as the TCP error so a small TCP error alone cannot trigger grasping.
        connection_errors = [
            np.linalg.norm(target_att_pos - data.site_xpos[top_spring_id]),
            np.linalg.norm(target_left_pos - data.site_xpos[left_grip_id]),
            np.linalg.norm(target_right_pos - data.site_xpos[right_grip_id]),
        ]

        if state_updated and (time.time() - last_telemetry_log > 1.0):

            # R_target.T @ R_measured expresses the orientation difference in
            # target-frame coordinates. The negative rotation vector reports
            # the signed correction around those target x, y and z axes.
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


        # --- Arrival and grasp-state decisions ---
        # Both geometric conditions must hold continuously; the APPROACH gate
        # below additionally requires recent robot and camera data.
        within_tolerance = (np.linalg.norm(error_xyz) < position_tolerance and max(connection_errors) < connection_tolerance)
        close_requested = False

        # perf_counter is monotonic, so elapsed-time comparisons are not
        # affected by wall-clock changes on the computer.
        t_actual = time.perf_counter()
        robot_state_recent = t_actual - last_state_time <= args.state_timeout
        camera_data_recent = t_actual - last_camera_time <= camera_timeout
        if phase == "APPROACH":
            # Start a dwell timer on first arrival. Reset it whenever either
            # error or input freshness leaves the allowed region.
            if within_tolerance and robot_state_recent and camera_data_recent:
                if t_arrival is None:
                    t_arrival = t_actual

                elif t_actual - t_arrival >= 0.5:
                    # The phase change prevents a second close request on
                    # following iterations. The outer loop sends the command.
                    phase = "CLOSING"
                    close_requested = True
                    t_arrival = None
            else:
                t_arrival = None
        
        # Read both gripper feedback buffers atomically with respect to their
        # callbacks; the following checks run after the lock is released.
        with gripper_lock:
            detection = last_gripper_detection
            status = last_gripper_status

        if phase == "CLOSING" and t_close_request is not None:
            # Only feedback received after this close command is accepted.
            # gSTA=3: active; gOBJ=2: object stopped closing; gFLT=0: no fault.
            if status is not None and detection is not None:
                values, t_status = status
                g_sta, g_obj, g_flt, g_cu = values
                detected, t_detection = detection
                if t_status > t_close_request and t_detection > t_close_request:
                    if detected and g_sta == 3 and g_obj == 2 and g_flt == 0:
                        phase = "GRASPED"

            # A closing attempt that never reports a valid grasp expires.
            if phase == "CLOSING" and t_actual - t_close_request >= closing_timeout:
                phase = "FAILED"


        # --- Compute and publish the virtual mechanism torque ---
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

        # The caller owns the actual /hand/finger_distance publication.
        return close_requested

    # --- Real-time paced execution and optional viewer ---
    # step_iteration() is the only consumer of the bridge's latest joint state.
    # The loop uses wall-clock pacing; MuJoCo dynamics are not stepped here.
    t_start = time.perf_counter()
    next_tick = t_start
    # This one-shot command asks the gripper node to close to its minimum
    # accepted finger distance. The gripper node performs the actual motion.
    close_msg = Float64()
    close_msg.data = 0.001
    # Set only after publishing the close request. Feedback with an earlier
    # receive time must not be mistaken for the result of this grasp attempt.
    t_close_request = None
    try:
        if args.headless:
            # Run without rendering until the requested wall-clock duration;
            # a nonpositive duration means continue until interrupted.
            while args.duration <= 0 or time.perf_counter() - t_start < args.duration:
                close_request = step_iteration()
                if close_request:
                    gripper_pub.publish(close_msg)
                    t_close_request = time.perf_counter()
                # Sleep only for the remaining fraction of this tick. If the
                # calculation ran late, restart pacing from the current time
                # rather than accumulating a growing scheduling backlog.
                next_tick += dt
                remaining = next_tick - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    next_tick = time.perf_counter()
        else:
            # The passive viewer shows the last measured MjData state. It does
            # not command the robot; ROS telemetry still determines q and dq.
            next_viewer_sync = t_start
            with viewer.launch_passive(model, data) as v:
                print("[INFO] Passive viewer active. Double-right-click and drag the robot end-effector!")
                while v.is_running() and (args.duration <= 0 or time.perf_counter() - t_start < args.duration):
                    close_request = step_iteration()
                    if close_request:
                        gripper_pub.publish(close_msg)
                        t_close_request = time.perf_counter()

                    now = time.perf_counter()
                    # Rendering is limited to about 30 Hz while the control
                    # loop follows the shorter --timestep pacing interval.
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
        # Stop the ROS bridge executor/publishers when either execution mode
        # exits, including an interrupted run.
        bridge.stop()

if __name__ == "__main__":
    main()

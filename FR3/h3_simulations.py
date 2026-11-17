"""Run repeated H3 pick-and-place on PC2 with a real camera and MuJoCo.

The geometry, explicit connections, phases and step_iteration() follow
h3_pick_place.py. The only ROS inputs are the cube and table PoseStamped
topics in frame "base". No physical robot/gripper commands or parameter
requests are created; the ROS bridge runs only the camera subscriptions.

data computes VMC from data_sim, the dynamic FR3/Robotiq substitute.
Only data_sim is integrated; its torques include model bias and joint limits.

The existing gripper_position equality animates the fingers. Local 10 Hz
feedback emulates GRASPED when the closing angle is reached. This tests
the sequence, not contact-based grasping, object transport or grasp force.
The camera does not create a physical cube or table in the MuJoCo scene.

A virtual robot cannot move the physical cube marker. After a simulated
place, WAIT_FOR_MOVE therefore measures displacement from the last observed
physical cube position, rather than from the virtual release target. Fresh
camera observations must span the same restart dwell used by real H3.

Example in the PC2 VMC shell, with zivid_camera_h3.py running on the host:
    /opt/venv/bin/python h3_simulations.py \
        --scene /workspace/VMControl/fr3_robotiq_2f_140/fr3_robotiq_2f_140/scene_fr3_robotiq.xml
"""

import argparse
from contextlib import nullcontext
from pathlib import Path
import time
from threading import Lock

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from mujoco_vmc import (
    LinearDamper,
    MuJoCoSite,
    ObjectState,
    RobotStateSynchronizer,
    TanhSpring,
    VirtualConnection,
    VirtualMechanism,
    StaticObject,
    resolve_menagerie_path,
)


def draw_sphere(scene, pos, radius=0.015, rgba=None):
    """Draw a viewer-only target without adding physical model geometry."""
    if scene.ngeom >= scene.maxgeom:
        return False
    color = [0.2, 0.6, 1.0, 0.9] if rgba is None else rgba
    mujoco.mjv_initGeom(
        scene.geoms[scene.ngeom],
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=np.array([radius, 0.0, 0.0], dtype=float),
        pos=np.asarray(pos, dtype=float),
        mat=np.eye(3).ravel(),
        rgba=np.asarray(color, dtype=np.float32),
    )
    scene.ngeom += 1
    return True


def draw_capsule_line(scene, p1, p2, radius=0.0035, rgba=None):
    """Draw one visual connection using MuJoCo's native connector helper."""
    if scene.ngeom >= scene.maxgeom:
        return False
    start = np.asarray(p1, dtype=float)
    end = np.asarray(p2, dtype=float)
    if np.linalg.norm(end - start) < 1e-4:
        return False
    color = [0.1, 0.9, 0.3, 0.8] if rgba is None else rgba
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        size=np.zeros(3),
        pos=np.zeros(3),
        mat=np.eye(3).ravel(),
        rgba=np.asarray(color, dtype=np.float32),
    )
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, start, end)
    scene.ngeom += 1
    return True


def build_ee_link_model(
    spec,
    link_length: float = 0.8,
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

def build_positions(camera_pose, link_length, height_offset=0.03, yaw_sign=1):
    """Build the three target points from one measured base-frame marker pose.

    camera_pose contains (position, quaternion, receive_time). The position is
    in metres in the FR3 base frame; the quaternion uses MuJoCo order wxyz.
    The returned points are also in base/world coordinates, so they can be
    compared directly with MuJoCo's site_xpos values.
    height_offset raises the TCP and all three anchors along base +z. The
    default is 3 cm for picking; H3 passes zero for its separate place offset.
    yaw_sign=-1 selects the equivalent 180-degree grasp about marker z.
    """
    L = float(link_length)
    p_marker, q_marker, received_at = camera_pose

    # Convert the marker orientation to a rotation matrix. Its columns are
    # the marker's x, y and z axes expressed in the base frame.
    rotation_flat = np.empty(9)
    mujoco.mju_quat2Mat(rotation_flat, q_marker)  # MuJoCo quaternion order: w, x, y, z.
    R_base_aruco = rotation_flat.reshape(3, 3)
    # Flip both in-plane axes together to preserve a right-handed frame.
    R_base_aruco[:, :2] *= yaw_sign

    x_aruco_in_base = R_base_aruco[:, 0]
    z_aruco = R_base_aruco[:, 2]
    # Raise the pick TCP along base +z, independently of the marker orientation.
    # Side targets lie along marker x. The upper robot site is at -L along TCP z,
    # so matching TCP +z to marker +z puts its target along marker -z.
    h = float(height_offset)
    p_target = p_marker + np.array([0.0, 0.0, h])
    p_left = p_target + 0.30 * x_aruco_in_base
    p_right = p_target - 0.30 * x_aruco_in_base
    # For the measured lab marker (+z downward), this point is above the marker.
    p_top = p_target - L * z_aruco

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


def select_yaw_sign(camera_pose, link_length, current_top, current_left, current_right):
    """Choose the nearer equivalent yaw once at the start of a motion.

    The candidates share z and differ by 180 degrees about it. Maximizing
    the rotation trace selects the smaller angle from the measured gripper.
    The caller keeps this sign fixed while subsequent camera poses arrive.
    """
    target_top, target_left, target_right, _ = build_positions(camera_pose, link_length)
    target_rotation = orientation_from_points(target_top, target_left, target_right)
    current_rotation = orientation_from_points(current_top, current_left, current_right)
    alignment = np.sum(target_rotation[:, :2] * current_rotation[:, :2])
    return 1 if alignment >= 0.0 else -1


def main():
    """Set up the two camera inputs, dynamic model and repeated H3 workflow."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, help="combined FR3/Robotiq scene XML; otherwise use MUJOCO_MENAGERIE_PATH")
    parser.add_argument("--initial-keyframe", default="home")
    parser.add_argument("--camera-topic", default="/vision/aruco_pose")
    parser.add_argument("--camera-table-topic", default="/vision/aruco_table_pose")
    parser.add_argument("--camera-timeout", type=float, default=0.8)
    parser.add_argument("--link-length", type=float, default=0.8)
    parser.add_argument("--target-radius", type=float, default=0.015)
    # Match real H3 defaults; command-line overrides permit direct comparisons.
    parser.add_argument("--stiffness", type=float, default=2000.0)
    parser.add_argument("--stiffness2", type=float, default=20.0)
    parser.add_argument("--damping", type=float, default=50.0)
    parser.add_argument("--max-force", type=float, default=2.0)
    parser.add_argument("--max-force2", type=float, default=8.0)
    parser.add_argument("--timestep", type=float, default=0.001)
    parser.add_argument("--close-time", type=float, default=1.0, help="simulated closing ramp duration (s)")
    parser.add_argument("--open-time", type=float, default=1.0, help="simulated opening ramp duration (s)")
    parser.add_argument("--restart-distance", type=float, default=0.05,
                        help="physical cube-marker displacement after a simulated cycle (m)")
    parser.add_argument("--restart-dwell", type=float, default=0.5)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="wall-clock seconds after initialization; <= 0 runs until stopped")
    args = parser.parse_args()
    values = (args.stiffness, args.stiffness2, args.damping, args.max_force,
              args.max_force2, args.link_length, args.timestep, args.target_radius,
              args.camera_timeout, args.close_time, args.open_time,
              args.restart_distance, args.restart_dwell, args.duration)
    if not np.all(np.isfinite(values)):
        parser.error("numeric arguments must be finite")
    if min(args.stiffness, args.stiffness2, args.damping) < 0:
        parser.error("stiffness, stiffness2 and damping must be nonnegative")
    if min(args.max_force, args.max_force2, args.link_length, args.timestep,
           args.target_radius, args.camera_timeout, args.close_time,
           args.open_time, args.restart_distance, args.restart_dwell) <= 0:
        parser.error("force limits, lengths, timestep, radius and time intervals must be positive")

    # Keep ROS optional when importing the geometric helpers or requesting help.
    from geometry_msgs.msg import PoseStamped
    from mujoco_vmc import RobotROSBridge

    phase = "APPROACH"
    position_tolerance = 0.01
    connection_tolerance = 0.01

    # --- Load the same model and create virtual and dynamic states ---
    scene_xml = (args.scene.expanduser().resolve() if args.scene is not None else
                 resolve_menagerie_path() / "fr3_robotiq_2f_140" / "scene_fr3_robotiq.xml")
    if not scene_xml.is_file():
        raise FileNotFoundError(f"Scene not found: {scene_xml}; pass --scene with its actual path")
    spec = mujoco.MjSpec.from_file(str(scene_xml))
    build_ee_link_model(spec, args.link_length)
    model = spec.compile()
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION
    model.opt.timestep = args.timestep
    data = mujoco.MjData(model)
    data_sim = mujoco.MjData(model)
    sync = RobotStateSynchronizer(model)
    if sync.num_joints != 7:
        raise ValueError("Expected exactly seven FR3 arm joints")
    qpos_ids = np.asarray(sync.qpos_indices, dtype=int)
    dof_ids = np.asarray(sync.qvel_indices, dtype=int)
    torque_limits = model.jnt_actfrcrange[sync.joint_ids].copy()
    if np.any(torque_limits[:, 0] >= torque_limits[:, 1]):
        raise ValueError("The scene must define actuatorfrcrange limits for all seven arm joints")
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.initial_keyframe)
    if key_id < 0:
        raise ValueError(f"Unknown initial keyframe: {args.initial_keyframe}")
    mujoco.mj_resetDataKeyframe(model, data_sim, key_id)
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data_sim)
    sync.set_state(data, data_sim.qpos[qpos_ids], data_sim.qvel[dof_ids], forward=True)
    finger_qpos_id = model.jnt_qposadr[model.joint("finger_joint").id]
    gripper_eq_id = model.equality("gripper_position").id

    # --- Camera buffers and local emulated gripper feedback ---
    camera_lock = Lock()
    latest_camera_pose = None
    latest_camera_table_pose = None
    last_cube_observation = None
    last_table_observation = None
    gripper_lock = Lock()
    last_gripper_detection = None
    last_gripper_status = None
    last_gripper_distance = None

    def camera_callback(msg):
        """Store a finite normalized cube pose in the base frame."""
        nonlocal latest_camera_pose, last_cube_observation
        if msg.header.frame_id != "base":
            return
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        q = np.array([msg.pose.orientation.w, msg.pose.orientation.x,
                      msg.pose.orientation.y, msg.pose.orientation.z], dtype=float)
        norm = np.linalg.norm(q)
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)) or not np.isfinite(norm) or norm < 1e-6:
            return
        with camera_lock:
            latest_camera_pose = (p, q / norm, time.perf_counter())
            last_cube_observation = latest_camera_pose

    def camera_table_callback(msg):
        """Store the independently detected table pose in the base frame."""
        nonlocal latest_camera_table_pose, last_table_observation
        if msg.header.frame_id != "base":
            return
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        q = np.array([msg.pose.orientation.w, msg.pose.orientation.x,
                      msg.pose.orientation.y, msg.pose.orientation.z], dtype=float)
        norm = np.linalg.norm(q)
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)) or not np.isfinite(norm) or norm < 1e-6:
            return
        with camera_lock:
            latest_camera_table_pose = (p, q / norm, time.perf_counter())
            last_table_observation = latest_camera_table_pose

    # No sub_topic or pub_*_topic: this bridge has no hardware command endpoints.
    bridge = RobotROSBridge(synchronizer=sync, node_name="h3_simulations")
    viewer_started = False
    try:
        bridge.start()
        if bridge.node is None:
            raise RuntimeError("The ROS bridge did not start")
        camera_sub = bridge.node.create_subscription(PoseStamped, args.camera_topic, camera_callback, 1)
        camera_table_sub = bridge.node.create_subscription(PoseStamped, args.camera_table_topic, camera_table_callback, 1)
        print(f"[SIM] Scene: {scene_xml}")
        print(f"[ROS] Cube input: {args.camera_topic}; table input: {args.camera_table_topic}; frame_id=base")
        print(
            f"[SIM] stiffness={args.stiffness}, stiffness2={args.stiffness2}, "
            f"damping={args.damping}, max_force={args.max_force}, max_force2={args.max_force2}"
        )
        print("[SIM] Arm and gripper commands stay inside MuJoCo; no hardware commands are published")
        print("[SIM] GRASPED is emulated from finger angle; object contacts and grasp force are not tested")
        print("[SIM] After each virtual place, move the physical cube marker to start another cycle")

        # As in H3, construct all targets from the first valid cube observation.
        first_camera_pose = None
        print("[WAIT] Waiting for the first valid base-frame cube pose")
        while first_camera_pose is None:
            with camera_lock:
                first_camera_pose = latest_camera_pose
            if first_camera_pose is None:
                time.sleep(0.05)
        # Resolve compiled site IDs once. The TCP is the contact/error point; the
        # other three sites are where the virtual spring/damper forces act.
        tcp_id = model.site("tcp").id
        top_spring_id = model.site("ee_top_spring").id
        left_grip_id = model.site("gripper_left_finger").id
        right_grip_id = model.site("gripper_right_finger").id

        # Choose the nearer symmetric grasp once; camera updates retain this branch.
        pick_yaw_sign = select_yaw_sign(
            first_camera_pose, args.link_length, data.site_xpos[top_spring_id],
            data.site_xpos[left_grip_id], data.site_xpos[right_grip_id],
        )
        print(f"[TARGET] Pick yaw branch: {0 if pick_yaw_sign == 1 else 180} deg; fixed for this approach")
        target_top_position, target_left_position, target_right_position, R_base_aruco = build_positions(
            first_camera_pose, args.link_length, yaw_sign=pick_yaw_sign
        )
        target_top_spring, target_left, target_right = build_ref_objects(
            target_top_position, target_left_position, target_right_position, R_base_aruco
        )

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
        conn_att_2 = VirtualConnection(
            source=r_top_site,
            target=target_top_spring,
            components=[
                TanhSpring(args.stiffness2, max_force=args.max_force2),
            ],
            name="link_att_spring",
        )
        conn_left_2 = VirtualConnection(
            source=r_left_site,
            target=target_left,
            components=[
                TanhSpring(args.stiffness2, max_force=args.max_force2),
            ],
            name="link_left_spring",
        )
        conn_right_2 = VirtualConnection(
            source=r_right_site,
            target=target_right,
            components=[
                TanhSpring(args.stiffness2, max_force=args.max_force2),
            ],
            name="link_right_spring",
        )
        mechanism = VirtualMechanism([conn_att,conn_left, conn_right])
        mechanism_2 = VirtualMechanism([conn_att_2,conn_left_2, conn_right_2])

        mujoco.mj_forward(model, data)
        last_telemetry_log = time.time()
        t_arrival = None
        last_camera_time = first_camera_pose[2]
        camera_timeout = args.camera_timeout
        camera_hold_active = False
        closing_timeout = 5.0
        gripper_timeout = 0.5
        grasp_hold_pending = False
        failure_hold_active = False
        opening_timeout = 5.0
        gripper_open_width = 0.140
        opening_tolerance = 0.005
        place_arrival_dwell = 0.5
        placed_cube_position = None
        t_place_completed = None
        t_cube_moved = None
        completed_cycles = 0

        # The existing equality drives the fingers, as in h2_camera_simulation.py.
        sim_gripper_start = float(data_sim.qpos[finger_qpos_id])
        sim_gripper_target = 0.0
        sim_gripper_request_time = None
        sim_feedback_next = time.perf_counter()
        sim_feedback_period = 0.1
        sim_closed_angle = 0.7
        sim_angle_tolerance = 0.02
        t_close_request = None
        t_open_request = None
        dt = model.opt.timestep

        def step_iteration():

            """Read camera inputs, update H3 phases and step the dynamic robot.

            q and dq come from data_sim. The H3 decisions and both mechanisms stay
            explicit. The outer loop handles local finger requests; mj_step below
            advances only data_sim and no hardware command is published.
            """
            nonlocal last_telemetry_log, latest_camera_pose, latest_camera_table_pose, phase, position_tolerance, connection_tolerance, t_arrival, last_camera_time
            nonlocal camera_hold_active, grasp_hold_pending, failure_hold_active
            nonlocal placed_cube_position, t_place_completed, t_cube_moved, completed_cycles
            nonlocal t_close_request, t_open_request, pick_yaw_sign
            nonlocal last_gripper_detection, last_gripper_status, last_gripper_distance, sim_feedback_next
            # Copy the dynamic plant state instead of receiving physical FR3 states.
            mujoco.mj_forward(model, data_sim)
            sync.set_state(data, data_sim.qpos[qpos_ids], data_sim.qvel[dof_ids], forward=True)
            state_updated = True
            # Copy and clear the one-message camera buffer under the lock. Keeping
            # the lock short avoids holding up the ROS callback thread.
            with camera_lock:
                camera_pose = latest_camera_pose
                latest_camera_pose = None

            if camera_pose is not None:
                last_camera_time = camera_pose[2]

            t_actual = time.perf_counter()
            # Local plant state is available every iteration; PC1 loss is not tested.
            robot_state_recent = True
            camera_data_recent = t_actual - last_camera_time <= camera_timeout

            # --- Local finger motion and synthetic 10 Hz gripper feedback ---
            # Reaching the closed angle emulates contact; no object is sensed here.
            if sim_gripper_request_time is None:
                fraction = 1.0
                model.eq_data[gripper_eq_id, 0] = sim_gripper_target
            else:
                travel_time = args.close_time if sim_gripper_target > sim_gripper_start else args.open_time
                fraction = np.clip((t_actual - sim_gripper_request_time) / travel_time, 0.0, 1.0)
                model.eq_data[gripper_eq_id, 0] = (
                    sim_gripper_start + fraction * (sim_gripper_target - sim_gripper_start)
                )
            if t_actual >= sim_feedback_next:
                angle = float(data_sim.qpos[finger_qpos_id])
                closed_reached = (
                    sim_gripper_target > 0.35 and fraction >= 1.0
                    and abs(angle - sim_closed_angle) < sim_angle_tolerance
                )
                open_reached = (
                    sim_gripper_target <= 0.35 and fraction >= 1.0
                    and abs(angle) < sim_angle_tolerance
                )
                g_obj_sim = 2 if closed_reached else 3 if open_reached else 0
                width_sim = gripper_open_width * (1.0 - np.clip(angle / sim_closed_angle, 0.0, 1.0))
                with gripper_lock:
                    last_gripper_detection = (bool(closed_reached), t_actual)
                    last_gripper_status = ((3, g_obj_sim, 0, 0), t_actual)
                    last_gripper_distance = (float(width_sim), t_actual)
                sim_feedback_next = t_actual + sim_feedback_period


            # If the camera is lost during APPROACH, capture the current measured
            # anchors once. Copy the arrays so future robot motion cannot move the
            # held targets through a view into data.site_xpos. Local state is
            # refreshed every iteration by the dynamic model.
            if phase == "APPROACH" and not camera_data_recent:
                if not camera_hold_active and robot_state_recent:
                    hold_top = data.site_xpos[top_spring_id].copy()
                    hold_left = data.site_xpos[left_grip_id].copy()
                    hold_right = data.site_xpos[right_grip_id].copy()
                    hold_rotation = orientation_from_points(hold_top, hold_left, hold_right)
                    update_positions(hold_top, hold_left, hold_right, hold_rotation,
                                     target_top_spring, target_left, target_right)
                    camera_hold_active = True
                    print("[CAMERA] Pose timed out during APPROACH; holding the current end-effector pose")
            elif phase in ("APPROACH", "CLOSING") and camera_pose is not None:
                # Track the cube only while approaching or closing the fingers.
                # The transfer and later phases retain their selected place target.
                new_p_top, new_p_left, new_p_right, new_rotation = build_positions(
                    camera_pose, args.link_length, yaw_sign=pick_yaw_sign
                )
                update_positions(new_p_top, new_p_left, new_p_right, new_rotation,
                                 target_top_spring, target_left, target_right)
                if camera_hold_active:
                    camera_hold_active = False
                    print("[CAMERA] Fresh pose received; tracking the ArUco again")
            elif phase == "GRASPED":
                with camera_lock:
                    aruco_table_pose = latest_camera_table_pose
                    latest_camera_table_pose = None
                if aruco_table_pose is not None:
                    latest_aruco_table_time = aruco_table_pose[2]
                    aruco_table_recent = t_actual - latest_aruco_table_time <= camera_timeout

                    if aruco_table_recent and robot_state_recent:
                        # Select once for this transfer, independently of the cube marker.
                        place_yaw_sign = select_yaw_sign(
                            aruco_table_pose, args.link_length, data.site_xpos[top_spring_id],
                            data.site_xpos[left_grip_id], data.site_xpos[right_grip_id],
                        )
                        print(f"[TARGET] Place yaw branch: {0 if place_yaw_sign == 1 else 180} deg; fixed for this transfer")
                        place_top, place_left, place_right, place_rotation = build_positions(
                            aruco_table_pose, args.link_length, height_offset=0.0, yaw_sign=place_yaw_sign
                        )
                        # Shift all three targets upward in the base frame.
                        place_offset = np.array([0.0, 0.0, 0.07])
                        place_top += place_offset
                        place_left += place_offset
                        place_right += place_offset
                        update_positions(place_top, place_left, place_right, place_rotation, target_top_spring, target_left, target_right)
                        phase = "TRANSFER"
                        t_arrival = None
                        print("[INFO] TRANSFER phase started; fixed TCP target is 0.07 m above the table marker")



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

                target_mode = (
                    "HOLD_PENDING" if grasp_hold_pending else
                    "HOLD" if camera_hold_active or failure_hold_active else
                    "CUBE" if phase in ("APPROACH", "CLOSING") else "TABLE"
                )
                print(
                    f"[SIM] phase={phase}; cycles={completed_cycles}; sim_time={data_sim.time:.2f}s; target_mode={target_mode}; "
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
            open_requested = False

            # Freshness was evaluated before selecting the targets. An expired
            # camera inhibits closing even when the held-pose error is near zero.
            if phase == "APPROACH":
                # Start a dwell timer on first arrival. Reset it whenever either
                # error or input freshness leaves the allowed region.
                if within_tolerance and robot_state_recent and camera_data_recent:
                    if t_arrival is None:
                        t_arrival = t_actual
                        print("[ARRIVAL] Pick conditions satisfied; starting 0.5 s timer")

                    elif t_actual - t_arrival >= 0.5:
                        # The phase change prevents a second close request on
                        # following iterations. The outer loop sends the command.
                        phase = "CLOSING"
                        print("[INFO] CLOSING phase started")
                        close_requested = True
                        t_arrival = None
                else:
                    if t_arrival is not None:
                        print(
                            f"[ARRIVAL] Timer cancelled: "
                            f"within_tolerance={within_tolerance}, "
                            f"robot_recent={robot_state_recent}, "
                            f"camera_recent={camera_data_recent}"
                        )
                    t_arrival = None

            # Read both gripper feedback buffers atomically with respect to their
            # callbacks; the following checks run after the lock is released.
            with gripper_lock:
                detection = last_gripper_detection
                status = last_gripper_status
                distance = last_gripper_distance

            feedback_time = time.perf_counter()
            gripper_is_open = False
            if distance is not None and status is not None:
                width, t_distance = distance
                values, t_status = status
                gripper_is_open = (
                    feedback_time - t_distance <= gripper_timeout
                    and feedback_time - t_status <= gripper_timeout
                    and values[0] == 3 and values[1] == 3 and values[2] == 0
                    and abs(width - gripper_open_width) <= opening_tolerance
                )

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
                            print("[GRIPPER SIM] GRASPED: emulated finger-angle confirmation, not object detection")

                # A closing attempt that never reports a valid grasp expires.
                if phase == "CLOSING" and t_actual - t_close_request >= closing_timeout:
                    phase = "FAILED"
                    grasp_hold_pending = True
                    print("[INFO] CLOSING phase failed")

            # Monitor the grasp on every iteration, independently of CLOSING.
            if phase in ("GRASPED", "TRANSFER"):
                feedback_time = time.perf_counter()
                grasp_failure_reason = None
                if detection is None or status is None:
                    grasp_failure_reason = "gripper feedback missing"
                else:
                    detected, t_detection = detection
                    values, t_status = status
                    g_sta, g_obj, g_flt, g_cu = values
                    if (feedback_time - t_detection > gripper_timeout
                            or feedback_time - t_status > gripper_timeout):
                        grasp_failure_reason = "gripper feedback timed out"
                    elif (t_close_request is None
                            or t_detection <= t_close_request
                            or t_status <= t_close_request):
                        grasp_failure_reason = "gripper feedback predates the close request"
                    elif not detected or g_sta != 3 or g_obj != 2 or g_flt != 0:
                        grasp_failure_reason = (
                            f"grasp not confirmed: detected={detected}, "
                            f"gSTA={g_sta}, gOBJ={g_obj}, gFLT={g_flt}"
                        )

                if grasp_failure_reason is not None:
                    phase = "FAILED"
                    grasp_hold_pending = True
                    t_arrival = None
                    print(f"[GRIPPER] FAILED: {grasp_failure_reason}")

            # Release only after TCP arrival has persisted with a confirmed grasp.
            # Cube orientation is not part of the place-arrival condition.
            if phase == "TRANSFER":
                if np.linalg.norm(error_xyz) < position_tolerance and robot_state_recent:
                    if t_arrival is None:
                        t_arrival = t_actual
                        print("[ARRIVAL] Place position reached; starting 0.5 s timer")
                    elif t_actual - t_arrival >= place_arrival_dwell:
                        phase = "OPENING"
                        open_requested = True
                        t_arrival = None
                else:
                    if t_arrival is not None:
                        print("[ARRIVAL] Place timer cancelled")
                    t_arrival = None

            elif phase == "OPENING" and t_open_request is not None:
                # Reject faults immediately, but allow the fingers time to open.
                opening_fault = (
                    status is not None
                    and status[1] > t_open_request
                    and feedback_time - status[1] <= gripper_timeout
                    and (status[0][0] != 3 or status[0][2] != 0)
                )
                if opening_fault:
                    phase = "FAILED"
                    grasp_hold_pending = True
                    print(f"[GRIPPER] OPENING failed: gSTA={status[0][0]}, gFLT={status[0][2]}")
                elif (gripper_is_open and t_status > t_open_request
                        and t_distance > t_open_request):
                    completed_cycles += 1
                    # The virtual robot did not move the physical cube. Rearm from
                    # its last observed real position, not the virtual place target.
                    with camera_lock:
                        placed_cube_position = last_cube_observation[0].copy()
                    t_place_completed = feedback_time
                    t_cube_moved = None
                    phase = "WAIT_FOR_MOVE"
                    print(
                        f"[CYCLE] Place {completed_cycles} completed; waiting for cube-marker "
                        f"displacement >= {args.restart_distance:.3f} m for {args.restart_dwell:.2f} s"
                    )
                elif feedback_time - t_open_request >= opening_timeout:
                    phase = "FAILED"
                    grasp_hold_pending = True
                    print("[GRIPPER] OPENING failed: no fresh full-opening confirmation within 5 s")

            elif phase == "WAIT_FOR_MOVE":
                # Only post-place observations can trigger another pick. Use their
                # receive times so a single image cannot satisfy the restart dwell.
                if not (robot_state_recent and camera_data_recent and gripper_is_open):
                    t_cube_moved = None
                elif camera_pose is not None and camera_pose[2] > t_place_completed:
                    cube_displacement = np.linalg.norm(camera_pose[0] - placed_cube_position)
                    if cube_displacement >= args.restart_distance:
                        if t_cube_moved is None:
                            t_cube_moved = camera_pose[2]
                            print("[CYCLE] Cube displacement detected; checking new observations before restarting")
                        elif camera_pose[2] - t_cube_moved >= args.restart_dwell:
                            # Re-evaluate symmetry from the current gripper for each new pick.
                            pick_yaw_sign = select_yaw_sign(
                                camera_pose, args.link_length, data.site_xpos[top_spring_id],
                                data.site_xpos[left_grip_id], data.site_xpos[right_grip_id],
                            )
                            print(f"[TARGET] Pick yaw branch: {0 if pick_yaw_sign == 1 else 180} deg; fixed for this approach")
                            new_p_top, new_p_left, new_p_right, new_rotation = build_positions(
                                camera_pose, args.link_length, yaw_sign=pick_yaw_sign
                            )
                            update_positions(new_p_top, new_p_left, new_p_right, new_rotation,
                                             target_top_spring, target_left, target_right)
                            t_arrival = None
                            t_close_request = None
                            t_open_request = None
                            t_cube_moved = None
                            t_place_completed = None
                            placed_cube_position = None
                            camera_hold_active = False
                            failure_hold_active = False
                            grasp_hold_pending = False
                            phase = "APPROACH"
                            print(f"[CYCLE] APPROACH started for cycle {completed_cycles + 1}")
                    else:
                        t_cube_moved = None

            # Capture the hold targets once, using recent robot telemetry only.
            # Plant data is local, so the failed-cycle hold can be captured now.
            if grasp_hold_pending and robot_state_recent:
                hold_top = data.site_xpos[top_spring_id].copy()
                hold_left = data.site_xpos[left_grip_id].copy()
                hold_right = data.site_xpos[right_grip_id].copy()
                hold_rotation = orientation_from_points(hold_top, hold_left, hold_right)
                update_positions(hold_top, hold_left, hold_right, hold_rotation,
                                 target_top_spring, target_left, target_right)
                grasp_hold_pending = False
                failure_hold_active = True
                print("[VMC] Holding the measured end-effector pose after cycle failure")

            # --- Compute VMC with data, then integrate only the dynamic data_sim ---
            data.qfrc_applied[:] = 0.0
            mechanism.apply(model, data)
            mechanism_2.apply(model, data)
            vmc_torques = np.array([data.qfrc_applied[adr] for adr in sync.qvel_indices], dtype=float)
            if not np.all(np.isfinite(vmc_torques)):
                raise RuntimeError("Non-finite VMC torques; stopping simulation")
            required_torques = vmc_torques
            data_sim.qfrc_applied[:] = 0.0
            data_sim.qfrc_applied[dof_ids] = np.clip(
                required_torques + data_sim.qfrc_bias[dof_ids],
                torque_limits[:, 0], torque_limits[:, 1],
            )
            mujoco.mj_step(model, data_sim)
            if not np.all(np.isfinite(data_sim.qpos)) or not np.all(np.isfinite(data_sim.qvel)):
                raise RuntimeError("Non-finite dynamic state; stopping simulation")

            return close_requested, open_requested

        # --- One-shot finger requests remain in the paced outer loop ---
        t_start = time.perf_counter()
        next_tick = t_start
        next_viewer_sync = t_start
        context = nullcontext(None)
        if not args.headless:
            from mujoco import viewer
            context = viewer.launch_passive(model, data_sim)
            viewer_started = True
        with context as v:
            if v is not None:
                v.cam.lookat[:] = [0.4, 0.0, 0.5]
                v.cam.distance = 2.0
                v.cam.azimuth = 135.0
                v.cam.elevation = -20.0
            while (v is None or v.is_running()) and (args.duration <= 0 or time.perf_counter() - t_start < args.duration):
                close_request, open_request = step_iteration()
                if close_request:
                    # Replace the real Float64(0.001) command with a local ramp.
                    sim_gripper_start = float(data_sim.qpos[finger_qpos_id])
                    sim_gripper_target = sim_closed_angle
                    sim_gripper_request_time = time.perf_counter()
                    t_close_request = sim_gripper_request_time
                    print("[GRIPPER SIM] CLOSING: moving only the MuJoCo fingers")
                if open_request:
                    # Replace the real Float64(0.140) command with a local ramp.
                    sim_gripper_start = float(data_sim.qpos[finger_qpos_id])
                    sim_gripper_target = 0.0
                    sim_gripper_request_time = time.perf_counter()
                    t_open_request = sim_gripper_request_time
                    print("[GRIPPER SIM] OPENING: moving only the MuJoCo fingers")
                now = time.perf_counter()
                if v is not None and now >= next_viewer_sync:
                    with camera_lock:
                        cube_for_view = last_cube_observation
                        table_for_view = last_table_observation
                    with v.lock():
                        v.user_scn.ngeom = 0
                        if cube_for_view is not None:
                            draw_sphere(v.user_scn, cube_for_view[0], radius=0.012, rgba=[1.0, 0.55, 0.1, 0.7])
                        if table_for_view is not None:
                            draw_sphere(v.user_scn, table_for_view[0], radius=0.014, rgba=[0.7, 0.3, 1.0, 0.7])
                        desired_tcp = (target_left.state(model, data).position
                                       + target_right.state(model, data).position) / 2.0
                        draw_sphere(v.user_scn, desired_tcp, radius=args.target_radius, rgba=[1.0, 0.8, 0.1, 0.9])
                        for source_id, target, color in (
                            (top_spring_id, target_top_spring, [0.1, 0.9, 1.0, 0.9]),
                            (left_grip_id, target_left, [0.4, 1.0, 0.4, 0.9]),
                            (right_grip_id, target_right, [1.0, 0.4, 0.7, 0.9]),
                        ):
                            position = target.state(model, data).position
                            draw_sphere(v.user_scn, position, radius=args.target_radius, rgba=color)
                            draw_capsule_line(v.user_scn, data_sim.site_xpos[source_id], position,
                                              radius=0.003, rgba=color)
                    v.sync()
                    next_viewer_sync = now + 1.0 / 30.0
                next_tick += dt
                remaining = next_tick - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    next_tick = time.perf_counter()
    except KeyboardInterrupt:
        print("\n[SIM] Interrupted")
    finally:
        bridge.stop()
        if viewer_started:
            time.sleep(1.0)  # Also allow rendering cleanup after Ctrl+C.


if __name__ == "__main__":
    main()

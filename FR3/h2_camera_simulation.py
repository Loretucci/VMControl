"""Run the H2 workflow on PC2 using a real camera and a simulated FR3.

The structure and geometry helpers follow h2_vmc_ros.py: main() waits for the
first camera pose, creates two sets of three connections, and runs step_iteration().
Measured q,dq come from data_sim instead of PC1; torques are applied with
mj_step instead of being published. The gripper closes only inside MuJoCo.
There are no robot torque publishers or physical gripper command publishers.

The lab marker pose has +z pointing down toward the object. TCP +z follows
that direction, so the upper force target lies along marker -z.

During APPROACH, camera expiry captures the current three anchors as fixed
targets. Both mechanisms remain active to hold that pose compliantly until
a fresh camera pose arrives, matching h2_vmc_ros.py.
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
    StaticObject,
    TanhSpring,
    VirtualConnection,
    VirtualMechanism,
    resolve_menagerie_path,
)


def draw_sphere(scene, pos, radius=0.012, rgba=None):
    """Draw a viewer-only target without depending on VMC visual exports."""
    if scene.ngeom >= scene.maxgeom:
        return False
    color = [0.2, 0.6, 1.0, 0.8] if rgba is None else rgba
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

def build_positions(camera_pose, link_length, height_offset=0.015):
    """Build the three target points from one measured base-frame marker pose.

    camera_pose contains (position, quaternion, receive_time). The position is
    in metres in the FR3 base frame; the quaternion uses MuJoCo order wxyz.
    The returned points are also in base/world coordinates, so they can be
    compared directly with MuJoCo's site_xpos values.
    height_offset raises the TCP and all three anchors along base +z. The
    default is 1.5 cm for picking; H3 passes zero for its separate place offset.
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


def main():
    # ROS is needed only when running main. The geometry helpers remain usable
    # for local headless checks on a Mac without a ROS installation.
    from geometry_msgs.msg import PoseStamped
    from mujoco_vmc import RobotROSBridge

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, help="combined scene XML; otherwise use MUJOCO_MENAGERIE_PATH")
    parser.add_argument("--camera-topic", default="/vision/aruco_pose")
    parser.add_argument("--camera-timeout", type=float, default=0.8)
    parser.add_argument("--initial-keyframe", default="home")
    parser.add_argument("--stiffness", type=float, default=600.0)
    parser.add_argument("--stiffness2", type=float, default=2000.0,
                        help="second parallel spring stiffness for each endpoint (N/m)")
    parser.add_argument("--damping", type=float, default=30.0)
    parser.add_argument("--max-force", type=float, default=10.0)
    parser.add_argument("--max-force2", type=float, default=20.0,
                        help="second parallel spring force limit per axis (N)")
    parser.add_argument("--link-length", type=float, default=0.8)
    parser.add_argument("--timestep", type=float, default=0.001)
    parser.add_argument("--close-time", type=float, default=1.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="wall-clock seconds after initialization; <=0 runs until stopped")
    args = parser.parse_args()
    values = (args.stiffness, args.stiffness2, args.damping, args.max_force,
              args.max_force2, args.link_length, args.timestep, args.close_time,
              args.camera_timeout, args.duration)
    if not np.all(np.isfinite(values)):
        parser.error("numeric arguments must be finite")
    if args.stiffness < 0 or args.stiffness2 < 0 or args.damping < 0:
        parser.error("stiffness, stiffness2 and damping must be nonnegative")
    if min(args.max_force, args.max_force2, args.link_length, args.timestep, args.close_time, args.camera_timeout) <= 0:
        parser.error("max-force, max-force2, link-length, timestep, close-time and camera-timeout must be positive")

    phase = "APPROACH"
    position_tolerance = 0.01
    connection_tolerance = 0.01

    # --- Load the robot and create the two states ---
    scene_xml = (args.scene.expanduser().resolve() if args.scene is not None else
                 resolve_menagerie_path() / "fr3_robotiq_2f_140" / "scene_fr3_robotiq.xml")
    if not scene_xml.is_file():
        raise FileNotFoundError(f"Scene not found: {scene_xml}; pass --scene with its actual path")
    spec = mujoco.MjSpec.from_file(str(scene_xml))
    build_ee_link_model(spec, args.link_length)
    model = spec.compile()
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION
    model.opt.timestep = args.timestep

    # data keeps its H2 meaning: the virtual model used to calculate VMC.
    # data_sim is the additional dynamic state that replaces the physical FR3.
    data = mujoco.MjData(model)
    data_sim = mujoco.MjData(model)
    sync = RobotStateSynchronizer(model)
    if sync.num_joints != 7:
        raise ValueError("Expected exactly seven FR3 arm joints")
    qpos_ids = np.asarray(sync.qpos_indices, dtype=int)
    dof_ids = np.asarray(sync.qvel_indices, dtype=int)
    torque_limits = model.jnt_actfrcrange[sync.joint_ids].copy()
    if np.any(torque_limits[:, 0] >= torque_limits[:, 1]):
        raise ValueError("The scene must define valid actuatorfrcrange limits for each FR3 joint")

    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.initial_keyframe)
    if key_id < 0:
        raise ValueError(f"Unknown initial keyframe: {args.initial_keyframe}")
    mujoco.mj_resetDataKeyframe(model, data_sim, key_id)
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data_sim)
    sync.set_state(data, data_sim.qpos[qpos_ids], data_sim.qvel[dof_ids], forward=True)
    finger_qpos_id = model.jnt_qposadr[model.joint("finger_joint").id]
    gripper_eq_id = model.equality("gripper_position").id

    # --- ROS callback and subscriber: same pattern as real H2 ---
    # The bridge runs the ROS executor only. Without sub_topic/pub_*_topic,
    # it neither subscribes to PC1 states nor creates command publishers.
    bridge = RobotROSBridge(synchronizer=sync, node_name="h2_camera_simulation")
    bridge.start()
    camera_lock = Lock()
    latest_camera_pose = None

    def camera_callback(msg):
        nonlocal latest_camera_pose
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

    try:
        if bridge.node is None:
            raise RuntimeError("The ROS bridge did not start")
        camera_sub = bridge.node.create_subscription(PoseStamped, args.camera_topic, camera_callback, 1)
        print(f"[SIM] Scene: {scene_xml}")
        print(f"[ROS] Camera input: {args.camera_topic}; required frame_id=base")
        print("[SIM] Arm torques and finger closure stay inside MuJoCo; no hardware commands are published")

        # --- Wait for the first target, then build the mechanism outside the loop ---
        # As in h2_vmc_ros.py, there are no placeholder target coordinates.
        # The viewer opens only after this first valid camera observation.
        first_camera_pose = None
        print("[WAIT] Waiting for the first valid base-frame camera pose")
        while first_camera_pose is None:
            with camera_lock:
                first_camera_pose = latest_camera_pose
            if first_camera_pose is None:
                time.sleep(0.05)
        print(f"[CAMERA] First valid target: {np.round(first_camera_pose[0], 4)} m")

        target_top_position, target_left_position, target_right_position, R_base_aruco = build_positions(
            first_camera_pose, args.link_length,
        )
        target_top_spring, target_left, target_right = build_ref_objects(
            target_top_position, target_left_position, target_right_position, R_base_aruco,
        )
        tcp_id = model.site("tcp").id
        top_spring_id = model.site("ee_top_spring").id
        left_grip_id = model.site("gripper_left_finger").id
        right_grip_id = model.site("gripper_right_finger").id
        r_top_site = MuJoCoSite(top_spring_id, "top_site")
        r_left_site = MuJoCoSite(left_grip_id, "left_finger")
        r_right_site = MuJoCoSite(right_grip_id, "right_finger")

        # The first three connections contain a spring and a damper.
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
        # A second spring acts in parallel between the same endpoints. Omitting
        # the damper means no second damping contribution; 0 is not a component.
        conn_att_2 = VirtualConnection(
            source=r_top_site,
            target=target_top_spring,
            components=[
                TanhSpring(args.stiffness2, max_force=args.max_force2),
            ],
            name="link_att_spring_2",
        )
        conn_left_2 = VirtualConnection(
            source=r_left_site,
            target=target_left,
            components=[
                TanhSpring(args.stiffness2, max_force=args.max_force2),
            ],
            name="link_left_spring_2",
        )
        conn_right_2 = VirtualConnection(
            source=r_right_site,
            target=target_right,
            components=[
                TanhSpring(args.stiffness2, max_force=args.max_force2),
            ],
            name="link_right_spring_2",
        )
        mechanism = VirtualMechanism([conn_att, conn_left, conn_right])
        mechanism_2 = VirtualMechanism([conn_att_2, conn_left_2, conn_right_2])

        mujoco.mj_forward(model, data)
        last_telemetry_log = time.time()
        t_arrival = None
        last_camera_time = first_camera_pose[2]
        camera_timeout = args.camera_timeout
        camera_hold_active = False
        closing_timeout = 5.0
        t_close_request = None
        dt = model.opt.timestep

        def step_iteration():
            nonlocal latest_camera_pose, last_camera_time, last_telemetry_log
            nonlocal t_arrival, phase, camera_hold_active

            # H2 gets q,dq from bridge.apply_incoming_state(). Here the same
            # synchronizer copies q,dq from the locally simulated FR3 instead.
            mujoco.mj_forward(model, data_sim)
            sync.set_state(data, data_sim.qpos[qpos_ids], data_sim.qvel[dof_ids], forward=True)
            with camera_lock:
                camera_pose = latest_camera_pose
                latest_camera_pose = None
            if camera_pose is not None:
                last_camera_time = camera_pose[2]

            t_actual = time.perf_counter()
            # The plant state is copied every iteration, so it has no ROS
            # reception timeout. This version does not test PC1 connection loss.
            robot_state_recent = True
            camera_data_recent = t_actual - last_camera_time <= camera_timeout

            # Match the real controller: capture the current anchors only once
            # per camera loss. Copies keep these references fixed while the
            # dynamic robot continues moving and both mechanisms remain active.
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
            elif camera_pose is not None:
                # New observations resume tracking. Later phases retain the
                # existing camera-update behavior, just like real H2.
                new_p_top, new_p_left, new_p_right, new_rotation = build_positions(camera_pose, args.link_length)
                update_positions(new_p_top, new_p_left, new_p_right, new_rotation,
                                 target_top_spring, target_left, target_right)
                if camera_hold_active:
                    camera_hold_active = False
                    print("[CAMERA] Fresh pose received; tracking the ArUco again")

            # --- Same target/error calculations as H2 ---
            target_att_pos = target_top_spring.state(model, data).position
            target_left_pos = target_left.state(model, data).position
            target_right_pos = target_right.state(model, data).position
            desired_tip_pos = (target_left_pos + target_right_pos) / 2.0
            error_xyz = desired_tip_pos - data.site_xpos[tcp_id]
            connection_errors = [
                np.linalg.norm(target_att_pos - data.site_xpos[top_spring_id]),
                np.linalg.norm(target_left_pos - data.site_xpos[left_grip_id]),
                np.linalg.norm(target_right_pos - data.site_xpos[right_grip_id]),
            ]
            target_rotation = orientation_from_points(target_att_pos, target_left_pos, target_right_pos)
            measured_rotation = orientation_from_points(
                data.site_xpos[top_spring_id], data.site_xpos[left_grip_id], data.site_xpos[right_grip_id],
            )

            within_tolerance = (
                np.linalg.norm(error_xyz) < position_tolerance
                and max(connection_errors) < connection_tolerance
            )
            close_requested = False

            # --- Same APPROACH -> CLOSING decision and 0.5 s dwell as H2 ---
            # HOLD errors can be small, but an expired camera still blocks closing.
            if phase == "APPROACH":
                if within_tolerance and robot_state_recent and camera_data_recent:
                    if t_arrival is None:
                        t_arrival = t_actual
                        print("[ARRIVAL] Conditions satisfied; starting 0.5 s timer")
                    elif t_actual - t_arrival >= 0.5:
                        phase = "CLOSING"
                        close_requested = True
                        t_arrival = None
                else:
                    if t_arrival is not None:
                        print(f"[ARRIVAL] Timer cancelled: within_tolerance={within_tolerance}, camera_recent={camera_data_recent}")
                    t_arrival = None

            # H2 checks real gripper feedback here. The local visual substitute
            # ramps the existing gripper_position equality to a closing angle.
            # CLOSED means only that this angle was reached, not a grasped object.
            if phase == "CLOSING" and t_close_request is not None:
                elapsed = t_actual - t_close_request
                fraction = min(elapsed / args.close_time, 1.0)
                model.eq_data[gripper_eq_id, 0] = 0.7 * fraction
                if fraction >= 1.0 and abs(data_sim.qpos[finger_qpos_id] - 0.7) < 0.02:
                    phase = "CLOSED"
                    print("[GRIPPER] CLOSED: simulated fingers closed; no object-detection feedback")
                elif elapsed >= closing_timeout:
                    phase = "FAILED"
                    print("[GRIPPER] FAILED: simulated fingers did not reach the closing angle")

            # Apply both parallel mechanisms. Each apply adds its joint forces
            # to qfrc_applied, so clear it once before the two evaluations.
            data.qfrc_applied[:] = 0.0
            mechanism.apply(model, data)
            mechanism_2.apply(model, data)
            vmc_torques = np.array([data.qfrc_applied[adr] for adr in sync.qvel_indices], dtype=float)
            if not np.all(np.isfinite(vmc_torques)):
                raise RuntimeError("Non-finite VMC torques; stopping simulation")
            required_torques = vmc_torques

            # H2 calls bridge.publish(data, torques=required_torques) here.
            # Instead, compensate the local plant bias, clip its joint torques,
            # and integrate only data_sim. Nothing is published to the hardware.
            data_sim.qfrc_applied[:] = 0.0
            data_sim.qfrc_applied[dof_ids] = np.clip(
                required_torques + data_sim.qfrc_bias[dof_ids],
                torque_limits[:, 0], torque_limits[:, 1],
            )
            mujoco.mj_step(model, data_sim)

            if time.time() - last_telemetry_log > 1.0:
                orientation_error_xyz_deg = -np.degrees(
                    Rotation.from_matrix(target_rotation.T @ measured_rotation).as_rotvec()
                )
                dwell = 0.0 if t_arrival is None else t_actual - t_arrival
                print(
                    f"[SIM] phase={phase}; sim_time={data_sim.time:.2f}s; "
                    f"target_mode={'HOLD' if camera_hold_active else 'ARUCO'}; "
                    f"TCP error={np.round(error_xyz, 4)} m; "
                    f"connections={np.round(connection_errors, 4)} m; "
                    f"orientation error={np.round(orientation_error_xyz_deg, 2)} deg; "
                    f"camera_age={t_actual - last_camera_time:.2f}s; arrival={dwell:.2f}s; "
                    f"max_abs_tau={np.max(np.abs(vmc_torques)):.2f} Nm"
                )
                last_telemetry_log = time.time()
            return close_requested

        # --- Same paced outer loop and one-shot close request pattern as H2 ---
        # Only the action taken for a close request is local to MuJoCo.
        t_start = time.perf_counter()
        next_tick = t_start
        next_viewer_sync = t_start
        context = nullcontext(None)
        if not args.headless:
            from mujoco import viewer
            context = viewer.launch_passive(model, data_sim)
        with context as v:
            if v is not None:
                v.cam.lookat[:] = [0.4, 0.0, 0.5]
                v.cam.distance = 2.0
                v.cam.azimuth = 135.0
                v.cam.elevation = -20.0
            while (v is None or v.is_running()) and (args.duration <= 0 or time.perf_counter() - t_start < args.duration):
                close_request = step_iteration()
                if close_request:
                    # Real H2 publishes Float64(0.001) to /hand/finger_distance.
                    # This timestamp instead starts the local finger-angle ramp.
                    t_close_request = time.perf_counter()
                    print("[GRIPPER] CLOSING: closing only the MuJoCo fingers")
                now = time.perf_counter()
                if v is not None and now >= next_viewer_sync:
                    with v.lock():
                        v.user_scn.ngeom = 0
                        target_tcp = (target_left.state(model, data).position + target_right.state(model, data).position) / 2.0
                        draw_sphere(v.user_scn, target_tcp, radius=0.018, rgba=[1.0, 0.8, 0.1, 0.9])
                        for source_id, target, color in (
                            (top_spring_id, target_top_spring, [0.1, 0.9, 1.0, 0.8]),
                            (left_grip_id, target_left, [0.4, 1.0, 0.4, 0.8]),
                            (right_grip_id, target_right, [1.0, 0.4, 0.7, 0.8]),
                        ):
                            target_pos = target.state(model, data).position
                            draw_sphere(v.user_scn, target_pos, radius=0.012, rgba=color)
                            draw_capsule_line(v.user_scn, data_sim.site_xpos[source_id], target_pos,
                                              radius=0.003, rgba=color)
                    v.sync()
                    next_viewer_sync = now + 1.0 / 30.0
                next_tick += dt
                remaining = next_tick - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    next_tick = time.perf_counter()
        if not args.headless:
            time.sleep(1.0)  # Allow the passive rendering thread to finish.
    except KeyboardInterrupt:
        print("\n[SIM] Interrupted")
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()

"""Run the H2 three-connection controller against a simulated FR3.

The dynamic MuJoCo state stands in for the physical robot. A second MjData
receives its measured arm joints and evaluates the same virtual mechanism used
by h2_vmc_ros.py. A draggable mocap marker stands in for the camera target.
As in vmc_simulations.py, ROS optionally publishes simulated joint states and
computed VMC torques; it does not supply the controller's input state. The
gripper is simulated locally and has no physical hardware connection.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import mujoco
from mujoco import viewer
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
    draw_capsule_line,
    draw_sphere,
)


FINGER_OFFSET = 0.30  # Same virtual lever arm as h2_vmc_ros.py, in metres.
ARRIVAL_TIME = 0.5
POSITION_TOLERANCE = 0.01
CONNECTION_TOLERANCE = 0.01
GRIPPER_CLOSED_ANGLE = 0.7


def build_ee_link_model(spec: mujoco.MjSpec, link_length: float) -> None:
    """Add H2's three force-application sites to robotiq_base_link."""
    ee_spec = spec.body("robotiq_base_link")
    tcp_pos = np.asarray(spec.site("tcp").pos, dtype=float)
    ee_spec.add_site(
        name="ee_top_spring",
        pos=[tcp_pos[0], tcp_pos[1], tcp_pos[2] - link_length],
        size=[0.012, 0.0, 0.0],
        rgba=[1.0, 0.0, 0.0, 0.8],
        group=0,
    )
    ee_spec.add_site(
        name="gripper_left_finger",
        pos=[tcp_pos[0], tcp_pos[1] - FINGER_OFFSET, tcp_pos[2]],
        size=[0.012, 0.0, 0.0],
        rgba=[1.0, 0.0, 0.0, 0.8],
        group=0,
    )
    ee_spec.add_site(
        name="gripper_right_finger",
        pos=[tcp_pos[0], tcp_pos[1] + FINGER_OFFSET, tcp_pos[2]],
        size=[0.012, 0.0, 0.0],
        rgba=[1.0, 0.0, 0.0, 0.8],
        group=0,
    )


def add_camera_marker(spec: mujoco.MjSpec) -> None:
    """Add a movable visual marker without changing robot dynamics."""
    marker = spec.worldbody.add_body(name="sim_camera_marker", mocap=True)
    marker.add_geom(
        name="sim_marker_center",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.018, 0.0, 0.0],
        rgba=[1.0, 0.8, 0.1, 0.9],
        contype=0,
        conaffinity=0,
    )
    for axis, color in (
        ([0.07, 0.0, 0.0], [1.0, 0.2, 0.2, 1.0]),
        ([0.0, 0.07, 0.0], [0.2, 1.0, 0.2, 1.0]),
        ([0.0, 0.0, 0.07], [0.2, 0.4, 1.0, 1.0]),
    ):
        marker.add_geom(
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[0.0, 0.0, 0.0, *axis],
            size=[0.003, 0.0, 0.0],
            rgba=color,
            contype=0,
            conaffinity=0,
        )


def build_positions(camera_pose, link_length: float, height_offset: float = 0.03, yaw_sign: int = 1):
    """Build H2 targets; yaw_sign=-1 selects the equivalent 180-degree grasp."""
    p_marker, q_marker = camera_pose  # Quaternion order: w, x, y, z.
    rotation_flat = np.empty(9)
    mujoco.mju_quat2Mat(rotation_flat, q_marker)
    R_base_marker = rotation_flat.reshape(3, 3)
    # Flip both in-plane axes together to preserve a right-handed frame.
    R_base_marker[:, :2] *= yaw_sign
    x_marker = R_base_marker[:, 0]
    z_marker = R_base_marker[:, 2]

    # Raise the pick TCP 3 cm along world/base +z, not the marker's z axis.
    p_target = p_marker + np.array([0.0, 0.0, float(height_offset)])
    p_left = p_target + FINGER_OFFSET * x_marker
    p_right = p_target - FINGER_OFFSET * x_marker
    p_top = p_target + link_length * z_marker
    return p_top, p_left, p_right, R_base_marker


def new_target(name: str, position: np.ndarray, rotation: np.ndarray) -> StaticObject:
    """Create the same stationary target state that H2 uses for camera data."""
    return StaticObject(
        ObjectState(
            position=position,
            rotation=rotation,
            linear_velocity=np.zeros(3),
        ),
        name=name,
    )


def marker_quaternion_from_sites(
    data: mujoco.MjData,
    tcp_id: int,
    top_id: int,
    left_id: int,
    right_id: int,
) -> np.ndarray:
    """Orient the initial marker so all three targets match the gripper sites.

    H2's marker +x points from the right virtual site toward the left one;
    marker +z points from TCP toward the upper spring site. The +y axis
    completes a right-handed frame. This makes the initial orientation error
    zero even if the home gripper is rotated in world coordinates.
    """
    x_axis = data.site_xpos[left_id] - data.site_xpos[right_id]
    x_axis /= np.linalg.norm(x_axis)
    z_axis = data.site_xpos[top_id] - data.site_xpos[tcp_id]
    z_axis -= np.dot(z_axis, x_axis) * x_axis
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rotation.ravel())
    return quat


def orientation_from_points(top, left, right) -> np.ndarray:
    """Reconstruct H2's target-frame axes for the orientation diagnostic."""
    x_axis = np.asarray(left) - right
    x_axis /= np.linalg.norm(x_axis)
    z_axis = (np.asarray(left) + right) / 2.0 - top
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


def main() -> None:
    parser = argparse.ArgumentParser(description="H2 controller with a simulated FR3 and camera marker")
    parser.add_argument("--state-pub-topic", type=str, default="/mujoco/h2/joint_states",
                        help="ROS topic for simulated arm JointState")
    parser.add_argument("--torque-pub-topic", type=str, default="/mujoco/h2/torques",
                        help="ROS topic for the seven VMC torques, without simulated gravity compensation")
    parser.add_argument("--pub-rate", type=float, default=1000.0,
                        help="requested ROS publish rate in Hz")
    parser.add_argument("--no-ros", action="store_true",
                        help="run locally without ROS, as when using the Mac viewer")
    parser.add_argument("--initial-keyframe", type=str, default="home",
                        help="initial robot pose from the scene XML")
    parser.add_argument("--stiffness", type=float, default=600.0, help="spring stiffness per connection (N/m)")
    parser.add_argument("--damping", type=float, default=30.0, help="damper coefficient per connection (Ns/m)")
    parser.add_argument("--max-force", type=float, default=10.0, help="spring force limit per axis (N)")
    parser.add_argument("--link-length", type=float, default=0.4, help="TCP to upper spring site distance (m)")
    parser.add_argument("--timestep", type=float, default=0.001, help="MuJoCo physics step (s)")
    parser.add_argument("--camera-rate", type=float, default=4.0, help="simulated camera updates per second")
    parser.add_argument("--target-offset", nargs=3, type=float, default=(0.05, 0.0, 0.02),
                        metavar=("DX", "DY", "DZ"), help="initial marker displacement from home TCP (m)")
    parser.add_argument("--close-time", type=float, default=1.0, help="time for the simulated gripper to close (s)")
    parser.add_argument("--headless", action="store_true", help="run without the MuJoCo viewer")
    parser.add_argument("--duration", type=float, default=0.0, help="simulation seconds; 0 runs until interrupted")
    args = parser.parse_args()
    if args.stiffness < 0 or args.damping < 0 or args.max_force <= 0:
        parser.error("stiffness and damping must be nonnegative; max-force must be positive")
    if (args.link_length <= 0 or args.timestep <= 0 or args.camera_rate <= 0
            or args.close_time <= 0 or args.pub_rate <= 0):
        parser.error("link-length, timestep, camera-rate, close-time and pub-rate must be positive")

    # The combined FR3/Robotiq scene is alongside MuJoCo Projects on the Mac,
    # and both directories are visible under /workspace in the ROS container.
    # The model is extended only in memory; its XML/meshes are not modified.
    scene_xml = Path(__file__).resolve().parents[2] / "fr3_robotiq_2f_140" / "scene_fr3_robotiq.xml"
    if not scene_xml.is_file():
        parser.error(f"scene not found: {scene_xml}")
    spec = mujoco.MjSpec.from_file(str(scene_xml))
    build_ee_link_model(spec, args.link_length)
    add_camera_marker(spec)
    model = spec.compile()
    model.opt.timestep = args.timestep
    # The XML's seven position servos are disabled. qfrc_applied below carries
    # VMC torque plus simulated gravity/bias compensation instead.
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION

    real = mujoco.MjData(model)
    virtual = mujoco.MjData(model)
    keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.initial_keyframe)
    if keyframe_id < 0:
        parser.error(f"unknown keyframe: {args.initial_keyframe}")
    mujoco.mj_resetDataKeyframe(model, real, keyframe_id)
    mujoco.mj_resetDataKeyframe(model, virtual, keyframe_id)
    mujoco.mj_forward(model, real)

    sync = RobotStateSynchronizer(model)
    if sync.num_joints != 7:
        raise RuntimeError(f"Expected seven FR3 joints; found {sync.num_joints}")

    # Match vmc_simulations.py: this bridge only publishes the simulated
    # robot's measured arm state and the virtual mechanism's seven torques.
    # It does not subscribe to an external robot or feed its own ROS topics
    # back into the simulation. Using separate H2 topic names avoids the real
    # controller's external_torques topic.
    bridge = None
    if not args.no_ros:
        from mujoco_vmc.utils.ros import RobotROSBridge

        bridge = RobotROSBridge(
            synchronizer=sync,
            pub_state_topic=args.state_pub_topic,
            pub_torque_topic=args.torque_pub_topic,
            pub_rate_hz=args.pub_rate,
            node_name="fr3_h2_simulation",
        )
        bridge.start()
    qpos_ids = np.asarray(sync.qpos_indices, dtype=int)
    dof_ids = np.asarray(sync.qvel_indices, dtype=int)
    torque_limits = model.jnt_actfrcrange[np.asarray(sync.joint_ids, dtype=int)]

    tcp_id = model.site("tcp").id
    top_id = model.site("ee_top_spring").id
    left_id = model.site("gripper_left_finger").id
    right_id = model.site("gripper_right_finger").id
    marker_id = model.body_mocapid[model.body("sim_camera_marker").id]
    finger_qpos_id = model.jnt_qposadr[model.joint("finger_joint").id]
    gripper_eq_id = model.equality("gripper_position").id

    # A pose derived from the home gripper makes the three target directions
    # consistent with H2. The small translation creates a reachable motion.
    real.mocap_pos[marker_id] = real.site_xpos[tcp_id] + np.asarray(args.target_offset)
    real.mocap_quat[marker_id] = marker_quaternion_from_sites(real, tcp_id, top_id, left_id, right_id)
    mujoco.mj_forward(model, real)
    initial_camera_pose = (real.mocap_pos[marker_id].copy(), real.mocap_quat[marker_id].copy())
    pick_yaw_sign = select_yaw_sign(
        initial_camera_pose, args.link_length, real.site_xpos[top_id],
        real.site_xpos[left_id], real.site_xpos[right_id],
    )
    print(f"[TARGET] Pick yaw branch: {0 if pick_yaw_sign == 1 else 180} deg; fixed for this approach")
    p_top, p_left, p_right, rotation = build_positions(
        initial_camera_pose, args.link_length, yaw_sign=pick_yaw_sign
    )
    target_top = new_target("target_attachment_site", p_top, rotation)
    target_left = new_target("target_left_finger", p_left, rotation)
    target_right = new_target("target_right_finger", p_right, rotation)

    # These are the same three TanhSpring + LinearDamper connections as H2.
    mechanism = VirtualMechanism([
        VirtualConnection(
            source=MuJoCoSite(top_id, "top_site"), target=target_top,
            components=[TanhSpring(args.stiffness, max_force=args.max_force), LinearDamper(args.damping)],
            name="link_att_spring",
        ),
        VirtualConnection(
            source=MuJoCoSite(left_id, "left_finger"), target=target_left,
            components=[TanhSpring(args.stiffness, max_force=args.max_force), LinearDamper(args.damping)],
            name="link_left_spring",
        ),
        VirtualConnection(
            source=MuJoCoSite(right_id, "right_finger"), target=target_right,
            components=[TanhSpring(args.stiffness, max_force=args.max_force), LinearDamper(args.damping)],
            name="link_right_spring",
        ),
    ])

    phase = "APPROACH"
    arrival_since = None
    close_started = None
    camera_period = 1.0 / args.camera_rate
    next_camera_sample = camera_period
    last_camera_sample = 0.0
    next_log = 0.0
    last_error = np.zeros(3)
    last_connection_error = 0.0

    print(f"Scene: {scene_xml}")
    print(f"Home TCP: {np.round(real.site_xpos[tcp_id], 4)} m")
    print(f"Initial synthetic camera target: {np.round(real.mocap_pos[marker_id], 4)} m")
    print("The yellow marker can be moved in the viewer; its pose is sampled at camera-rate.")
    print("CLOSED means the simulated fingers closed; there is no object-detection feedback.")

    def step_iteration() -> None:
        nonlocal phase, arrival_since, close_started, next_camera_sample
        nonlocal last_camera_sample, next_log, last_error, last_connection_error

        t = float(real.time)
        # mj_step integrates the state; mj_forward refreshes poses, Jacobians
        # and qfrc_bias at the newly integrated q before this control cycle.
        mujoco.mj_forward(model, real)

        # The mocap body substitutes for /vision/aruco_pose. Sampling at 4 Hz
        # by default makes marker movement visible to H2 at a camera-like rate.
        if t + 1e-10 >= next_camera_sample:
            next_camera_sample += camera_period
            p_marker = real.mocap_pos[marker_id].copy()
            q_marker = real.mocap_quat[marker_id].copy()
            q_norm = np.linalg.norm(q_marker)
            if np.all(np.isfinite(p_marker)) and np.all(np.isfinite(q_marker)) and q_norm > 1e-6:
                p_top, p_left, p_right, rotation = build_positions(
                    (p_marker, q_marker / q_norm), args.link_length, yaw_sign=pick_yaw_sign
                )
                for target, position in ((target_top, p_top), (target_left, p_left), (target_right, p_right)):
                    target.set_state(ObjectState(position=position, rotation=rotation, linear_velocity=np.zeros(3)))
                last_camera_sample = t

        # Copy the simulated real robot's measured arm q and dq into the
        # virtual robot. Its gripper DOFs remain at home, matching H2's ROS
        # JointState input, which contains only the seven arm joints.
        sync.set_state(virtual, real.qpos[qpos_ids], real.qvel[dof_ids], forward=True)

        top_position = target_top.state(model, virtual).position
        left_position = target_left.state(model, virtual).position
        right_position = target_right.state(model, virtual).position
        target_tcp = (left_position + right_position) / 2.0
        error_xyz = target_tcp - virtual.site_xpos[tcp_id]
        connection_errors = [
            np.linalg.norm(top_position - virtual.site_xpos[top_id]),
            np.linalg.norm(left_position - virtual.site_xpos[left_id]),
            np.linalg.norm(right_position - virtual.site_xpos[right_id]),
        ]
        max_connection_error = max(connection_errors)
        last_error = error_xyz.copy()
        last_connection_error = max_connection_error

        # Reproduce H2's 1 cm / 0.5 s arrival gate. In this local simulation,
        # robot state arrives every physics step; camera freshness uses the
        # synthetic camera's most recent sample.
        camera_fresh = t - last_camera_sample <= 0.8
        within_tolerance = (
            np.linalg.norm(error_xyz) < POSITION_TOLERANCE
            and max_connection_error < CONNECTION_TOLERANCE
        )
        if phase == "APPROACH":
            if within_tolerance and camera_fresh:
                if arrival_since is None:
                    arrival_since = t
                elif t - arrival_since >= ARRIVAL_TIME:
                    phase = "CLOSING"
                    close_started = t
                    arrival_since = None
                    print(f"t={t:.2f}s: arrival confirmed; simulated gripper closing")
            else:
                arrival_since = None

        # The XML has no gripper actuator. Its existing equality constraint
        # pins finger_joint to a target angle; ramp it to visualize a closure.
        # This constraint can exert arbitrary force, so this is not a contact
        # or grasp-quality test of the real Robotiq gripper.
        if phase == "CLOSING":
            elapsed = t - close_started
            fraction = min(elapsed / args.close_time, 1.0)
            model.eq_data[gripper_eq_id, 0] = GRIPPER_CLOSED_ANGLE * fraction
            if fraction >= 1.0 and abs(real.qpos[finger_qpos_id] - GRIPPER_CLOSED_ANGLE) < 0.02:
                phase = "CLOSED"
                print(f"t={t:.2f}s: simulated fingers closed (no object detection)")
            elif elapsed >= args.close_time + 5.0:
                phase = "FAILED"
                print(f"t={t:.2f}s: simulated gripper did not reach its commanded angle")

        # The virtual model evaluates spring/damper forces and maps them to
        # generalized torque through the site Jacobians. Only its seven arm
        # DOFs are sent to the dynamic robot.
        virtual.qfrc_applied[:] = 0.0
        mechanism.apply(model, virtual)
        vmc_torques = virtual.qfrc_applied[dof_ids].copy()
        if not np.all(np.isfinite(vmc_torques)):
            raise RuntimeError("Non-finite VMC torque; stopping simulation")

        # MuJoCo needs gravity/bias compensation when its position servos are
        # disabled. The real FR3 controller has its own torque convention.
        # qfrc_applied bypasses actuatorfrcrange, so clip total applied torque
        # to the seven joint limits before advancing the simulated robot.
        applied_torques = np.clip(
            vmc_torques + real.qfrc_bias[dof_ids],
            torque_limits[:, 0], torque_limits[:, 1],
        )
        real.qfrc_applied[:] = 0.0
        real.qfrc_applied[dof_ids] = applied_torques
        if bridge is not None:
            # Publish the VMC component alone, as h2_vmc_ros.py does. The
            # gravity/bias term above is required only by MuJoCo dynamics.
            bridge.publish(real, torques=vmc_torques)

        if t + 1e-10 >= next_log:
            target_rotation = orientation_from_points(top_position, left_position, right_position)
            measured_rotation = orientation_from_points(
                virtual.site_xpos[top_id], virtual.site_xpos[left_id], virtual.site_xpos[right_id]
            )
            orientation_error = -np.degrees(
                Rotation.from_matrix(target_rotation.T @ measured_rotation).as_rotvec()
            )
            print(
                f"t={t:5.2f}s  phase={phase:8s}  "
                f"TCP error={np.round(error_xyz, 4)} m  "
                f"max link error={max_connection_error:.4f} m  "
                f"orientation error={np.round(orientation_error, 1)} deg  "
                f"|tau_vmc|max={np.max(np.abs(vmc_torques)):.2f} Nm"
            )
            next_log += 1.0

        mujoco.mj_step(model, real)

    try:
        if args.headless:
            # Pace headless mode to wall time so ROS observers can sample the
            # running simulation, just as in vmc_simulations.py.
            next_tick = time.perf_counter()
            while args.duration <= 0 or real.time < args.duration:
                step_iteration()
                next_tick += args.timestep
                remaining = next_tick - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    next_tick = time.perf_counter()
        else:
            next_tick = time.perf_counter()
            next_render = next_tick
            with viewer.launch_passive(model, real) as v:
                print("Viewer active: drag the yellow mocap marker to move the target.")
                while v.is_running() and (args.duration <= 0 or real.time < args.duration):
                    step_iteration()
                    now = time.perf_counter()
                    if now >= next_render:
                        with v.lock():
                            v.user_scn.ngeom = 0
                            for source_id, target, color in (
                                (top_id, target_top, [0.1, 0.9, 1.0, 0.8]),
                                (left_id, target_left, [0.4, 1.0, 0.4, 0.8]),
                                (right_id, target_right, [1.0, 0.4, 0.7, 0.8]),
                            ):
                                target_pos = target.state(model, virtual).position
                                draw_sphere(v.user_scn, target_pos, radius=0.012, rgba=color)
                                draw_capsule_line(
                                    v.user_scn, real.site_xpos[source_id], target_pos,
                                    radius=0.003, rgba=color,
                                )
                        v.sync()
                        next_render = now + 1.0 / 30.0
                    next_tick += args.timestep
                    remaining = next_tick - time.perf_counter()
                    if remaining > 0:
                        time.sleep(remaining)
                    else:
                        next_tick = time.perf_counter()
            # launch_passive renders on a separate Linux thread. Its close()
            # requests shutdown but does not join that thread; let it finish
            # before ROS and GLFW are torn down at process exit.
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nSimulation interrupted")
    finally:
        if bridge is not None:
            bridge.stop()

    print(
        f"Final: t={real.time:.2f}s, phase={phase}, "
        f"TCP error={np.round(last_error, 4)} m, "
        f"max link error={last_connection_error:.4f} m, "
        f"finger_joint={real.qpos[finger_qpos_id]:.3f} rad"
    )


if __name__ == "__main__":
    main()

"""Simulate the FR3 with the same three-point virtual mechanism as h1_vmc_ros.

MuJoCo supplies the simulated joint state and advances the robot with mj_step.
ROS can optionally publish this state and the VMC torques for inspection; no
external robot state or camera measurement is needed.
"""

import argparse
from pathlib import Path
import time

import mujoco
from mujoco import viewer
import numpy as np
import math
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
    draw_capsule_line,
    draw_sphere,
)

def build_ee_link_model(
    spec,
    link_length: float = 0.4,
):
    """Attach the custom tool's visualization and three control points to link 7.

    All positions below are in the local frame of ``fr3_link7``. Because the
    sites belong to that body, they follow its motion without adding joints.
    ``link_length`` is measured from the existing attachment site, whose local
    z coordinate is 0.107 m.
    """

    # MjSpec lets us extend the XML scene in memory before compiling MjModel.
    ee_spec = spec.body("robotiq_base_link")
    tcp_site = spec.site("tcp")
    tcp_pos = tcp_site.pos
    L = float(link_length)

    # The top spring site defines the point where the top spring is attached.
    ee_spec.add_site(name="ee_top_spring", pos=[tcp_pos[0], tcp_pos[1],tcp_pos[2]-L], size=[0.012, 0.0, 0.0], rgba=[1.0, 0.0, 0.0, 0.8], group=0)

    # Separate finger sites let three translational connections also resist
    # changes in tool orientation through their different lever arms.
    ee_spec.add_site(name="gripper_left_finger", pos=[tcp_pos[0], tcp_pos[1] - 0.3, tcp_pos[2]], size=[0.012, 0.0, 0.0], rgba=[1.0, 0.0, 0.0, 0.8], group=0) 
    ee_spec.add_site(name="gripper_right_finger", pos=[tcp_pos[0], tcp_pos[1] + 0.3, tcp_pos[2]], size=[0.012, 0.0, 0.0], rgba=[1.0, 0.0, 0.0, 0.8], group=0)


def build_ref_objects(pos_x, pos_y, pos_z,link_length):
    """Create three stationary world-frame targets for the virtual tool sites.

    ``(pos_x, pos_y, pos_z)`` is the desired tip position in the MuJoCo world.
    For this scene the world and FR3 base frames are aligned. The other two
    points encode the desired orientation: fingers lie along world x, while
    the attachment site lies ``L`` above the tip.
    """
    L = float(link_length)
    cur_ee_rot = np.eye(3)
    target_att_pos = np.array([pos_x, pos_y, pos_z+L])
    target_l_pos = np.array([pos_x + 0.3, pos_y, pos_z])
    target_r_pos = np.array([pos_x - 0.3, pos_y, pos_z])
    

    # At the desired tool orientation, the attachment is L above the tip and
    # the two finger targets lie at +x/-x in world coordinates.
    target_top_spring = StaticObject(ObjectState(position=target_att_pos, rotation=cur_ee_rot, linear_velocity=np.zeros(3)),name = "target_attachment_site")
    target_l = StaticObject(ObjectState(position=target_l_pos, rotation=cur_ee_rot,linear_velocity=np.zeros(3)),name ="target_left_finger")
    target_r = StaticObject(ObjectState(position=target_r_pos, rotation=cur_ee_rot,linear_velocity=np.zeros(3)),name = "target_right_finger")
    return target_top_spring,target_l, target_r


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
    """Build the virtual mechanism and drive a simulated FR3 with its torques."""
    parser = argparse.ArgumentParser(description="FR3 virtual-mechanism simulation")
    parser.add_argument("--state-pub-topic", type=str, default="/mujoco/robot/joint_states", help="Topic for simulated joint states")
    parser.add_argument("--torque-pub-topic", type=str, default="/mujoco/robot/torques", help="Topic to publish computed joint torques (default: /mujoco/robot/torques)")
    parser.add_argument("--pub-rate", type=float, default=1000.0, help="Publish rate in Hz for outgoing ROS topics (default: 1000.0)")
    parser.add_argument("--no-ros", action="store_true", help="Run the simulation without ROS publishers")
    parser.add_argument("--initial-keyframe", type=str, default="home", help="Initial robot pose from the scene (default: home)")
    parser.add_argument("--link-length", type=float, default=0.4, help="Length of extended rigid link in meters (default: 0.24)")
    parser.add_argument("--stiffness", type=float, default=2000.0, help="VMC spring stiffness for each endpoint (N/m)")
    parser.add_argument("--damping", type=float, default=50.0, help="VMC damper coefficient for each endpoint (Ns/m)")
    parser.add_argument("--max-force", type=float, default=20.0, help="Maximum saturating VMC force per spring (N)")
    parser.add_argument("--timestep", type=float, default=0.001, help="MuJoCo physics timestep in seconds (default: 0.001)")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode without passive viewer")
    parser.add_argument("--duration", type=float, default=60.0, help="Run duration in seconds if headless (<= 0 runs indefinitely)")
    args = parser.parse_args()
    if args.timestep <= 0 or args.pub_rate <= 0:
        parser.error("timestep and pub-rate must be positive")

    # Fixed desired location of the custom tool tip, in world coordinates.
    reference_position = (0.4, 0.0, 0.15)

    # The combined scene contains the robot, gripper, and world. Add the custom
    # sites before compiling the complete model.
    scene_xml = (
        Path(__file__).resolve().parents[2]
        / "fr3_robotiq_2f_140"
        / "scene_fr3_robotiq.xml"
    )    
    if not scene_xml.exists():
        raise FileNotFoundError(f"Scene XML not found at {scene_xml}.")
    

    spec = mujoco.MjSpec.from_file(str(scene_xml))
    build_ee_link_model(spec, args.link_length)
    target_top_spring, target_left, target_right = build_ref_objects(*reference_position, args.link_length)
    model = spec.compile()
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION

    # The model stores geometry/parameters; MjData stores its evolving state.
    model.opt.timestep = args.timestep
    data = mujoco.MjData(model)
    sync = RobotStateSynchronizer(model)
    # In simulation the state starts at a keyframe instead of arriving via ROS.
    keyframe_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.initial_keyframe)
    if keyframe_id < 0:
        parser.error(f"unknown keyframe: {args.initial_keyframe}")
    mujoco.mj_resetDataKeyframe(model, data, keyframe_id)
    mujoco.mj_forward(model, data)

    # ROS is only an output of this simulation. The control loop reads data
    # directly, as in Omar's standalone simulation examples.
    bridge = None
    if not args.no_ros:
        from mujoco_vmc.utils.ros import RobotROSBridge

        bridge = RobotROSBridge(
            synchronizer=sync,
            pub_state_topic=args.state_pub_topic,
            pub_torque_topic=args.torque_pub_topic,
            pub_rate_hz=args.pub_rate,
            node_name="fr3_vmc_simulation",
        )
        bridge.start()

    # The tool tip and three point pairs are used for control diagnostics.
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

    # The XML has position servos, disabled above for this simulation. Apply
    # generalized forces directly at the seven arm DOFs instead of data.ctrl.
    dof_ids = np.array(sync.qvel_indices)
    torque_limits = model.jnt_actfrcrange[sync.joint_ids]
    last_telemetry_log = time.perf_counter()
    dt = model.opt.timestep

    def step_iteration():
        """Read MuJoCo's state, compute VMC torques, then advance the robot."""
        nonlocal last_telemetry_log
        # mj_step integrates q at the end of each step. Refresh kinematics,
        # site velocities and qfrc_bias for that new q before evaluating VMC.
        mujoco.mj_forward(model, data)

        if time.perf_counter() - last_telemetry_log > 1.0:
            # The fixed StaticObjects are the source of truth for both errors.
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
            # The rotation vector gives the signed correction about the
            # fixed target frame's X, Y and Z axes.
            relative_rotation_target = target_rotation.T @ measured_rotation
            orientation_error_xyz_deg = -np.degrees(
                Rotation.from_matrix(relative_rotation_target).as_rotvec()
            )

            connection_errors = [
                np.linalg.norm(target_att_pos - data.site_xpos[top_spring_id]),
                np.linalg.norm(target_left_pos - data.site_xpos[left_grip_id]),
                np.linalg.norm(target_right_pos - data.site_xpos[right_grip_id]),
            ]
            print(
                f"Errore punta xyz: {np.round(error_xyz, 4)} m; "
                f"modulo: {np.linalg.norm(error_xyz):.4f} m; "
                f"errore orientazione target xyz: {np.round(orientation_error_xyz_deg, 2)}°; "
                f"massimo sui 3 collegamenti: {max(connection_errors):.4f} m"
            )
            last_telemetry_log = time.perf_counter()

        # MuJoCoSite maps each virtual world force to generalized joint forces
        # via its Jacobian. Clear the accumulator at every iteration.
        data.qfrc_applied[:] = 0.0
        mechanism.apply(model, data)
        vmc_torques = data.qfrc_applied[dof_ids].copy()

        # The optional ROS torque topic keeps the h1 convention: VMC torques
        # alone. MuJoCo also needs bias compensation to hold the simulated arm.
        # Applied generalized forces are not limited by actuatorfrcrange, so
        # clip the seven arm torques explicitly before the physics step.
        simulated_torques = np.clip(
            vmc_torques + data.qfrc_bias[dof_ids],
            torque_limits[:, 0],
            torque_limits[:, 1],
        )
        data.qfrc_applied[:] = 0.0
        data.qfrc_applied[dof_ids] = simulated_torques
        if bridge is not None:
            bridge.publish(data, torques=vmc_torques)

        # mj_step integrates the robot under the clipped force stored above.
        mujoco.mj_step(model, data)

    # The controller and simulation share one physics clock; wall time paces it.
    t_start = time.perf_counter()
    next_tick = t_start
    try:
        if args.headless:
            while args.duration <= 0 or time.perf_counter() - t_start < args.duration:
                step_iteration()
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
                    step_iteration()
                    now = time.perf_counter()
                    if now >= next_viewer_sync:
                        # StaticObject targets are not XML sites. Draw the
                        # three target points and their virtual connections.
                        with v.lock():
                            v.user_scn.ngeom = 0
                            for source_id, target in (
                                (top_spring_id, target_top_spring),
                                (left_grip_id, target_left),
                                (right_grip_id, target_right),
                            ):
                                target_pos = target.state(model, data).position
                                draw_sphere(v.user_scn, target_pos, radius=0.012)
                                draw_capsule_line(
                                    v.user_scn,
                                    data.site_xpos[source_id],
                                    target_pos,
                                    radius=0.003,
                                )
                        v.sync()
                        next_viewer_sync = now + 1.0 / 30.0
                    next_tick += dt
                    remaining = next_tick - time.perf_counter()
                    if remaining > 0:
                        time.sleep(remaining)
                    else:
                        next_tick = time.perf_counter()
    except KeyboardInterrupt:
        print("\n[INFO] Simulation interrupted by user.")
    finally:
        if bridge is not None:
            bridge.stop()

     
if __name__ == "__main__":
    main()

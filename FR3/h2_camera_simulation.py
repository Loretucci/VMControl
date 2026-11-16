"""Use the real ArUco camera topic to control a simulated FR3 on PC2.

The only application ROS endpoint is a PoseStamped subscriber. Arm torques
and finger closure are applied directly to MuJoCo; no hardware command is
published. The geometry helpers are imported from h2_simulations.py; importing
that module does not run its main function or start its ROS bridge.

As in the current real H2 controller, an old camera pose prevents a new close
request but remains the VMC target. Targets also keep updating during closing.
The simulated fingers closing is a visual test, not object-detection feedback.
"""

from __future__ import annotations

import argparse
from contextlib import nullcontext
from pathlib import Path
from threading import Lock, Thread
import time

import mujoco
import numpy as np

from mujoco_vmc import (
    LinearDamper,
    MuJoCoSite,
    ObjectState,
    RobotStateSynchronizer,
    TanhSpring,
    VirtualConnection,
    VirtualMechanism,
    draw_capsule_line,
    draw_sphere,
    resolve_menagerie_path,
)
from h2_simulations import (
    ARRIVAL_TIME,
    CONNECTION_TOLERANCE,
    GRIPPER_CLOSED_ANGLE,
    POSITION_TOLERANCE,
    build_ee_link_model,
    build_positions,
    new_target,
)


class CameraPickSimulation:
    """Keep the dynamic simulated plant separate from the virtual controller."""

    def __init__(self, scene: Path, args: argparse.Namespace):
        self.args = args
        spec = mujoco.MjSpec.from_file(str(scene))
        build_ee_link_model(spec, args.link_length)
        self.model = spec.compile()
        self.model.opt.timestep = args.timestep
        self.model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION

        # plant is the physical FR3 stand-in; virtual evaluates VMC at its q,dq.
        # Only plant is advanced with mj_step. Neither state comes from PC1.
        self.plant = mujoco.MjData(self.model)
        self.virtual = mujoco.MjData(self.model)
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, args.initial_keyframe)
        if key_id < 0:
            raise ValueError(f"Unknown initial keyframe: {args.initial_keyframe}")
        for data in (self.plant, self.virtual):
            mujoco.mj_resetDataKeyframe(self.model, data, key_id)
            mujoco.mj_forward(self.model, data)

        self.sync = RobotStateSynchronizer(self.model)
        if self.sync.num_joints != 7:
            raise ValueError("Expected exactly seven FR3 arm joints")
        self.qpos_ids = np.asarray(self.sync.qpos_indices, dtype=int)
        self.dof_ids = np.asarray(self.sync.qvel_indices, dtype=int)
        self.torque_limits = self.model.jnt_actfrcrange[self.sync.joint_ids].copy()
        if np.any(self.torque_limits[:, 0] >= self.torque_limits[:, 1]):
            raise ValueError("The scene must define valid actuatorfrcrange limits for each FR3 joint")

        self.tcp_id = self.model.site("tcp").id
        self.site_ids = [self.model.site(name).id for name in (
            "ee_top_spring", "gripper_left_finger", "gripper_right_finger",
        )]
        self.finger_qpos_id = self.model.jnt_qposadr[self.model.joint("finger_joint").id]
        self.gripper_eq_id = self.model.equality("gripper_position").id

        # No placeholder targets: hold the home state until a valid pose arrives.
        self.targets = None
        self.mechanism = None
        self.phase = "WAITING_TARGET"
        self.last_camera_time = None
        self.camera_recent = None
        self.arrival_since = None
        self.close_started = None
        self.error_xyz = np.zeros(3)
        self.connection_errors = np.zeros(3)
        self.vmc_torques = np.zeros(7)

    def update_target(self, camera_pose) -> None:
        """Use the same marker axes, target offsets and zero target velocity as H2."""
        position, quaternion, received_at = camera_pose
        p_top, p_left, p_right, rotation = build_positions(
            (position, quaternion), self.args.link_length,
        )
        positions = (p_top, p_left, p_right)
        self.last_camera_time = received_at
        if self.targets is None:
            self.targets = [new_target(name, p, rotation) for name, p in zip(
                ("target_attachment_site", "target_left_finger", "target_right_finger"),
                positions,
            )]
            self.mechanism = VirtualMechanism([
                VirtualConnection(
                    source=MuJoCoSite(site_id, f"robot_anchor_{index}"),
                    target=target,
                    components=[
                        TanhSpring(self.args.stiffness, max_force=self.args.max_force),
                        LinearDamper(self.args.damping),
                    ],
                    name=f"camera_connection_{index}",
                )
                for index, (site_id, target) in enumerate(zip(self.site_ids, self.targets))
            ])
            self.phase = "APPROACH"
            print(f"[CAMERA] First valid target: {np.round(position, 4)} m; phase=APPROACH")
        else:
            # All phases use new targets, including CLOSING/CLOSED, as in H2.
            for target, p in zip(self.targets, positions):
                target.set_state(ObjectState(
                    position=p, rotation=rotation, linear_velocity=np.zeros(3),
                ))

    def step(self, now: float, camera_pose=None) -> None:
        """Consume at most one pose and advance only the simulated robot."""
        if camera_pose is not None:
            self.update_target(camera_pose)
        if self.targets is None:
            return

        mujoco.mj_forward(self.model, self.plant)
        self.sync.set_state(
            self.virtual, self.plant.qpos[self.qpos_ids], self.plant.qvel[self.dof_ids],
            forward=True,
        )
        target_positions = [target.state(self.model, self.virtual).position for target in self.targets]
        target_tcp = (target_positions[1] + target_positions[2]) / 2.0
        self.error_xyz = target_tcp - self.virtual.site_xpos[self.tcp_id]
        self.connection_errors = np.array([
            np.linalg.norm(p - self.virtual.site_xpos[site_id])
            for p, site_id in zip(target_positions, self.site_ids)
        ])

        # Camera freshness uses local wall time, just like the real H2 script.
        # In particular, expiry does NOT remove the last target or zero VMC.
        camera_recent = now - self.last_camera_time <= self.args.camera_timeout
        if camera_recent != self.camera_recent:
            if not camera_recent:
                print("[CAMERA] Pose timed out: keeping the last VMC target; new closing is inhibited")
            elif self.camera_recent is False:
                print("[CAMERA] Fresh camera poses resumed")
            self.camera_recent = camera_recent

        within_tolerance = (
            np.linalg.norm(self.error_xyz) < POSITION_TOLERANCE
            and np.max(self.connection_errors) < CONNECTION_TOLERANCE
        )
        if self.phase == "APPROACH":
            if within_tolerance and camera_recent:
                if self.arrival_since is None:
                    self.arrival_since = now
                    print("[ARRIVAL] Conditions satisfied; starting 0.5 s timer")
                elif now - self.arrival_since >= ARRIVAL_TIME:
                    self.phase = "CLOSING"
                    self.close_started = now
                    self.arrival_since = None
                    print("[GRIPPER] CLOSING: closing only the MuJoCo fingers")
            else:
                if self.arrival_since is not None:
                    print(f"[ARRIVAL] Timer cancelled: within_tolerance={within_tolerance}, camera_recent={camera_recent}")
                self.arrival_since = None

        # The XML's gripper_position equality supplies a simulated finger target.
        # This is a visual closing test, not a simulated grasp of a physical object.
        if self.phase == "CLOSING":
            elapsed = now - self.close_started
            fraction = min(elapsed / self.args.close_time, 1.0)
            self.model.eq_data[self.gripper_eq_id, 0] = GRIPPER_CLOSED_ANGLE * fraction
            if fraction >= 1.0 and abs(self.plant.qpos[self.finger_qpos_id] - GRIPPER_CLOSED_ANGLE) < 0.02:
                self.phase = "CLOSED"
                print("[GRIPPER] CLOSED: simulated fingers closed; no object-detection feedback")
            elif elapsed >= self.args.close_time + 5.0:
                self.phase = "FAILED"
                print("[GRIPPER] FAILED: simulated fingers did not reach the closing angle")

        self.virtual.qfrc_applied[:] = 0.0
        self.mechanism.apply(self.model, self.virtual)
        self.vmc_torques = self.virtual.qfrc_applied[self.dof_ids].copy()
        if not np.all(np.isfinite(self.vmc_torques)):
            raise RuntimeError("Non-finite VMC torques; stopping simulation")
        # Bias compensation and final clipping belong to the simulated plant.
        # These torques are never placed in a ROS publisher or sent to hardware.
        self.plant.qfrc_applied[:] = 0.0
        self.plant.qfrc_applied[self.dof_ids] = np.clip(
            self.vmc_torques + self.plant.qfrc_bias[self.dof_ids],
            self.torque_limits[:, 0], self.torque_limits[:, 1],
        )
        mujoco.mj_step(self.model, self.plant)

    def draw(self, scene) -> None:
        """Show the measured target and three virtual connections in the viewer."""
        scene.ngeom = 0
        if self.targets is None:
            return
        positions = [target.state(self.model, self.virtual).position for target in self.targets]
        draw_sphere(scene, (positions[1] + positions[2]) / 2.0, radius=0.018, rgba=[1.0, 0.8, 0.1, 0.9])
        for site_id, p, color in zip(self.site_ids, positions, (
            [0.1, 0.9, 1.0, 0.8], [0.4, 1.0, 0.4, 0.8], [1.0, 0.4, 0.7, 0.8],
        )):
            draw_sphere(scene, p, radius=0.012, rgba=color)
            draw_capsule_line(scene, self.plant.site_xpos[site_id], p, radius=0.003, rgba=color)

    def log_status(self, now: float) -> None:
        """Called once per second, independently of new camera observations."""
        if self.targets is None:
            print("[WAIT] Waiting for the first valid base-frame camera pose; simulated robot held at home")
            return
        dwell = 0.0 if self.arrival_since is None else now - self.arrival_since
        print(
            f"[SIM] phase={self.phase}; sim_time={self.plant.time:.2f}s; "
            f"TCP error={np.round(self.error_xyz, 4)} m; "
            f"links={np.round(self.connection_errors, 4)} m; "
            f"camera_age={now - self.last_camera_time:.2f}s; arrival={dwell:.2f}s; "
            f"max_abs_tau={np.max(np.abs(self.vmc_torques)):.2f} Nm"
        )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, help="combined scene XML; otherwise use MUJOCO_MENAGERIE_PATH")
    parser.add_argument("--camera-topic", default="/vision/aruco_pose")
    parser.add_argument("--camera-timeout", type=float, default=0.8)
    parser.add_argument("--initial-keyframe", default="home")
    parser.add_argument("--stiffness", type=float, default=600.0)
    parser.add_argument("--damping", type=float, default=30.0)
    parser.add_argument("--max-force", type=float, default=10.0)
    parser.add_argument("--link-length", type=float, default=0.4)
    parser.add_argument("--timestep", type=float, default=0.001)
    parser.add_argument("--close-time", type=float, default=1.0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0, help="wall-clock seconds, including the initial wait; <=0 runs until stopped")
    args = parser.parse_args()
    values = (args.stiffness, args.damping, args.max_force, args.link_length,
              args.timestep, args.close_time, args.camera_timeout, args.duration)
    if not np.all(np.isfinite(values)):
        parser.error("numeric arguments must be finite")
    if args.stiffness < 0 or args.damping < 0:
        parser.error("stiffness and damping must be nonnegative")
    if min(args.max_force, args.link_length, args.timestep, args.close_time, args.camera_timeout) <= 0:
        parser.error("max-force, link-length, timestep, close-time and camera-timeout must be positive")
    return args


def main() -> None:
    args = parse_args()
    scene = (args.scene.expanduser().resolve() if args.scene is not None else
             resolve_menagerie_path() / "fr3_robotiq_2f_140" / "scene_fr3_robotiq.xml")
    if not scene.is_file():
        raise FileNotFoundError(f"Scene not found: {scene}; pass --scene with its actual path")
    simulation = CameraPickSimulation(scene, args)

    # ROS exists only at this boundary. Keeping it out of the physics class also
    # allows local headless dynamics checks when rclpy is not installed on a Mac.
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = rclpy.create_node("h2_camera_simulation")
    camera_lock = Lock()
    latest_camera_pose = None

    def camera_callback(msg):
        nonlocal latest_camera_pose
        if msg.header.frame_id != "base":
            return
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = np.array([msg.pose.orientation.w, msg.pose.orientation.x,
                      msg.pose.orientation.y, msg.pose.orientation.z])
        norm = np.linalg.norm(q)
        if not np.all(np.isfinite(p)) or not np.all(np.isfinite(q)) or not np.isfinite(norm) or norm < 1e-6:
            return
        with camera_lock:
            latest_camera_pose = (p, q / norm, time.perf_counter())

    camera_sub = node.create_subscription(PoseStamped, args.camera_topic, camera_callback, 1)

    def spin_ros():
        try:
            rclpy.spin(node)
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    ros_thread = Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    print(f"[SIM] Scene: {scene}")
    print(f"[ROS] Camera input: {args.camera_topic}; required frame_id=base")
    print("[SIM] Arm motion and finger closing are local MuJoCo operations; no hardware commands are published")
    t_start = time.perf_counter()
    next_tick = t_start
    next_render = t_start
    next_log = t_start
    try:
        context = nullcontext(None)
        if not args.headless:
            from mujoco import viewer
            context = viewer.launch_passive(simulation.model, simulation.plant)
        with context as v:
            if v is not None:
                v.cam.lookat[:] = [0.4, 0.0, 0.5]
                v.cam.distance = 2.0
                v.cam.azimuth = 135.0
                v.cam.elevation = -20.0
            while rclpy.ok() and (v is None or v.is_running()):
                now = time.perf_counter()
                if args.duration > 0 and now - t_start >= args.duration:
                    break
                with camera_lock:
                    camera_pose = latest_camera_pose
                    latest_camera_pose = None
                simulation.step(now, camera_pose)
                if now >= next_log:
                    simulation.log_status(now)
                    next_log = now + 1.0
                if v is not None and now >= next_render:
                    with v.lock():
                        simulation.draw(v.user_scn)
                    v.sync()
                    next_render = now + 1.0 / 30.0
                next_tick += args.timestep
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
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=2.0)
        node.destroy_node()


if __name__ == "__main__":
    main()

"""Control a simulated FR3 using forces computed in a second, virtual model.

The real scene is the simulated robot we can see and interact with. The virtual
scene contains a copy of the FR3 plus a moving reference mass on a rail. Three
site-based position actuators act like Cartesian springs and dampers between the
virtual end effector and that reference. At each time step, we copy the real
joint state into the virtual robot, read the resulting virtual joint torques,
and command those torques to the real robot. Only the virtual rail state is
carried from one time step to the next without being overwritten.
"""

from pathlib import Path
import time

import matplotlib
matplotlib.use("Agg")  # Render PNGs without opening a second GUI window.
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np

# -----------------------------------------------------------------------------
# 1. Load the two MuJoCo scenes
# -----------------------------------------------------------------------------
# Resolve paths relative to this file, so launching from another terminal
# directory does not change which XML files are loaded.
project_root = Path(__file__).resolve().parents[2]

scene_path_real = (
    project_root
    / "MuJoCo Projects"
    / "FR3"
    / "torque_control"
    / "model"
    / "scene.xml"
)
scene_path_virt = (
    project_root
    / "MuJoCo Projects"
    / "FR3"
    / "torque_control"
    / "model"
    / "scene_virtual.xml"
)

# MjModel is the compiled, mostly fixed description of a scene. Each MjData is
# its mutable state: qpos, qvel, ctrl, time, site positions, forces, and more.
# "real" here means the simulated plant, not a physical FR3 connected via ROS.
model_real = mujoco.MjModel.from_xml_path(str(scene_path_real))
model_virt = mujoco.MjModel.from_xml_path(str(scene_path_virt))
data_real = mujoco.MjData(model_real)
data_virt = mujoco.MjData(model_virt)

# -----------------------------------------------------------------------------
# 2. Look up the model elements used by the controller
# -----------------------------------------------------------------------------
# Names make XML readable, while arrays such as site_xpos and jnt_range are
# indexed by integer IDs. A name must be looked up in its own model: the same
# element may have a different ID in the real and virtual scenes.
home_id = mujoco.mj_name2id(
    model_real,
    mujoco.mjtObj.mjOBJ_KEY,
    "home"
)

attachment_site_id = mujoco.mj_name2id(
    model_real,
    mujoco.mjtObj.mjOBJ_SITE,
    "attachment_site"
)

reference_site_mobile_id = mujoco.mj_name2id(
    model_virt,
    mujoco.mjtObj.mjOBJ_SITE,
    "reference_site_mobile"
)

rail_joint_id_virt = mujoco.mj_name2id(
    model_virt,
    mujoco.mjtObj.mjOBJ_JOINT,
    "rail_joint"
)

rail_joint_id_real = mujoco.mj_name2id(
    model_real,
    mujoco.mjtObj.mjOBJ_JOINT,
    "rail_joint"
)

# A joint ID identifies the joint in MjModel. jnt_qposadr and jnt_dofadr then
# locate that joint's entries in qpos and qvel/qfrc_applied, respectively.
# The rail is a slide joint, so it occupies one scalar in each of these arrays.
# Its qpos is displacement along the rail, not the site's 3D world position.
rail_dof_id_virt = model_virt.jnt_dofadr[rail_joint_id_virt]
rail_qpos_id_virt = model_virt.jnt_qposadr[rail_joint_id_virt]

rail_dof_id_real = model_real.jnt_dofadr[rail_joint_id_real]
rail_qpos_id_real = model_real.jnt_qposadr[rail_joint_id_real]


# -----------------------------------------------------------------------------
# 3. Initialize the simulated robot
# -----------------------------------------------------------------------------
# The keyframe supplies the initial joint state of the real FR3. mj_forward
# computes quantities derived from that state, including site_xpos and bias
# forces, without integrating the simulation or advancing its clock. The
# virtual FR3 will receive this joint state at the start of the first loop.
mujoco.mj_resetDataKeyframe(model_real, data_real, home_id)
mujoco.mj_forward(model_real, data_real)


# -----------------------------------------------------------------------------
# 4. Prepare position-error recording
# -----------------------------------------------------------------------------
# Keep samples in memory for the plot and save only the resulting PNG.
# All positions are in the world frame and all errors use
#     error = mobile_reference_position - real_end_effector_position.
# A negative x error therefore means the reference is farther toward -x.
plot_path = Path(__file__).resolve().parent / "Plots" / "error_rail.png"
error_times = []
error_vectors = []
attachment_positions = []
reference_positions = []
# Write an intermediate result at 20 simulated seconds. The viewer can remain
# open afterward; closing it writes the complete run to the same files.
plot_saved_at_20s = False


def save_error_plot():
    """Plot world-frame positions (left) and signed axis errors (right).

    The dashed curve is the moving reference, not the end of the rail. The
    dotted horizontal line in each error panel marks perfect position match.
    """
    # Lists become arrays of shape (number_of_samples, 3), where columns are
    # x, y, and z. The time array has one entry per position/error sample.
    times = np.asarray(error_times)
    errors = np.asarray(error_vectors)
    attachments = np.asarray(attachment_positions)
    references = np.asarray(reference_positions)
    # Rows correspond to x/y/z. The left column compares the two positions;
    # the right column shows their difference with its sign preserved.
    figure, axes = plt.subplots(3, 2, sharex="col", figsize=(12, 8))

    for axis, label in enumerate(("x", "y", "z")):
        axes[axis, 0].plot(times, attachments[:, axis], label="End effector")
        axes[axis, 0].plot(times, references[:, axis], "--", label="Mobile reference")
        axes[axis, 0].set_ylabel(f"{label} position [m]")
        axes[axis, 0].grid(True)

        axes[axis, 1].plot(times, errors[:, axis])
        axes[axis, 1].axhline(0.0, color="0.5", linestyle=":")
        axes[axis, 1].set_ylabel(f"{label} error [m]")
        axes[axis, 1].grid(True)

    axes[0, 0].legend()
    axes[-1, 0].set_xlabel("Simulation time [s]")
    axes[-1, 1].set_xlabel("Simulation time [s]")
    figure.suptitle("Mobile reference and real end effector")
    figure.tight_layout()
    # Create Plots on the first save if it does not exist yet.
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(plot_path, dpi=150)
    plt.close(figure)
    print(f"Error plot saved to: {plot_path}")


# -----------------------------------------------------------------------------
# 5. Configure the external push along the virtual rail
# -----------------------------------------------------------------------------
# jnt_range gives limits in the slide joint's coordinate s. In the current
# XML, s ranges from 0 to 0.5 m and the joint axis is world -x. The Boolean
# remembers when the push has been turned off, so it cannot reactivate if the
# spring later pulls the carriage back from the end of the rail.
rail_upper_limit = model_virt.jnt_range[rail_joint_id_virt, 1]
rail_force_active = True


# -----------------------------------------------------------------------------
# 6. Control and advance both simulations
# -----------------------------------------------------------------------------
# The viewer displays only the real scene. Its rail is a visual copy of the
# virtual rail; the virtual scene still evolves even though it is not shown.
with mujoco.viewer.launch_passive(model_real, data_real) as viewer:
    # The viewer has its own thread. Hold its lock while changing camera state,
    # then synchronize once so the new view appears immediately.
    with viewer.lock():
        viewer.cam.lookat[:] = [0.0, 0.0, 0.5]
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90.0
        viewer.cam.elevation = -25.0

    viewer.sync()

    while viewer.is_running():
        # Wall-clock time is used only to pace the display. MuJoCo's simulated
        # time advances by model.opt.timestep whenever mj_step is called.
        step_start = time.time()

        # qpos[:7]/qvel[:7] belong to the seven FR3 joints in both models.
        # The virtual rail is the eighth coordinate and is intentionally not
        # copied: it keeps the position/velocity produced by previous steps.
        # Align the clocks before evaluating forces at the current real state.
        data_virt.qpos[:7] = data_real.qpos[:7]
        data_virt.qvel[:7] = data_real.qvel[:7]
        data_virt.time = data_real.time
        # Recompute kinematics, actuator forces, and bias forces without moving
        # anything. In particular, qfrc_actuator now reflects the three
        # virtual Cartesian spring-damper actuators at the measured FR3 pose.
        mujoco.mj_forward(model_virt, data_virt)

        # Both site positions now refer to the same simulation time. site_xpos
        # contains world-frame XYZ coordinates, unlike the rail's scalar qpos.
        # Copy the coordinates because MjData's arrays change at later steps.
        reference = data_virt.site_xpos[reference_site_mobile_id].copy()
        attachment = data_real.site_xpos[attachment_site_id].copy()
        error_times.append(data_real.time)
        error_vectors.append(reference - attachment)
        attachment_positions.append(attachment)
        reference_positions.append(reference)

        # Save and print once after 20 seconds of simulated (not wall) time.
        if not plot_saved_at_20s and data_real.time >= 20.0:
            save_error_plot()
            # At a 2 ms step, the in-memory array has about 10,000 rows.
            # Print one row per second and the latest 3-component error vector.
            sample_stride = max(1, round(1.0 / model_real.opt.timestep))
            sampled_errors = np.column_stack(
                (np.asarray(error_times)[::sample_stride], np.asarray(error_vectors)[::sample_stride])
            )
            with np.printoptions(precision=6, suppress=True):
                print(f"Error at t={data_real.time:.2f} s [ex, ey, ez] m: {error_vectors[-1]}")
                print("Error samples [time_s, ex_m, ey_m, ez_m]:")
                print(sampled_errors)
            plot_saved_at_20s = True

        # Stop applying the external push within 1 mm of the upper joint
        # limit. Setting the push to zero does not lock the carriage: its
        # inertia and the virtual springs may still move it afterward.
        if rail_force_active and data_virt.qpos[rail_qpos_id_virt] >= rail_upper_limit - 1e-3:
            rail_force_active = False
            print(f"Rail reached its upper limit at t={data_real.time:.2f} s; push turned off.")

        # qfrc_applied is indexed by DoF. Because this slide's axis is -x,
        # +20 N in its generalized coordinate pushes the carriage toward -x.
        # Set it every cycle so the post-limit value remains explicitly zero.
        data_virt.qfrc_applied[rail_dof_id_virt] = 20.0 if rail_force_active else 0.0

        # qfrc_actuator is a vector of generalized forces for all eight virtual
        # DoFs. Its first seven entries are the spring-generated joint torques;
        # the eighth belongs to the rail and is not sent to the real motors.
        # qfrc_bias supplies gravity and velocity-dependent inertial terms.
        # The seven real motor actuators have gear=1, so ctrl contains torques.
        bias_compensation = data_virt.qfrc_bias
        tau_virtual = data_virt.qfrc_actuator.copy()
        data_real.ctrl[:] = tau_virtual[:7] + bias_compensation[:7]

        # mj_step integrates ALL DoFs in each model. The virtual FR3 joints can
        # therefore drift during this step, but their predicted state is
        # discarded when we overwrite qpos[:7]/qvel[:7] next iteration. The
        # virtual rail is not overwritten and remains dynamically evolving.
        mujoco.mj_step(model_virt, data_virt)
        mujoco.mj_step(model_real, data_real)

        # The viewer shows the real scene only. Mirror the virtual rail's
        # scalar displacement and speed into its visible counterpart there.
        # Recompute real-scene site positions after this manual state change,
        # then send the updated scene and mouse interaction to the viewer.
        data_real.qpos[rail_qpos_id_real] = data_virt.qpos[rail_qpos_id_virt]
        data_real.qvel[rail_dof_id_real] = data_virt.qvel[rail_dof_id_virt]

        mujoco.mj_forward(model_real, data_real)
        viewer.sync()

        # Sleep for the unused part of the timestep so that, when computation
        # is fast enough, one simulated second takes about one wall second.
        computation_time = time.time() - step_start
        remaining_time = model_real.opt.timestep - computation_time

        if remaining_time > 0:
            time.sleep(remaining_time)

# On a normal viewer close, replace the 20-second snapshot with the full run.
# This also saves short experiments that ended before 20 simulated seconds.
if error_times:
    save_error_plot()

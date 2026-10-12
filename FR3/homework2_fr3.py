from pathlib import Path
import time

import matplotlib
matplotlib.use("Agg")  # Save the figure without opening another window.
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np

# -----------------------------------------------------------------------------
# 1. Finding and loading the FR3 scene
# -----------------------------------------------------------------------------
# Build an absolute path from this script, so the program does not depend on
# the directory from which it is launched.
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
# MjModel stores the compiled robot description. MjData stores the current
# simulation state and all quantities computed from it.
model_real = mujoco.MjModel.from_xml_path(str(scene_path_real))
model_virt = mujoco.MjModel.from_xml_path(str(scene_path_virt))
data_real = mujoco.MjData(model_real)    # Real Robot
data_virt = mujoco.MjData(model_virt)    # Virtual Robot

# -----------------------------------------------------------------------------
# 2. Finding the model elements used by this script
# -----------------------------------------------------------------------------
# MuJoCo uses integer IDs internally. mj_name2id converts names declared in the
# XML model into the IDs required by the Python API.
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

reference_site_id = mujoco.mj_name2id(
    model_virt,
    mujoco.mjtObj.mjOBJ_SITE,
    "reference_site"
)

# -----------------------------------------------------------------------------
# 3. Initializing the robot in the "home" configuration
# -----------------------------------------------------------------------------
# Copy the home keyframe into data, then update all quantities derived from the
# state without advancing simulation time.
mujoco.mj_resetDataKeyframe(model_real, data_real, home_id)
mujoco.mj_forward(model_real, data_real)

# Record the end-effector position, reference, and error for 10 simulated seconds.
error_times = []
error_vectors = []
attachment_positions = []
reference_positions = []
plot_saved = False
plot_path = Path(__file__).resolve().parent / "Plots" / "homework2_fr3_error.png"

with mujoco.viewer.launch_passive(model_real, data_real) as viewer:
    # Set the initial camera. The lock prevents concurrent access while the
    # viewer thread is using the same data.
    with viewer.lock():
        viewer.cam.lookat[:] = [0.0, 0.0, 0.5]
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90.0
        viewer.cam.elevation = -25.0

    viewer.sync()

    while viewer.is_running():
        # Measure computation time so the simulation can be paced in real time.
        step_start = time.time()



        data_virt.qpos[:] = data_real.qpos
        data_virt.qvel[:] = data_real.qvel
        data_virt.time = data_real.time
        mujoco.mj_forward(model_virt, data_virt)

        if not plot_saved:
            reference = data_virt.site_xpos[reference_site_id].copy()
            attachment = data_virt.site("attachment_site").xpos.copy()
            error = reference - attachment
            error_times.append(data_real.time)
            error_vectors.append(error)
            attachment_positions.append(attachment)
            reference_positions.append(reference)

            if data_real.time >= 20.0:
                errors = np.asarray(error_vectors)
                times = np.asarray(error_times)
                attachments = np.asarray(attachment_positions)
                references = np.asarray(reference_positions)
                figure, axes = plt.subplots(4, 1, sharex=True, figsize=(9, 9))

                for axis, label in enumerate(("x", "y", "z")):
                    axes[axis].plot(times, attachments[:, axis], label="End effector")
                    axes[axis].plot(times, references[:, axis], "--", label="Reference")
                    axes[axis].set_ylabel(f"{label} position [m]")
                    axes[axis].grid(True)
                axes[0].legend()

                axes[3].plot(times, np.linalg.norm(errors, axis=1))
                axes[3].set_xlabel("Simulation time [s]")
                axes[3].set_ylabel("Error magnitude [m]")
                axes[3].grid(True)

                figure.tight_layout()
                plot_path.parent.mkdir(parents=True, exist_ok=True)
                figure.savefig(plot_path, dpi=150)
                plt.close(figure)
                print(f"Error plot saved to: {plot_path}")
                plot_saved = True

        bias_compensation = data_virt.qfrc_bias
        tau_virtual = data_virt.qfrc_actuator.copy()
        data_real.ctrl[:] = tau_virtual + bias_compensation


        # Advance the dynamics using the torque command and update the viewer.
        mujoco.mj_step(model_real, data_real)
        viewer.sync()

        # mj_step is normally computed faster than the simulated timestep. This
        # pause keeps the simulation close to real time.
        computation_time = time.time() - step_start
        remaining_time = model_real.opt.timestep - computation_time

        if remaining_time > 0:
            time.sleep(remaining_time)
    

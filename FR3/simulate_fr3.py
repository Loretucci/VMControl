from pathlib import Path
import time

import mujoco
import mujoco.viewer

# -----------------------------------------------------------------------------
# 1. Finding and loading the FR3 scene
# -----------------------------------------------------------------------------
project_root = Path(__file__).resolve().parents[2]

scene_path = (
    project_root
    / "MuJoCo Menagerie"
    / "franka_fr3"
    / "scene.xml"
)

model = mujoco.MjModel.from_xml_path(str(scene_path))
data = mujoco.MjData(model)

# -----------------------------------------------------------------------------
# 2. Finding the model elements used by this script
# -----------------------------------------------------------------------------
home_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home"
)

attachment_site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "attachment_site"
)

# -----------------------------------------------------------------------------
# 3. Initializing the robot in the "home" configuration
# -----------------------------------------------------------------------------
mujoco.mj_resetDataKeyframe(model, data, home_id)
mujoco.mj_forward(model, data)

# Copy the keyframe control targets. copy() creates an independent array, so
# home_ctrl does not change when data.ctrl is modified during the simulation.
home_ctrl = data.ctrl.copy()

# -----------------------------------------------------------------------------
# 4. Experiment parameters
# -----------------------------------------------------------------------------
simulation_duration = 10.0
number_of_steps = int(simulation_duration / model.opt.timestep)

# Joint 1 remains at its home target for one second. Its target then moves
# linearly from 0 to 0.5 rad over two seconds.
movement_start = 1.0
movement_duration = 2.0
target_position = 0.5

print("qpos iniziale:", data.qpos)
print("ctrl iniziale:", data.ctrl)

# -----------------------------------------------------------------------------
# 5. Passive viewer and simulation loop
# -----------------------------------------------------------------------------
# With a passive viewer, the script remains responsible for calling mj_step and
# keeping pace with real time. On macOS, this file must be run with mjpython.
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set the initial camera. The lock prevents concurrent access while the
    # viewer thread is using the same data.
    with viewer.lock():
        viewer.cam.lookat[:] = [0.0, 0.0, 0.5]
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90.0
        viewer.cam.elevation = -25.0

    viewer.sync()

    for step in range(number_of_steps):
        if not viewer.is_running():
            break

        step_start = time.time()

        # Restore the home targets for every joint at each step. We then modify
        # only the target of joint 1.
        data.ctrl[:] = home_ctrl

        if data.time >= movement_start:
            # alpha increases from 0 to 1 during the ramp and then remains at 1.
            alpha = min(
                (data.time - movement_start) / movement_duration,
                1.0
            )

            data.ctrl[0] = (
                (1.0 - alpha) * home_ctrl[0]
                + alpha * target_position
            )

        # Compute the dynamics, integrate qpos/qvel, and advance data.time.
        mujoco.mj_step(model, data)

        # Every 100 steps, inspect both joint tracking and the Cartesian
        # kinematics of the robot attachment site.
        if step % 100 == 0:
            x, y, z = data.site_xpos[attachment_site_id]

            print(
                f"t={data.time:.2f}, "
                f"desired={data.ctrl[0]:.3f}, "
                f"actual={data.qpos[0]:.3f}, "
                f"error={data.ctrl[0] - data.qpos[0]:.4f}, "
                f"attachment_site=({x:.3f}, {y:.3f}, {z:.3f})"
            )

        # Send the updated state to the viewer and receive GUI inputs.
        viewer.sync()

        # mj_step is normally computed faster than the simulated timestep. This
        # pause keeps the simulation close to real time.
        computation_time = time.time() - step_start
        remaining_time = model.opt.timestep - computation_time

        if remaining_time > 0:
            time.sleep(remaining_time)

    # -------------------------------------------------------------------------
    # 6. State reached at the end of the simulation
    # -------------------------------------------------------------------------
    print("qpos finale:", data.qpos)
    print("ctrl finale:", data.ctrl)
    print("errore finale:", data.ctrl - data.qpos)

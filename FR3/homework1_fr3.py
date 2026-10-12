from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

# -----------------------------------------------------------------------------
# 1. Finding and loading the FR3 scene
# -----------------------------------------------------------------------------
# Build an absolute path from this script, so the program does not depend on
# the directory from which it is launched.
project_root = Path(__file__).resolve().parents[2]

scene_path = (
    project_root
    / "MuJoCo Projects"
    / "FR3"
    / "torque_control"
    / "model"
    / "scene.xml"
)

# MjModel stores the compiled robot description. MjData stores the current
# simulation state and all quantities computed from it.
model = mujoco.MjModel.from_xml_path(str(scene_path))
data = mujoco.MjData(model)

# -----------------------------------------------------------------------------
# 2. Finding the model elements used by this script
# -----------------------------------------------------------------------------
# MuJoCo uses integer IDs internally. mj_name2id converts names declared in the
# XML model into the IDs required by the Python API.
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

reference_site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "reference_site"
)

# -----------------------------------------------------------------------------
# 3. Initializing the robot in the "home" configuration
# -----------------------------------------------------------------------------
# Copy the home keyframe into data, then update all quantities derived from the
# state without advancing simulation time.
mujoco.mj_resetDataKeyframe(model, data, home_id)
mujoco.mj_forward(model, data)

# -----------------------------------------------------------------------------
# 4. Experiment parameters
# -----------------------------------------------------------------------------
# Cartesian PID gains.
kp = 100
kv = 20
ki = 5

# The reference site is fixed in the world frame. copy() stores an independent
# target vector instead of a view into MuJoCo's data arrays.
reference_position = data.site_xpos[reference_site_id].copy()

# mj_jacSite writes into these preallocated matrices at every simulation step.
jacobian_position = np.zeros((3, model.nv))
jacobian_rotation = np.zeros((3, model.nv))

# The integral state is Cartesian: one accumulated error for x, y, and z.
integral = np.zeros((3,))

# Joint torque limits from the FR3 model.
tau_maxlimit = np.array([87, 87, 87, 87, 12, 12, 12])
tau_minlimit = np.array([-87, -87, -87, -87, -12, -12, -12])

# Print controller diagnostics every 0.5 seconds of simulated time.
print_interval = max(1, int(0.5 / model.opt.timestep))
step_count = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
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

        # Compute the current linear and angular Jacobians of the attachment
        # site in the world frame.
        mujoco.mj_jacSite(
            model,
            data,
            jacobian_position,
            jacobian_rotation,
            attachment_site_id
        )

        # Position error points from the current attachment position towards the
        # fixed reference. integral_error is the increment for this timestep.
        current_position = data.site_xpos[attachment_site_id]
        position_error = reference_position - current_position
        integral_error = position_error * model.opt.timestep

        # Convert generalized joint velocities into the linear velocity of the
        # attachment site: z_dot = J_position * q_dot.
        cartesian_vel = jacobian_position @ data.qvel

        # Proportional and derivative Cartesian forces.
        spring = kp * position_error
        damper = -kv * cartesian_vel

        # Test a candidate integral update. data.qfrc_bias compensates gravity,
        # Coriolis, and centrifugal bias forces.
        integral_candidate = integral + integral_error
        force_candidate = spring + damper + ki * integral_candidate
        torque_candidate = jacobian_position.T @ force_candidate
        bias_compensation = data.qfrc_bias
        tau_candidate = torque_candidate + bias_compensation

        integral_update_allowed = np.all(
            (tau_candidate >= tau_minlimit)
            & (tau_candidate <= tau_maxlimit)
        )

        # Conditional integration prevents windup when any joint would saturate.
        if integral_update_allowed:
            integral = integral_candidate

        # Compute the command from the last accepted integral state.
        force = spring + damper + ki * integral
        torque = jacobian_position.T @ force
        tau = torque + bias_compensation

        # Enforce the physical joint torque limits on the final command.
        tau_limited = np.clip(tau, tau_minlimit, tau_maxlimit)
        data.ctrl[:] = tau_limited

        # Two periodic diagnostics show whether the integral update is accepted
        # and whether the final command has to be clipped.
        if step_count % print_interval == 0:
            print(
                f"t={data.time:.2f}, "
                f"error={np.array2string(position_error, precision=3)}, "
                f"integral={np.array2string(integral, precision=3)}, "
                f"integral accepted={integral_update_allowed}"
            )
            print(
                f"tau candidate={np.array2string(tau_candidate, precision=2)}, "
                f"tau applied={np.array2string(tau_limited, precision=2)}, "
                f"command clipped={not np.allclose(tau, tau_limited)}"
            )

        # Advance the dynamics using the torque command and update the viewer.
        mujoco.mj_step(model, data)
        viewer.sync()
        step_count += 1

        # mj_step is normally computed faster than the simulated timestep. This
        # pause keeps the simulation close to real time.
        computation_time = time.time() - step_start
        remaining_time = model.opt.timestep - computation_time

        if remaining_time > 0:
            time.sleep(remaining_time)
    

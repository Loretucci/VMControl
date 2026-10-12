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

rail_dof_id_virt = model_virt.jnt_dofadr[rail_joint_id_virt]
rail_qpos_id_virt = model_virt.jnt_qposadr[rail_joint_id_virt]

rail_dof_id_real = model_real.jnt_dofadr[rail_joint_id_real]
rail_qpos_id_real = model_real.jnt_qposadr[rail_joint_id_real]


# -----------------------------------------------------------------------------
# 3. Initializing the robot in the "home" configuration
# -----------------------------------------------------------------------------
# Copy the home keyframe into data, then update all quantities derived from the
# state without advancing simulation time.
mujoco.mj_resetDataKeyframe(model_real, data_real, home_id)
mujoco.mj_forward(model_real, data_real)


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
        

        data_virt.qpos[:7] = data_real.qpos[:7]
        data_virt.qvel[:7] = data_real.qvel[:7]
        data_virt.time = data_real.time
        mujoco.mj_forward(model_virt, data_virt)


        
        bias_compensation = data_virt.qfrc_bias
        tau_virtual = data_virt.qfrc_actuator.copy()
        data_real.ctrl[:] = tau_virtual[:7] + bias_compensation[:7]

        mujoco.mj_step(model_virt, data_virt)
        mujoco.mj_step(model_real, data_real)

        data_real.qpos[rail_qpos_id_real] = data_virt.qpos[rail_qpos_id_virt]
        data_real.qvel[rail_dof_id_real] = data_virt.qvel[rail_dof_id_virt]

        mujoco.mj_forward(model_real, data_real)
        viewer.sync()

        # mj_step is normally computed faster than the simulated timestep. This
        # pause keeps the simulation close to real time.
        computation_time = time.time() - step_start
        remaining_time = model_real.opt.timestep - computation_time

        if remaining_time > 0:
            time.sleep(remaining_time)
    

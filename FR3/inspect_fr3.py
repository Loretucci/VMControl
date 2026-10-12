from pathlib import Path

import mujoco

# -----------------------------------------------------------------------------
# 1. Finding and loading the model
# -----------------------------------------------------------------------------
# __file__ is the path of this script. Going up two directories gives us the
# "Progetto Tesi" root, regardless of the terminal's current directory.
project_root = Path(__file__).resolve().parents[2]

scene_path = (
    project_root
    / "MuJoCo Menagerie"
    / "franka_fr3"
    / "scene.xml"
)

# MjModel contains the compiled and mostly constant robot description.
# MjData contains the current state and computation results for one simulation.
model = mujoco.MjModel.from_xml_path(str(scene_path))
data = mujoco.MjData(model)

# -----------------------------------------------------------------------------
# 2. Looking for the "home" keyframe
# -----------------------------------------------------------------------------
# Model elements are stored using numerical IDs. mj_name2id translates the
# readable name "home" into the corresponding keyframe ID.
home_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home"
)

# -----------------------------------------------------------------------------
# 3. Model structure inspection
# -----------------------------------------------------------------------------
print("nq:", model.nq)  # Number of configuration coordinates.
print("nv:", model.nv)  # Number of generalized velocities/ degrees of freedom.
print("nu:", model.nu)  # Number of control inputs.
print("numero di keyframe:", model.nkey)

print("ID del keyframe home:", home_id)
print("qpos nel keyframe:", model.key_qpos[home_id])
print("ctrl nel keyframe:", model.key_ctrl[home_id])

# -----------------------------------------------------------------------------
# 4. Keyframe state initialization
# -----------------------------------------------------------------------------
# The keyframe is stored in the model, but it is not selected automatically when
# MjData is created. This function copies the saved keyframe state into data.
mujoco.mj_resetDataKeyframe(model, data, home_id)

# The reset changes the primary variables (qpos, qvel, ctrl, ...). mj_forward
# updates all derived quantities without advancing simulation time.
mujoco.mj_forward(model, data)

# -----------------------------------------------------------------------------
# 5. Verifying the current state
# -----------------------------------------------------------------------------
print("data.qpos dopo il reset:", data.qpos)
print("data.ctrl dopo il reset:", data.ctrl)
print("data.qvel dopo il reset:", data.qvel)

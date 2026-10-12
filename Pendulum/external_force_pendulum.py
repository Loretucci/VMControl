from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

xml_path = Path(__file__).with_name("single_pendulum.xml")

model = mujoco.MjModel.from_xml_path(str(xml_path))
data = mujoco.MjData(model)
data.qpos[0] = 0.0
mujoco.mj_forward(model,data)

bar_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_BODY,
    "bar"
)

tip_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "tip"
)

print("Numero di corpi:", model.nbody)
print("ID di bar:", bar_id)
print("Dimensioni xfrc_applied:", data.xfrc_applied.shape)
print(data.xfrc_applied)

simulation_duration = 10.0
number_of_steps = int(simulation_duration / model.opt.timestep)

force = np.array([1.0, 0.0, 0.0])
torque = np.zeros(3)

with mujoco.viewer.launch_passive(model, data) as viewer:

    #to set properly the camera
    with viewer.lock():
        viewer.cam.lookat[:] = [0.0, 0.0, 1.5]
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 90.0
        viewer.cam.elevation = -20.0
    
    viewer.sync()

    for step in range(number_of_steps):
        if not viewer.is_running():
            break

        data.qfrc_applied[:] = 0.0
        
        if data.time < 1:
            #data.xfrc_applied[bar_id,0] = 1.0

            mujoco.mj_applyFT(
                model,
                data,
                force,
                torque,
                data.site_xpos[tip_id],
                bar_id,
                data.qfrc_applied
            )
        #else:
            #data.xfrc_applied[bar_id,:] = 0.0

        step_start =time.time()

        mujoco.mj_step(model, data)

        if step % 100 == 0:
            print(
                f"t={data.time:.2f}, "
                f"q={data.qpos[0]:.3f}, "
                f"dq={data.qvel[0]:.3f}, "
                #f"Fx={data.xfrc_applied[bar_id, 0]:.1f}"
                f"tau={data.qfrc_applied[0]:.3f}"
            )
        
        viewer.sync()

        computation_time = time.time() - step_start
        remaning_time = model.opt.timestep - computation_time

        if remaning_time > 0:
            time.sleep(remaning_time)
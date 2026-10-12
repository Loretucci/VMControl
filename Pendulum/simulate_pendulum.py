from pathlib import Path
import time

import mujoco
import mujoco.viewer

xml_path = Path(__file__).with_name("single_pendulum.xml")

model = mujoco.MjModel.from_xml_path(str(xml_path))
data = mujoco.MjData(model)
data.qpos[0] = 0.5
mujoco.mj_forward(model,data)


simulation_duration = 10.0
number_of_steps = int(simulation_duration / model.opt.timestep)

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

        step_start =time.time()

        mujoco.mj_step(model, data)
        viewer.sync()

        computation_time = time.time() - step_start
        remaning_time = model.opt.timestep - computation_time

        if remaning_time > 0:
            time.sleep(remaning_time)

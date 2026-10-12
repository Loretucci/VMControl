from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

xml_path = Path(__file__).with_name("actuated_pendulum.xml")

model = mujoco.MjModel.from_xml_path(str(xml_path))
data = mujoco.MjData(model)
data.qpos[0] = 0.0
data.ctrl[0] = 0.5
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
        q = data.qpos[0]
        tau_comp = 4.905 * np.sin(q)
        data.ctrl[1] = tau_comp
        mujoco.mj_step(model, data)

        if step % 100 == 0:
            print(
                f"t={data.time:.2f}, "
                f"q_des={data.ctrl[0]:.3f}, "
                f"q={data.qpos[0]:.3f}, "
                f"tau_position={data.actuator_force[0]:.3f}, "
                f"tau_comp={data.actuator_force[1]:.3f}, "
                f"tau_total={data.qfrc_actuator[0]:.3f}"
            )
        viewer.sync()

        computation_time = time.time() - step_start
        remaning_time = model.opt.timestep - computation_time

        if remaning_time > 0:
            time.sleep(remaning_time)
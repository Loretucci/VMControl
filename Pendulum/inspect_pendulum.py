from pathlib import Path
import mujoco

xml_path = Path(__file__).with_name("single_pendulum.xml")

model = mujoco.MjModel.from_xml_path(str(xml_path))
data = mujoco.MjData(model)
data.qpos[0] = 0.5
mujoco.mj_forward(model,data) #asks to MuJoCo to compute all quantities depending on the new state
print("qacc iniziale:", data.qacc)

print("nq:", model.nq)
print("nv:", model.nv)
print("qpos:", data.qpos)
print("qvel:", data.qvel)
print("time:", data.time)

print("timestep:", model.opt.timestep)

simulation_duration = 3.0
number_of_steps = int(simulation_duration/ model.opt.timestep)

for step in range(number_of_steps):
    mujoco.mj_step(model, data)

    if step % 100 == 0:
        print(
            f"time = {data.time:.3f}, "
            f"qpos = {data.qpos[0]: .4f}, "
            f"qvel = {data.qvel[0]: .4f} "
        )

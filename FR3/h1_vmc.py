import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from mujoco_vmc import (
    LinearDamper,
    MuJoCoSite,
    TanhSpring,
    VirtualConnection,
    VirtualMechanism,
)


def build_ee_link_model(
    spec,
    link_length: float = 0.20,
):
    """Add a rigid extension and its tip site to the last FR3 link."""

    # 1. Resolve robot end-effector link body in spec
    ee_spec = spec.body("fr3_link7")

    L = float(link_length)

    # 2. Extend rigid link from native attachment_site (pos=[0, 0, 0.107]) along robot local +z axis:
    ee_spec.add_site(name="ee_link_tip", pos=[0.0, 0.0, 0.107 + L], size=[0.012, 0.0, 0.0], rgba=[1.0, 0.2, 0.2, 0.8])
    ee_spec.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        fromto=[0.0, 0.0, 0.107, 0.0, 0.0, 0.107 + L],
        size=[0.007, 0.0, 0.0],
        rgba=[0.1, 0.6, 1.0, 0.7],
        contype=0,
        conaffinity=0,
    )

def build_ref_site(spec, pos_x, pos_y, pos_z):
    """Place the fixed reference site already defined in scene1.xml."""
    spec.site("reference_site").pos = [pos_x, pos_y, pos_z]

def main():
    # Parameters for this first simulation; no command-line parser is needed.
    link_length = 0.20
    stiffness = 500.0
    damping = 20.0
    max_force = 60.0
    reference_position = (0.7, 0.0, 0.3)

    scene_xml = Path(__file__).resolve().parent / "torque_control" / "model" / "scene1.xml"
    if not scene_xml.exists():
        raise FileNotFoundError(f"Scene XML not found at {scene_xml}.")

    spec = mujoco.MjSpec.from_file(str(scene_xml))
    build_ee_link_model(spec, link_length)
    build_ref_site(spec, *reference_position)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    mujoco.mj_forward(model, data)

    tip_id = model.site("ee_link_tip").id
    ref_id = model.site("reference_site").id
    r_tip_site = MuJoCoSite(tip_id, "robot_ee_tip")
    r_fix_site = MuJoCoSite(ref_id, "fixed_target")
    conn_base = VirtualConnection(
        source=r_tip_site,
        target=r_fix_site,
        components=[
            TanhSpring(stiffness, max_force=max_force),
            LinearDamper(damping),
        ],
        name="link_base_spring",
    )
    mechanism = VirtualMechanism([conn_base])

    print("Initial tip position:", data.site_xpos[tip_id].copy())
    print("Fixed reference position:", data.site_xpos[ref_id].copy())

    def step_iteration():
        """Compute the virtual force and advance the physical simulation once."""
        data.qfrc_applied[:] = 0.0
        # Refresh site positions, velocities, and bias forces for this state.
        mujoco.mj_forward(model, data)
        mechanism.apply(model, data)
        data.qfrc_applied[:] += data.qfrc_bias[:]
        mujoco.mj_step(model, data)

    # Physics runs at model.opt.timestep; the viewer is refreshed at about 60 Hz.
    frame_interval = 1.0 / 60.0
    steps_per_frame = max(1, round(frame_interval / model.opt.timestep))
    with mujoco.viewer.launch_passive(model, data) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = [0.4, 0.0, 0.4]
            viewer.cam.distance = 1.5

        while viewer.is_running():
            frame_start = time.perf_counter()
            for _ in range(steps_per_frame):
                step_iteration()
            viewer.sync()
            time.sleep(max(0.0, frame_interval - (time.perf_counter() - frame_start)))

    mujoco.mj_forward(model, data)
    final_error = data.site_xpos[ref_id] - data.site_xpos[tip_id]
    print("Final tip-to-reference error:", final_error)
    print("Final error magnitude:", np.linalg.norm(final_error))

if __name__ == "__main__":
    main()

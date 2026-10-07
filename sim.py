import os
import time
import mujoco
import mujoco.viewer

def assemble_and_view():
    ur5e_path = os.path.join("assets", "universal_robots_ur5e", "ur5e.xml")
    robotiq_path = os.path.join("assets", "robotiq_2f85", "2f85.xml")

    arm = mujoco.MjSpec.from_file(ur5e_path)
    gripper = mujoco.MjSpec.from_file(robotiq_path)

    # Mount the gripper on the arm's existing flange site
    site = arm.site("attachment_site")
    arm.attach(gripper, site=site, prefix="gripper_")

    # Floor + light go straight into the same spec
    arm.worldbody.add_light(pos=[0, -0.2, 1], dir=[0, 0.2, -0.8],
                        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    arm.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE,
                           size=[0, 0, 0.05])

    model = arm.compile()
    data = mujoco.MjData(model)

    print(f"nq={model.nq}, nu={model.nu}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)

if __name__ == "__main__":
    assemble_and_view()
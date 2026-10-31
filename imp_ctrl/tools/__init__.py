import mujoco
import mujoco.viewer
import numpy as np

from geodude_assets import get_geodude_path
from imp_ctrl.controller.impedance_controller import ImpedanceController
from imp_ctrl.robot_model import ARM_JOINTS


def main():
    model = mujoco.MjModel.from_xml_path(str(get_geodude_path()))
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, model.key("ready").id)
    mujoco.mj_forward(model, data)

    side = "left"
    joints = [f"{side}_ur5e/{j}" for j in ARM_JOINTS]
    site = f"{side}_ur5e/gripper_attachment_site"

    jids = {model.joint(j).id for j in joints}
    for a in range(model.nu):
        if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT and model.actuator_trnid[a, 0] in jids:
            model.actuator_gainprm[a, :] = 0
            model.actuator_biasprm[a, :] = 0

    ctrl = ImpedanceController(model, data, site_name=site, joint_names=joints)

    x_home = data.site_xpos[ctrl.site_id].copy()
    x_des = x_home + np.array([0.0, 0.0, 0.05])

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            data.qfrc_applied[ctrl.dof] = ctrl.compute(data, x_des)
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    main()

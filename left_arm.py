import mujoco, mujoco.viewer
import numpy as np
from geodude_assets import get_geodude_path
from impedance_controller import ImpedanceController 
from robot_model import ARM_JOINTS               # your single-arm joint names

model = mujoco.MjModel.from_xml_path(str(get_geodude_path()))
data = mujoco.MjData(model)

# Start at the lab's home pose, then compute kinematics
mujoco.mj_resetDataKeyframe(model, data, model.key("ready").id)
mujoco.mj_forward(model, data)

# Names in the lab model have a prefix (check against Step 3 output)
side = "left"
joints = [f"{side}_ur5e/{j}" for j in ARM_JOINTS]
site = f"{side}_ur5e/gripper_attachment_site"

# The lab arms use POSITION actuators, but your controller outputs TORQUES.
# Switch off the position actuators for these joints so they don't fight you.
jids = {model.joint(j).id for j in joints}
for a in range(model.nu):
    if (model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT
            and model.actuator_trnid[a, 0] in jids):
        model.actuator_gainprm[a, :] = 0
        model.actuator_biasprm[a, :] = 0

ctrl = ImpedanceController(model, data, site_name=site, joint_names=joints)

x_home = data.site_xpos[ctrl.site_id].copy()
x_des = x_home + np.array([0.0, 0.0, 0.05])   # 5 cm up from home

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Torques go to qfrc_applied, not data.ctrl
        data.qfrc_applied[ctrl.dof] = ctrl.compute(data, x_des)
        mujoco.mj_step(model, data)
        viewer.sync()
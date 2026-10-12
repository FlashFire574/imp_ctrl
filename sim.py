from curses.ascii import ctrl
import os
import time
import numpy as np
import mujoco
import matplotlib.pyplot as plt
import mujoco.viewer

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
Q_HOME = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

# Impedance gains (tune these)
KP = np.diag([500, 500, 500, 50, 50, 50])   # N/m, Nm/rad
KD = np.diag([60, 60, 60, 8, 8, 8])

def build_model():
    base = os.path.dirname(os.path.abspath(__file__))
    ur5e_path = os.path.join(base, "assets", "universal_robots_ur5e", "ur5e.xml")
    robotiq_path = os.path.join(base, "assets", "robotiq_2f85", "2f85.xml")

    arm = mujoco.MjSpec.from_file(ur5e_path)
    gripper = mujoco.MjSpec.from_file(robotiq_path)

    site = arm.site("attachment_site")
    arm.attach(gripper, site=site, prefix="gripper_")

    arm.worldbody.add_light(pos=[0, -0.2, 1], dir=[0, 0.2, -0.8],
                            type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    arm.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE,
                           size=[0, 0, 0.05])

    # Settings the gripper model wants (parent values win on attach)
    arm.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    arm.option.impratio = 10

    model = arm.compile()
    make_arm_torque_controlled(model)
    return model


def make_arm_torque_controlled(model, n=6):
    """Turn the first n (arm) position actuators into pure torque motors."""
    model.actuator_gainprm[:n, 0] = 1.0
    model.actuator_biasprm[:n, :3] = 0.0
    model.actuator_ctrllimited[:n] = 1
    limits = np.array([150, 150, 150, 28, 28, 28], dtype=float)
    model.actuator_ctrlrange[:n, 0] = -limits
    model.actuator_ctrlrange[:n, 1] = limits

class ImpedanceController:
    def __init__(self, model, data, site_name="attachment_site"):
        self.model = model
        self.site_id = model.site(site_name).id
        self.dof = np.array([model.jnt_dofadr[model.joint(j).id] for j in ARM_JOINTS])
        self.jacp = np.zeros((3, model.nv))
        self.jacr = np.zeros((3, model.nv))
        self.quat = np.zeros(4)
        self.quat_neg = np.zeros(4)
        self.quat_err = np.zeros(4)
        self.ori_err = np.zeros(3)

        # Desired orientation = orientation at home pose
        self.quat_des = np.zeros(4)
        mujoco.mju_mat2Quat(self.quat_des, data.site_xmat[self.site_id])

    def compute(self, data, x_des):
        m = self.model
        mujoco.mj_jacSite(m, data, self.jacp, self.jacr, self.site_id)
        J = np.vstack([self.jacp[:, self.dof], self.jacr[:, self.dof]])  # 6x6

        pos_err = x_des - data.site_xpos[self.site_id]

        mujoco.mju_mat2Quat(self.quat, data.site_xmat[self.site_id])
        mujoco.mju_negQuat(self.quat_neg, self.quat)
        mujoco.mju_mulQuat(self.quat_err, self.quat_des, self.quat_neg)
        mujoco.mju_quat2Vel(self.ori_err, self.quat_err, 1.0)

        err = np.concatenate([pos_err, self.ori_err])
        xdot = J @ data.qvel[self.dof]

        wrench = KP @ err - KD @ xdot
        tau = J.T @ wrench
        tau += data.qfrc_bias[self.dof]  # gravity + Coriolis compensation
        return tau

def main():
    model = build_model()
    data = mujoco.MjData(model)

    names = [model.actuator(i).name for i in range(model.nu)]
    print("Actuators:", names)

    arm_dof = [model.jnt_dofadr[model.joint(j).id] for j in ARM_JOINTS]
    arm_qadr = [model.jnt_qposadr[model.joint(j).id] for j in ARM_JOINTS]
    data.qpos[arm_qadr] = Q_HOME
    mujoco.mj_forward(model, data)

    ctrl = ImpedanceController(model, data)
    x_start = data.site_xpos[ctrl.site_id].copy()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        log = {"t": [], "pos": [], "target": [], "ori_err": []}
        while viewer.is_running():
            step_start = time.time()
            t = data.time

            CIRCLE = False
            if CIRCLE:
                # Example target: small circle in the XY plane
                target = x_start + np.array([0.1 * np.sin(t), 0.1 * (np.cos(t) - 1), 0.0])
            else:
                target = x_start

            data.ctrl[:6] = ctrl.compute(data, target)
            log["t"].append(data.time)
            log["pos"].append(data.site_xpos[ctrl.site_id].copy())
            log["target"].append(target.copy())
            log["ori_err"].append(np.linalg.norm(ctrl.ori_err))
            data.ctrl[6] = 0  # gripper: 0 open, 255 closed

            mujoco.mj_step(model, data)
            viewer.sync()

            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)

        pos = np.array(log["pos"])
    tgt = np.array(log["target"])
    
    # Calculate pure 2D (XY) position error and 3D position error separately
    err_2d = np.linalg.norm(pos[:, :2] - tgt[:, :2], axis=1)
    err_3d = np.linalg.norm(pos - tgt, axis=1)
    
    # Calculate deviation strictly along the vertical Z-axis relative to the starting height
    z_deviation = (pos[:, 2] - x_start[2]) * 1000  # converted to mm

    # Console Performance Metrics
    print(f"mean XY tracking error: {err_2d.mean()*1000:.1f} mm, max: {err_2d.max()*1000:.1f} mm")
    print(f"mean 3D tracking error: {err_3d.mean()*1000:.1f} mm, max: {err_3d.max()*1000:.1f} mm")
    print(f"z drift (peak-to-peak): {(pos[:,2].max()-pos[:,2].min())*1000:.1f} mm")
    print(f"max orientation error: {np.degrees(max(log['ori_err'])):.2f} deg")

    # Plotting with Axes Labels
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))

    # XY Spatial Path Comparison
    ax[0].plot(tgt[:, 0], tgt[:, 1], "--", label="target")
    ax[0].plot(pos[:, 0], pos[:, 1], label="actual")
    ax[0].set_aspect("equal")
    ax[0].set_title("XY Spatial Path Comparison")
    ax[0].set_xlabel("X Position (m)")
    ax[0].set_ylabel("Y Position (m)")
    ax[0].legend()
    ax[0].grid(True, linestyle="--", alpha=0.6)

    # Tracking Errors Over Time
    ax[1].plot(log["t"], err_2d * 1000, label="2D Error (XY Plane)", color="crimson")
    ax[1].plot(log["t"], z_deviation, label="Z Deviation from Start", color="royalblue", alpha=0.7)
    ax[1].set_title("Tracking Deviations Over Time")
    ax[1].set_xlabel("Time (s)")
    ax[1].set_ylabel("Error (mm)")
    ax[1].legend()
    ax[1].grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
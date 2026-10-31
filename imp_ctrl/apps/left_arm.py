#!/usr/bin/env python3
"""
Run the impedance controller on the Geodude lab model (two UR5e arms).
"""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from geodude_assets import get_geodude_path
from imp_ctrl.controller.impedance_controller import ImpedanceController
from imp_ctrl.teleop.quest_teleop import QuestTeleop

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
SITE = "gripper_attachment_site"


def disable_servos(model, joint_names):
    jids = {model.joint(j).id for j in joint_names}
    for a in range(model.nu):
        if model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT and model.actuator_trnid[a, 0] in jids:
            model.actuator_gainprm[a, :] = 0
            model.actuator_biasprm[a, :] = 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=["left", "right", "both"], default="both")
    parser.add_argument("--static", action="store_true")
    parser.add_argument("--axis", choices=["x", "y", "z"], default="y")
    parser.add_argument("--length", type=float, default=0.1)
    parser.add_argument("--period", type=float, default=6.0)
    parser.add_argument("--teleop", action="store_true")
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--max-lead", type=float, default=0.10)
    args = parser.parse_args()

    sides = ["left", "right"] if args.arm == "both" else [args.arm]

    model = mujoco.MjModel.from_xml_path(str(get_geodude_path()))
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, model.key("ready").id)
    mujoco.mj_forward(model, data)
    model.body_gravcomp[:] = 0

    arms = {}
    for side in sides:
        joints = [f"{side}_ur5e/{j}" for j in ARM_JOINTS]
        disable_servos(model, joints)
        ctrl = ImpedanceController(model, data, site_name=f"{side}_ur5e/{SITE}", joint_names=joints)
        arms[side] = {
            "ctrl": ctrl,
            "x_start": data.site_xpos[ctrl.site_id].copy(),
            "log": {"t": [], "pos": [], "target": [], "ori_err": []},
        }

    teleop = QuestTeleop(sides, scale=args.scale, yaw_deg=args.yaw) if args.teleop else None
    direction = np.zeros(3)
    direction["xyz".index(args.axis)] = 1.0

    def get_target(x_start, t):
        if args.static:
            return x_start
        s = 0.5 * (1.0 - np.cos(2.0 * np.pi * t / args.period))
        return x_start + args.length * s * direction

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            for side, arm in arms.items():
                ctrl = arm["ctrl"]
                x_now = data.site_xpos[ctrl.site_id].copy()

                if teleop is not None:
                    target = teleop.get_target(side, x_now)
                    lead = target - x_now
                    n = np.linalg.norm(lead)
                    if n > args.max_lead:
                        target = x_now + lead * (args.max_lead / n)
                else:
                    target = get_target(arm["x_start"], data.time)

                data.qfrc_applied[ctrl.dof] = ctrl.compute(data, target)

                log = arm["log"]
                log["t"].append(data.time)
                log["pos"].append(data.site_xpos[ctrl.site_id].copy())
                log["target"].append(np.array(target))
                log["ori_err"].append(np.linalg.norm(ctrl.ori_err))

            mujoco.mj_step(model, data)
            viewer.sync()

            sleep = model.opt.timestep - (time.time() - step_start)
            if sleep > 0:
                time.sleep(sleep)


if __name__ == "__main__":
    main()

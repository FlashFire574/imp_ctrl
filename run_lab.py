#!/usr/bin/env python3
"""
Run the impedance controller on the Geodude lab model (two UR5e arms).

Usage:
    python run_lab.py --arm left
    python run_lab.py --arm right
    python run_lab.py --arm both            (default)
    python run_lab.py --arm both --axis x --length 0.15 --period 8
    python run_lab.py --arm both --static   (hold position instead of a line)

Arms that are NOT selected stay held at the "ready" pose by their own
position servos. Selected arms have their servos switched off and are driven
by the impedance controller through data.qfrc_applied.
"""

import argparse
from email import parser
import time

import numpy as np
import mujoco
import mujoco.viewer

from geodude_assets import get_geodude_path
from impedance_controller import ImpedanceController
from quest_teleop import QuestTeleop
from plotting import report

ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
SITE = "gripper_attachment_site"


def disable_servos(model, joint_names):
    """Zero the position actuators on these joints so they exert no force."""
    jids = {model.joint(j).id for j in joint_names}
    for a in range(model.nu):
        if (model.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT
                and model.actuator_trnid[a, 0] in jids):
            model.actuator_gainprm[a, :] = 0
            model.actuator_biasprm[a, :] = 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=["left", "right", "both"], default="both",
                        help="Which arm(s) to control with impedance control.")
    parser.add_argument("--static", action="store_true",
                        help="Hold the start position instead of tracing a line.")
    parser.add_argument("--axis", choices=["x", "y", "z"], default="y",
                        help="World axis the line runs along (default y).")
    parser.add_argument("--length", type=float, default=0.1,
                        help="Line length in metres (default 0.1).")
    parser.add_argument("--period", type=float, default=6.0,
                        help="Seconds for one out-and-back sweep (default 6).")
    parser.add_argument("--teleop", action="store_true",
                    help="Drive the arms from the Meta Quest controllers.")
    parser.add_argument("--scale", type=float, default=1.0,
                    help="Hand-motion to end-effector-motion scale.")
    parser.add_argument("--yaw", type=float, default=0.0,
                    help="Rotate the Quest frame about z (deg) to match the robot's forward.")
    parser.add_argument("--max-lead", type=float, default=0.10,
                    help="Max distance (m) the target may lead the end effector.")
    args = parser.parse_args()

    sides = ["left", "right"] if args.arm == "both" else [args.arm]

    model = mujoco.MjModel.from_xml_path(str(get_geodude_path()))
    data = mujoco.MjData(model)

    # Home pose first; controllers read the home orientation at construction
    mujoco.mj_resetDataKeyframe(model, data, model.key("ready").id)
    mujoco.mj_forward(model, data)
    model.body_gravcomp[:] = 0 

    arms = {}
    for side in sides:
        joints = [f"{side}_ur5e/{j}" for j in ARM_JOINTS]
        disable_servos(model, joints)
        ctrl = ImpedanceController(model, data,
                                   site_name=f"{side}_ur5e/{SITE}",
                                   joint_names=joints)
        arms[side] = {
            "ctrl": ctrl,
            "x_start": data.site_xpos[ctrl.site_id].copy(),
            "log": {"t": [], "pos": [], "target": [], "ori_err": []},
        }

    if args.teleop:
        teleop = QuestTeleop(sides, scale=args.scale, yaw_deg=args.yaw)
    else:
        teleop = None
    direction = np.zeros(3)
    direction["xyz".index(args.axis)] = 1.0

    def get_target(x_start, t):
        if args.static:
            return x_start
        # Smooth out-and-back sweep along a straight line: starts at rest at
        # x_start, reaches x_start + length*direction, returns, and repeats.
        CIRCLE = False
        if CIRCLE:
            return x_start + np.array([0.1 * np.sin(t), 0.1 * (np.cos(t) - 1), 0.0])
        else:
            s = 0.5 * (1.0 - np.cos(2.0 * np.pi * t / args.period))
            return x_start + args.length * s * direction

    with mujoco.viewer.launch_passive(model, data) as viewer:
        dumped = False
        while viewer.is_running():
            step_start = time.time()

            for side, arm in arms.items():
                ctrl = arm["ctrl"]
                x_now = data.site_xpos[ctrl.site_id].copy()

                if teleop is not None:
                    target = teleop.get_target(side, x_now)
                    # Safety: don't let the target run far ahead of the EE
                    lead = target - x_now
                    n = np.linalg.norm(lead)
                    if n > args.max_lead:
                        target = x_now + lead * (args.max_lead / n)
                else:
                    target = get_target(arm["x_start"], data.time)

                data.qfrc_applied[ctrl.dof] = ctrl.compute(data, target)

                # logging (unchanged)
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
#!/usr/bin/env python3
"""
This script coordinates a 3D simulation pipeline in MuJoCo, deploying an
impedance controller to track static or circular trajectory targets.
"""

import time

import mujoco
import mujoco.viewer
import numpy as np

from imp_ctrl.controller.impedance_controller import ImpedanceController
from imp_ctrl.plotting import report
from imp_ctrl.robot_model import build_model, set_home

CIRCLE = True


def main():
    model = build_model()
    data = mujoco.MjData(model)
    set_home(model, data)

    ctrl = ImpedanceController(model, data)
    x_start = data.site_xpos[ctrl.site_id].copy()

    def get_target(t):
        if CIRCLE:
            return x_start + np.array([0.1 * np.sin(t), 0.1 * (np.cos(t) - 1), 0.0])
        return x_start

    log = {"t": [], "pos": [], "target": [], "ori_err": []}
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            target = get_target(data.time)

            data.ctrl[:6] = ctrl.compute(data, target)
            data.ctrl[6] = 0

            log["t"].append(data.time)
            log["pos"].append(data.site_xpos[ctrl.site_id].copy())
            log["target"].append(np.array(target))
            log["ori_err"].append(np.linalg.norm(ctrl.ori_err))

            mujoco.mj_step(model, data)
            viewer.sync()

            sleep = model.opt.timestep - (time.time() - step_start)
            if sleep > 0:
                time.sleep(sleep)

    report(log, x_start)


if __name__ == "__main__":
    main()

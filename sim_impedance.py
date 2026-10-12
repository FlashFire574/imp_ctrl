#!/usr/bin/env python3
"""
main_simulation.py

This script coordinates a 3D simulation pipeline in MuJoCo, deploying an
impedance controller to track static or circular trajectory targets.

Workflow:
    1. Initializes the custom robotic model and places it in a home configuration.
    2. Instantiates an ImpedanceController to manage tracking behaviors.
    3. Launches a passive visualization viewer to render physics steps in real-time.
    4. Logs physical operational data and generates a post-simulation report.

Constants:
    CIRCLE (bool): Set True to execute a circular trajectory tracking test.
                   Set False to hold a static target configuration.
"""

import time
import numpy as np
import mujoco
import mujoco.viewer

from robot_model import build_model, set_home
from impedance_controller import ImpedanceController
from plotting import report

CIRCLE = False


def main():
    """
    Execute the core simulation loop and handle hardware control assignments.

    Launches a passive visualizer, steps the MuJoCo physics engine, passes target
    positions to the operational space controller, and logs trajectory tracking performance.
    """
    model = build_model()
    data = mujoco.MjData(model)
    set_home(model, data)

    ctrl = ImpedanceController(model, data)
    x_start = data.site_xpos[ctrl.site_id].copy()

    def get_target(t):
        """
        Compute the desired end-effector cartesian coordinates at a given timestamp.

        Parameters:
            t (float): Continuous simulation time component passed from MjData.

        Returns:
            np.ndarray: A 3-element [X, Y, Z] target position vector.
        """
        if CIRCLE:
            return x_start + np.array([0.1 * np.sin(t), 0.1 * (np.cos(t) - 1), 0.0])
        return x_start

    log = {"t": [], "pos": [], "target": [], "ori_err": []}
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            target = get_target(data.time)

            data.ctrl[:6] = ctrl.compute(data, target)
            data.ctrl[6] = 0   # gripper open

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

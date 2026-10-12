"""
Main Interactive Teleoperation Simulation Entry Point.

This script integrates the `robot_model` assembly, `ImpedanceController` law, 
and `KeyboardTeleop` state filters into a live simulation loop. It compiles the 
bimanual/single-arm asset scene, spawns dynamic physical manipulation items (cube, 
cylinder) near the home position, handles user real-time keystroke callback events, 
and drives the Mujoco passive viewer infrastructure at a deterministic rate.
"""

import time
import mujoco
import mujoco.viewer

from robot_model import build_model, set_home, place_object
from impedance_controller import ImpedanceController
from keyboard_teleop import KeyboardTeleop


def main():
    """
    Instantiate the MuJoCo scene environment and execute the synchronous control loop.
    
    Manages the live teleoperation loop by querying keyboard inputs, updating 
    reference tracking targets, evaluating the operational space impedance task 
    wrench, and maintaining passive visual viewer synchronization at real-time speeds.
    """
    model = build_model(with_objects=True)
    data = mujoco.MjData(model)
    set_home(model, data)

    ctrl = ImpedanceController(model, data)
    x_start = data.site_xpos[ctrl.site_id].copy()

    place_object(model, data, "cube",     [x_start[0] + 0.10, x_start[1], 0.0255])
    place_object(model, data, "cylinder", [x_start[0] - 0.10, x_start[1], 0.0405])
    mujoco.mj_forward(model, data)

    dt = model.opt.timestep
    teleop = KeyboardTeleop(x_start, dt)
    print(teleop.help_text())

    with mujoco.viewer.launch_passive(model, data,
                                      key_callback=teleop.key_callback) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # Process user command updates and smooth spatial trajectories
            x_ref, grip = teleop.update()
            
            # Map Cartesian virtual forces to joints and forward gripper control states
            data.ctrl[:6] = ctrl.compute(data, x_ref)
            data.ctrl[6] = grip

            # Step the simulation physics engine and update render buffers
            mujoco.mj_step(model, data)
            viewer.sync()

            # Enforce strict real-time execution constraint pacing
            sleep = dt - (time.time() - step_start)
            if sleep > 0:
                time.sleep(sleep)


if __name__ == "__main__":
    main()

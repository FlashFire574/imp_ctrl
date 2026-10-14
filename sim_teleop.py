import time
import mujoco
import mujoco.viewer

from robot_model import build_model, set_home, place_object
from impedance_controller import ImpedanceController
from keyboard_teleop import KeyboardTeleop


def main():
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

            x_ref, grip = teleop.update()
            data.ctrl[:6] = ctrl.compute(data, x_ref)
            data.ctrl[6] = grip

            mujoco.mj_step(model, data)
            viewer.sync()

            sleep = dt - (time.time() - step_start)
            if sleep > 0:
                time.sleep(sleep)


if __name__ == "__main__":
    main()
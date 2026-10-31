"""
Keyboard Teleoperation System for MuJoCo Robot Control.

This module provides real-time end-effector pose and gripper control using standard
keyboard inputs via GLFW key codes. It updates a 3D Cartesian target position
with protective safety features, including a minimum height limit (`z_min`) and a
radial reach constraint (`max_reach`).

To ensure smooth robot movement, a first-order low-pass filter gently interpolates
the reference trajectory (`x_ref`) toward the discrete step targets. Gripper open/close
commands are rate-limited to simulate physical actuation time.
"""

import numpy as np

KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP = 262, 263, 264, 265
KEY_PGUP, KEY_PGDN = 266, 267
KEY_MOVES = {
    KEY_RIGHT: (0, +1),
    KEY_LEFT: (0, -1),
    KEY_UP: (1, +1),
    KEY_DOWN: (1, -1),
    KEY_PGUP: (2, +1),
    KEY_PGDN: (2, -1),
}


class KeyboardTeleop:
    """Manages keyboard-driven interactive Cartesian teleoperation and gripper control."""

    def __init__(self, x_start, dt, step=0.01, z_min=0.12, max_reach=0.80, ref_tau=0.3, gripper_rate=200.0):
        self.dt = dt
        self.step, self.z_min, self.max_reach = step, z_min, max_reach
        self.ref_tau, self.gripper_rate = ref_tau, gripper_rate

        self.target = np.array(x_start, dtype=float)
        self.x_ref = self.target.copy()
        self.grip_cmd = 0.0
        self.grip_val = 0.0

    def key_callback(self, keycode):
        if keycode in KEY_MOVES:
            axis, sign = KEY_MOVES[keycode]
            self.target[axis] += sign * self.step
            self.target[2] = max(self.target[2], self.z_min)
            r = np.linalg.norm(self.target[:2])
            if r > self.max_reach:
                self.target[:2] *= self.max_reach / r
        elif keycode == ord("O"):
            self.grip_cmd = 0.0
        elif keycode == ord("C"):
            self.grip_cmd = 255.0

    def update(self):
        self.x_ref += (self.dt / self.ref_tau) * (self.target - self.x_ref)
        delta = self.grip_cmd - self.grip_val
        self.grip_val += np.clip(delta, -self.gripper_rate * self.dt, self.gripper_rate * self.dt)
        return self.x_ref, self.grip_val

    @staticmethod
    def help_text():
        return "Controls: arrows = move x/y, PageUp/PageDown = z, O = open, C = close"

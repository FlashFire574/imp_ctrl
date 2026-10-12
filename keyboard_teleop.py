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

# GLFW key codes for direction and height adjustments
KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP = 262, 263, 264, 265
KEY_PGUP, KEY_PGDN = 266, 267
KEY_MOVES = {
    KEY_RIGHT: (0, +1), KEY_LEFT: (0, -1),    # x-axis controls
    KEY_UP:    (1, +1), KEY_DOWN: (1, -1),    # y-axis controls
    KEY_PGUP:  (2, +1), KEY_PGDN: (2, -1),    # z-axis controls
}


class KeyboardTeleop:
    """Manages keyboard-driven interactive Cartesian teleoperation and gripper control."""

    def __init__(self, x_start, dt, step=0.01, z_min=0.12, max_reach=0.80,
                 ref_tau=0.3, gripper_rate=200.0):
        """
        Initialize the teleoperation target buffers, workspace safety limits, and filter timings.

        Parameters:
            x_start (array_like): Initial 3D Cartesian position [x, y, z] of the end-effector.
            dt (float): Simulation timestep duration in seconds.
            step (float): Incremental distance per keypress movement.
            z_min (float): Minimum allowable height boundary to prevent floor collisions.
            max_reach (float): Maximum allowed horizontal workspace radius from the robot base.
            ref_tau (float): Time constant for the exponential trajectory smoothing filter.
            gripper_rate (float): Actuation speed limit for the gripper control state per second.
        """
        self.dt = dt
        self.step, self.z_min, self.max_reach = step, z_min, max_reach
        self.ref_tau, self.gripper_rate = ref_tau, gripper_rate

        self.target = np.array(x_start, dtype=float)   # Discrete commanded goal
        self.x_ref = self.target.copy()                # Smoothed interpolation reference
        self.grip_cmd = 0.0                            # Target gripper state (0=Open, 255=Closed)
        self.grip_val = 0.0                            # Current rate-limited gripper control value

    def key_callback(self, keycode):
        """
        Process incoming keyboard events to modify target positions and gripper states.

        Pass this method as a callback directly to `mujoco.viewer.launch_passive(key_callback=...)`.

        Parameters:
            keycode (int): Integer identifier corresponding to the pressed GLFW key.
        """
        if keycode in KEY_MOVES:
            axis, sign = KEY_MOVES[keycode]
            self.target[axis] += sign * self.step
            
            # Enforce floor safety boundary
            self.target[2] = max(self.target[2], self.z_min)
            
            # Enforce workspace workspace radius envelope constraint
            r = np.linalg.norm(self.target[:2])
            if r > self.max_reach:
                self.target[:2] *= self.max_reach / r
        elif keycode == ord("O"):
            self.grip_cmd = 0.0
        elif keycode == ord("C"):
            self.grip_cmd = 255.0

    def update(self):
        """
        Advance the smoothing filters by a single timestep toward the targeted positions.

        Call this once per physics simulation loop step before running the controller.

        Returns:
            tuple:
                - np.ndarray: Low-pass filtered 3D reference position [x_ref, y_ref, z_ref].
                - float: Current intermediate gripper command tracking variable.
        """
        # Exponentially smooth spatial coordinates toward discrete key-step targets
        self.x_ref += (self.dt / self.ref_tau) * (self.target - self.x_ref)
        
        # Apply rate limits to the gripper opening and closing velocities
        delta = self.grip_cmd - self.grip_val
        self.grip_val += np.clip(delta, -self.gripper_rate * self.dt,
                                 self.gripper_rate * self.dt)
        return self.x_ref, self.grip_val

    @staticmethod
    def help_text():
        """
        Provide a UI-friendly command overview string.

        Returns:
            str: Explanatory text summarizing key assignment behaviors.
        """
        return "Controls: arrows = move x/y, PageUp/PageDown = z, O = open, C = close"

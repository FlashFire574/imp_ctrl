"""
Cartesian Operational Space Impedance Controller for MuJoCo.

This module implements an impedance control architecture that models the robot's 
end-effector as a virtual multi-axis spring-damper system. It computes the 
necessary joint torques to track a target Cartesian position while maintaining a 
fixed target orientation. 

The controller applies a PD control law in task space (wrench) and projects it 
to joint space via a Jacobian Transpose mapping, automatically 
injecting dynamic gravity and Coriolis compensation terms (`qfrc_bias`).
"""

import numpy as np
import mujoco
from robot_model import ARM_JOINTS

# Default stiffness (KP) and damping (KD) matrices for operational space control
KP_DEFAULT = np.diag([500, 500, 500, 50, 50, 50])   # N/m, Nm/rad
KD_DEFAULT = np.diag([60, 60, 60, 8, 8, 8])


class ImpedanceController:
    """A Cartesian Operational Space Impedance Controller for a MuJoCo robotic arm."""

    def __init__(self, model, data, site_name="attachment_site",
                 kp=KP_DEFAULT, kd=KD_DEFAULT):
        """
        Initialize the impedance controller buffers, configurations, and target orientation.

        Parameters:
            model (mujoco.MjModel): MuJoCo model descriptor.
            data (mujoco.MjData): MuJoCo dynamic data instance.
            site_name (str): Name of the controlled operational space site.
            kp (np.ndarray): 6x6 diagonal task space stiffness matrix.
            kd (np.ndarray): 6x6 diagonal task space damping matrix.
        """
        self.model = model
        self.kp, self.kd = kp, kd
        self.site_id = model.site(site_name).id
        
        # Map arm joint IDs to their respective degrees of freedom (DoF) indices
        self.dof = np.array([model.jnt_dofadr[model.joint(j).id] for j in ARM_JOINTS])
        
        # Pre-allocate mathematical and kinematic buffers
        self.jacp = np.zeros((3, model.nv))
        self.jacr = np.zeros((3, model.nv))
        self.quat = np.zeros(4)
        self.quat_neg = np.zeros(4)
        self.quat_err = np.zeros(4)
        self.ori_err = np.zeros(3)

        # Store the home pose orientation as the constant desired orientation
        self.quat_des = np.zeros(4)
        mujoco.mju_mat2Quat(self.quat_des, data.site_xmat[self.site_id])

    def compute(self, data, x_des):
        """
        Compute joint space torques using a Cartesian PD impedance control law.

        Parameters:
            data (mujoco.MjData): Current MuJoCo simulation data.
            x_des (np.ndarray): Desired 3D Cartesian position target [x, y, z].

        Returns:
            np.ndarray: Evaluated joint torque array mapped via the Jacobian transpose.
        """
        # Fetch and stack translational and rotational Jacobians for the arm joints
        mujoco.mj_jacSite(self.model, data, self.jacp, self.jacr, self.site_id)
        J = np.vstack([self.jacp[:, self.dof], self.jacr[:, self.dof]])

        # Compute translation error
        pos_err = x_des - data.site_xpos[self.site_id]

        # Calculate orientation geodesic error using quaternion multiplication (q_des * q_curr^-1)
        mujoco.mju_mat2Quat(self.quat, data.site_xmat[self.site_id])
        mujoco.mju_negQuat(self.quat_neg, self.quat)
        mujoco.mju_mulQuat(self.quat_err, self.quat_des, self.quat_neg)
        mujoco.mju_quat2Vel(self.ori_err, self.quat_err, 1.0)

        # Combine errors and project joint velocities into operational space velocity
        err = np.concatenate([pos_err, self.ori_err])
        xdot = J @ data.qvel[self.dof]

        # Evaluate the Cartesian virtual spring-damper control law
        wrench = self.kp @ err - self.kd @ xdot
        
        # Map the task wrench to joint torques and add gravity/Coriolis compensation
        return J.T @ wrench + data.qfrc_bias[self.dof]

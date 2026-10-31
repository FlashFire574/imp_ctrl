"""
Robot Model Compilation and Scene Initialization for MuJoCo Simulation.

This module provides utility functions to dynamically compile a Universal Robots
UR5e arm combined with a Robotiq 2f85 gripper using MuJoCo's MjSpec API. It includes
helpers to convert the default arm position actuators into pure torque motors, spawn
manipulation objects (cubes, cylinders) with custom friction and elliptic contact cones,
and reset the system joints or free bodies to home/target poses.
"""

import os

import numpy as np
import mujoco

# Target arm joint configurations and home posture coordinates
ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
Q_HOME = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])


def add_object(spec, name, geom_type, size, rgba, mass=0.1):
    """
    Append a dynamically interactable free-body object to the world model spec.

    Configures high-friction surfaces and a 4-dimensional contact dimension
    for stable grasping manipulations.
    """
    body = spec.worldbody.add_body(name=name, pos=[0, 0, 1.0])
    body.add_freejoint()
    body.add_geom(
        name=f"{name}_geom",
        type=geom_type,
        size=size,
        rgba=rgba,
        mass=mass,
        friction=[1.0, 0.005, 0.0001],
        condim=4,
    )


def build_model(with_objects=False):
    """
    Assemble the complete UR5e and Robotiq gripper model from separate XML descriptors.

    Sets up global environmental properties like floor planes, lighting, elliptic
    friction cones, and modifies the arm for torque control before compilation.
    """
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ur5e_path = os.path.join(base, "assets", "universal_robots_ur5e", "ur5e.xml")
    robotiq_path = os.path.join(base, "assets", "robotiq_2f85", "2f85.xml")

    arm = mujoco.MjSpec.from_file(ur5e_path)
    gripper = mujoco.MjSpec.from_file(robotiq_path)
    arm.attach(gripper, site=arm.site("attachment_site"), prefix="gripper_")

    arm.worldbody.add_light(
        pos=[0, -0.2, 1],
        dir=[0, 0.2, -0.8],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
    )
    arm.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[0, 0, 0.05],
        friction=[1.0, 0.005, 0.0001],
    )

    if with_objects:
        add_object(
            arm,
            "cube",
            mujoco.mjtGeom.mjGEOM_BOX,
            [0.025, 0.025, 0.025],
            [0.9, 0.2, 0.2, 1],
        )
        add_object(
            arm,
            "cylinder",
            mujoco.mjtGeom.mjGEOM_CYLINDER,
            [0.0225, 0.04, 0],
            [0.2, 0.4, 0.9, 1],
        )

    arm.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    arm.option.impratio = 10

    model = arm.compile()
    make_arm_torque_controlled(model)
    return model


def make_arm_torque_controlled(model, n=6):
    """
    Convert the first 'n' position-driven actuators into pure joint torque motors.

    Disables internal biases, overrides gain profiles, and enforces physical
    Newton-meter limits tailored for UR5e joint motors.
    """
    model.actuator_gainprm[:n, 0] = 1.0
    model.actuator_biasprm[:n, :3] = 0.0
    model.actuator_ctrllimited[:n] = 1
    limits = np.array([150, 150, 150, 28, 28, 28], dtype=float)
    model.actuator_ctrlrange[:n, 0] = -limits
    model.actuator_ctrlrange[:n, 1] = limits


def set_home(model, data):
    """
    Reset the primary robotic arm joints to their designated Q_HOME position.

    Executes a forward kinematic update to synchronize site transformations.
    """
    qadr = [model.jnt_qposadr[model.joint(j).id] for j in ARM_JOINTS]
    data.qpos[qadr] = Q_HOME
    mujoco.mj_forward(model, data)


def place_object(model, data, body_name, pos):
    """
    Teleport a specified free body to a new 3D spatial position in the scene.

    Resets the object's orientation profile back to a default identity quaternion.
    """
    bid = model.body(body_name).id
    qadr = model.jnt_qposadr[model.body_jntadr[bid]]
    data.qpos[qadr:qadr + 3] = pos
    data.qpos[qadr + 3:qadr + 7] = [1, 0, 0, 0]

# imp_ctrl
(Work in progress the documentation or code may still break)
MuJoCo-based impedance control experiments for a UR5e robot with a Robotiq 2F-85 gripper.

This repository contains the control logic, robot model assembly, teleoperation helpers, and MuJoCo simulation entry points for Cartesian impedance experiments. The project is organized as a Python package under `imp_ctrl/` and includes a lab runner for dual-arm work in the Geodude scene.

## Project layout

```text
imp_ctrl/
├── __init__.py
├── robot_model.py
├── plotting.py
├── controller/
│   ├── __init__.py
│   └── impedance_controller.py
├── teleop/
│   ├── __init__.py
│   ├── keyboard_teleop.py
│   └── quest_teleop.py
├── sim/
│   ├── __init__.py
│   ├── sim_impedance.py
│   └── sim_teleop.py
├── apps/
│   ├── __init__.py
│   ├── run_lab.py
│   └── left_arm.py
├── tools/
│   ├── __init__.py
│   └── inspect_assests.py
└── ...
```

## What this project does

- Builds a UR5e + Robotiq gripper model from MuJoCo XML assets in `assets/`
- Applies a Cartesian impedance controller in task space
- Generates end-effector targets from keyboard or Meta Quest input
- Runs single-arm and dual-arm simulation/lab demos
- Uses separate local packages for `geodude_assets` and `oculus_reader`

## Key dependencies

This repo expects the following packages to be importable in your environment:

- Python 3.10+
- NumPy
- MuJoCo
- `geodude_assets`
- `oculus_reader`

The `geodude_assets` and `oculus_reader` packages are not inside this repository. They are separate local packages that should be installed in editable mode.

## Local editable install setup

From a parent directory that contains both packages:

```bash
pip install -e ./geodude_assets
pip install -e ./oculus_reader
```

If they live elsewhere, use explicit paths:

```bash
pip install -e /path/to/geodude_assets
pip install -e /path/to/oculus_reader
```

This matches imports such as:

```python
from geodude_assets import get_geodude_path
from oculus_reader import OculusReader
```

## Installing this repo for development

From the root of `imp_ctrl`:

```bash
python -m venv .venv
source .venv/bin/activate
pip install numpy mujoco
pip install -e ../geodude_assets
pip install -e ../oculus_reader
```

Then run the package entry points with Python module execution:

```bash
python -m imp_ctrl.sim.sim_impedance
python -m imp_ctrl.sim.sim_teleop
python -m imp_ctrl.apps.run_lab --arm both
```

## Quick start

### Basic impedance simulation

```bash
python -m imp_ctrl.sim.sim_impedance
```

### Keyboard teleop demo

```bash
python -m imp_ctrl.sim.sim_teleop
```

### Dual-arm lab demo

```bash
python -m imp_ctrl.apps.run_lab --arm both
```

### Single-arm lab demo

```bash
python -m imp_ctrl.apps.left_arm
```

### Quest teleop lab demo

```bash
python -m imp_ctrl.apps.run_lab --arm both --teleop --scale 1.0 --yaw 0.0
```

## Core modules

- `imp_ctrl.robot_model` — builds the UR5e + gripper model and home pose helpers
- `imp_ctrl.controller.impedance_controller` — operational-space impedance controller
- `imp_ctrl.teleop.keyboard_teleop` — keyboard target generation and smoothing
- `imp_ctrl.teleop.quest_teleop` — Meta Quest target generation
- `imp_ctrl.sim.sim_impedance` — single-arm impedance demo
- `imp_ctrl.sim.sim_teleop` — keyboard teleop simulation
- `imp_ctrl.apps.run_lab` — dual-arm Geodude lab runner
- `imp_ctrl.tools.inspect_assests` — dump model joints/sites/actuators/keyframes

## Asset notes

The repository includes MuJoCo asset packages under:

- `assets/universal_robots_ur5e/`
- `assets/robotiq_2f85/`

Those directories include their own model documentation and licenses.

## Troubleshooting

If imports fail, check that both local editable packages are installed and visible in the active Python environment:

```bash
python -c "import geodude_assets, oculus_reader; print('ok')"
```

If you need to inspect the lab model names or available sites, run:

```bash
python -m imp_ctrl.tools.inspect_assests
```

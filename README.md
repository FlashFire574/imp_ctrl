# imp_ctrl

MuJoCo-based impedance control experiments for a UR5e robot with a Robotiq 2F-85 gripper. This repository contains the controller logic, single-arm simulation utilities, and a dual-arm lab runner for testing Cartesian force/position control in simulation and with Meta Quest teleoperation.

## What this project does

- Builds a robot model from separate MuJoCo asset packages under `assets/`
- Implements a Cartesian impedance controller in `impedance_controller.py`
- Provides keyboard and Quest-based teleoperation for commanding the end-effector target
- Runs simplified simulation demos and a larger dual-arm lab scene
- Uses external local packages for scene assets and the Oculus reader

## Repository structure

- `robot_model.py` — assembles the UR5e + gripper model and prepares torque-controlled joints
- `impedance_controller.py` — operational-space impedance controller
- `sim_impedance.py` — basic impedance tracking demo in MuJoCo
- `sim_teleop.py` — keyboard teleop simulation
- `run_lab.py` — dual-arm lab runner with optional Meta Quest control
- `keyboard_teleop.py` — keyboard-based target generation and smoothing
- `quest_teleop.py` — Quest headset controller integration via `OculusReader`
- `plotting.py` — plotting helpers for controller logs
- `left_arm.py` — single-arm lab example
- `inspect_assests.py` — introspection utility for model names, joints, sites, and keyframes
- `assets/` — robot model files for the UR5e and Robotiq gripper
- `MUJOCO_LOG.TXT` — MuJoCo-generated log output

## Key dependencies

This project expects the following Python packages to be available:

- Python 3.10+
- NumPy
- MuJoCo
- `geodude_assets`
- `oculus_reader`

Important: `geodude_assets` and `oculus_reader` are not stored inside this repository. They are separate local packages that should be installed in editable mode from their own directories.

## Local editable install setup

From a parent directory that contains both of these project folders:

```bash
pip install -e ./geodude_assets
pip install -e ./oculus_reader
```

If the folders are located elsewhere, replace the paths accordingly:

```bash
pip install -e /path/to/geodude_assets
pip install -e /path/to/oculus_reader
```

This is the expected pattern for this repo because imports such as:

```python
from geodude_assets import get_geodude_path
from oculus_reader import OculusReader
```

depend on those packages being importable in your environment.

## Installation for this repo

From the root of `imp_ctrl`:

```bash
python -m venv .venv
source .venv/bin/activate
pip install numpy mujoco
pip install -e ../geodude_assets
pip install -e ../oculus_reader
```

Then run:

```bash
python sim_impedance.py
```

## Quick start

### Basic MuJoCo impedance demo

```bash
python sim_impedance.py
```

### Keyboard teleoperation demo

```bash
python sim_teleop.py
```

### Dual-arm lab demo

```bash
python run_lab.py --arm both
```

### Teleop with Meta Quest

```bash
python run_lab.py --arm both --teleop --scale 1.0 --yaw 0.0
```

### Single arm setup

```bash
python run_lab.py --arm left
```

## Typical workflow

1. Install the local editable packages for `geodude_assets` and `oculus_reader`
2. Run a simulation or lab demo from this repo
3. If using the Quest path, keep the Oculus reader running and use `QuestTeleop`
4. For non-Quest experiments, use the keyboard teleoperation or built-in path planning helpers

## Notes about the model

The robot model is assembled from MuJoCo XML files under `assets/universal_robots_ur5e/` and `assets/robotiq_2f85/`. The `build_model()` function attaches the gripper to the UR5e, modifies the arm into torque-controlled actuators, and sets up friction/scene properties suitable for manipulation experiments.

## Debugging / inspection

Useful inspection commands:

```bash
python inspect_assests.py
```

This prints model joints, sites, actuators, and keyframes so that you can confirm names used by the controller and lab runner.

## License and asset provenance

The asset directories include their own licensing and documentation.

- `assets/universal_robots_ur5e/` — UR5e MJCF model with included license and asset notes
- `assets/robotiq_2f85/` — Robotiq 2F-85 MJCF model with included license and asset notes

This repository itself is a control and simulation layer around those asset packages and is intended to be used together with the separate local editable installs listed above.

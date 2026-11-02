# imp_ctrl

Cartesian operational-space impedance control for a UR5e with a Robotiq 2F-85 gripper in MuJoCo. The end-effector is treated as a virtual spring-damper: a task-space PD wrench is mapped to joint torques through the Jacobian transpose, with gravity and Coriolis compensation from MuJoCo (`qfrc_bias`).

There are two simulation setups:

- **Single arm** (models in this repo): built at runtime from `assets/` and driven through `data.ctrl`.
- **Geodude lab** (two UR5e arms): loaded via `geodude_assets`. Selected arms have their position servos disabled and are driven through `data.qfrc_applied`. Unselected arms stay at the `ready` keyframe.

## Layout

```
imp_ctrl/
  assets/                 UR5e and Robotiq MuJoCo models
  imp_ctrl/               library
    controller.py         impedance controller
    robot.py              single-arm scene compile / home pose
    plotting.py           tracking report after sim_impedance
    teleop/
      keyboard.py         arrow-key Cartesian teleop
      quest.py            Meta Quest clutch teleop
  scripts/                things you actually run
```

## Setup

Run everything from the **repo root**. Scripts add that directory to `sys.path` automatically.

```bash
cd /path/to/imp_ctrl
```

Python packages:

- `numpy`, `mujoco`, `matplotlib` — all scripts
- `geodude_assets` — lab scripts (`run_lab.py`, `inspect_model.py`)
- `oculus_reader` — only if you pass `--teleop` on the lab runner

```bash
pip install numpy mujoco matplotlib
```

Install `geodude_assets` and `oculus_reader` the same way you already do for the lab stack.

## How to run

Always from the repo root:

```bash
python scripts/<script>.py
```

### Single-arm circle / hold

Tracks a 10 cm circle in XY (default) or holds the home pose. Close the MuJoCo window to stop; a tracking plot then opens.

```bash
python scripts/sim_impedance.py
```

To hold still instead of circling, set `CIRCLE = False` at the top of `scripts/sim_impedance.py`.

### Single-arm keyboard teleop

Spawns a cube and a cylinder in front of the gripper.

```bash
python scripts/sim_teleop.py
```

| Key | Action |
|-----|--------|
| Arrow keys | Move X / Y |
| Page Up / Page Down | Move Z |
| `O` | Open gripper |
| `C` | Close gripper |

### Dual-arm lab (Geodude)

Default: both arms sweep a straight line along world Y (0.1 m, 6 s out-and-back).

```bash
python scripts/run_lab.py
python scripts/run_lab.py --arm left
python scripts/run_lab.py --arm right
python scripts/run_lab.py --arm both --static
python scripts/run_lab.py --arm both --axis x --length 0.15 --period 8
```

| Flag | Default | Meaning |
|------|---------|---------|
| `--arm {left,right,both}` | `both` | Which arm(s) the impedance controller drives |
| `--static` | off | Hold the start pose instead of tracing a line |
| `--axis {x,y,z}` | `y` | World axis of the line |
| `--length` | `0.1` | Line length (m) |
| `--period` | `6` | Seconds for one out-and-back sweep |
| `--teleop` | off | Drive from Meta Quest controllers |
| `--scale` | `1.0` | Hand motion → end-effector motion scale |
| `--yaw` | `0` | Rotate the Quest frame about Z (deg) to match robot forward |
| `--max-lead` | `0.10` | Max distance (m) the target may lead the end-effector |

Quest teleop (grip clutch, mirrored hands; trigger closes the gripper):

```bash
python scripts/run_lab.py --arm both --teleop
python scripts/run_lab.py --arm both --teleop --scale 0.8 --yaw 90
```

### Inspect the lab model

Prints Geodude joint, site, actuator, and keyframe names.

```bash
python scripts/inspect_model.py
```

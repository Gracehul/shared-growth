# Shared Growth

Shared Growth is a tangible human-robot co-drawing project. A participant
draws on a shared physical surface and a myCobot 280 responds with a generated
branch trajectory. The research focus is how robot response timing affects
coordination, adaptation, and the experience of responsiveness, predictability,
control, and collaboration.

The repository currently contains the **Robot Drawing Base (Milestone 1)**:
safe Cartesian control, offline simulation, trajectory logging, and initial
shape-drawing scripts. Camera tracking, the human-robot interaction loop, and
experimental timing conditions are planned but are not implemented yet.

## Project ownership and contributions

- **Gracehul** — Project Lead; HRI Design & Research
- **Tom4sCruz** — Initial Robotics Contributor

See [CONTRIBUTORS.md](CONTRIBUTORS.md) for role scope and
[CONTRIBUTING.md](CONTRIBUTING.md) for the collaboration workflow.

## Project plan

The three-week development strategy uses progressive outcomes:

- **Case A — Minimal / Timing Probe:** baseline, fixed-delay, and jitter timing.
- **Case B — Medium / Coordination-aware Adaptation:** adapt a robot property
  using movement tempo, pauses, variability, or coordination.
- **Case C — Ambitious / Physiology-informed Adaptation:** optionally add
  physiological signals after the movement-based pipeline is stable and the
  required ethics and data handling are in place.

Case A is the committed outcome. Cases B and C are stretch levels, not parallel
requirements. Details and gates are documented in [docs/ROADMAP.md](docs/ROADMAP.md).

## Documentation

- [Architecture](docs/ARCHITECTURE.md)
- [Development and Jetson setup](docs/SETUP.md)
- [Setup record template](docs/SETUP_RECORD.md)
- [Safety protocol](docs/SAFETY.md)
- [Experiment protocol](docs/EXPERIMENT_PROTOCOL.md)
- [Data dictionary](docs/DATA_DICTIONARY.md)
- [Milestone roadmap](docs/ROADMAP.md)

> **Safety:** Never run a new trajectory on hardware before reviewing the
> [safety protocol](docs/SAFETY.md) and completing an offline and pen-up dry run.

## Robot Drawing Base

Milestone 1 of the "Shared Growth" project (see `Objectives.md`): get the
myCobot 280 JN reliably drawing simple shapes on paper. This is separate
from the thesis project.

**Current state:** `drawing_robot/` is a trimmed **clone** of the thesis
`armik` library's `Arm`/`ArmConnection` (single-joint mode and
jerk-injection removed) -- vendored here as an independent copy, not a
live dependency on `armik`. A from-scratch reimplementation kept hitting
subtle divergences from `armik`'s proven behavior even after its
kinematics were checked bit-for-bit equivalent, so this pivoted to
porting the actual working code instead.

The Objectives.md-driven drawing layer built earlier (`Robot`,
`shapes.py`'s point/line/curve/branch primitives, the `draw_*` scripts,
and the `ORIGIN_X`/`DRAWING_PLANE`/`PEN_RX/RY/RZ` drawing-plane
calibration) is parked under `future/`, not deleted, pending
re-integration on top of this new, more faithful base.

## Install

For local development and mock execution (no robot dependency required):

```
pip install -e .
```

On the robot/Jetson, install the hardware integration explicitly:

```
pip install -e ".[hardware]"
```

The setup helpers create an isolated environment and run a hardware-free
preflight:

```
# Windows PowerShell
.\scripts\setup.ps1

# Linux / Jetson
bash scripts/setup.sh --hardware
```

Full instructions and the split between mock and hardware dependencies are in
[docs/SETUP.md](docs/SETUP.md). `scripts/` and `future/` are not part of the
installed package; run them from a checkout of this repository.

## Hardware setup

- myCobot 280 JN, adaptive gripper holding a pen
- Serial port: `drawing_robot.config.DEFAULT_PORT` (`/dev/ttyTHS1`) by
  default, or `/tmp/ttyMyCobot` (socat/udev bridge) via `--port`
- Baud rate: `1000000` by default (`drawing_robot.config.DEFAULT_BAUDRATE`)

## Using `drawing_robot`

```python
from drawing_robot import Arm

with Arm(port="/dev/ttyTHS1") as arm:
    if not arm.conn.is_power_on():
        arm.conn.power_on()

    print(arm.get_coords())                      # cm and degrees
    arm.send_coords(x=20, y=-6, z=15, speed=4)   # 1 on success, 0 on failure
    arm.send_coords(z=12)                         # z only, rest held steady
```

Offline, with no hardware: `Arm(mock=True)`.

## Bounded drawing primitives

The active drawing layer uses robot-frame centimetres and requires explicit
workspace bounds. Placeholder calibration values are never accepted implicitly:

```python
from drawing_robot import Arm, DrawingController, DrawingWorkspace

workspace = DrawingWorkspace(
    x_min_cm=15.0,
    x_max_cm=20.0,
    y_min_cm=-9.0,
    y_max_cm=-4.0,
    drawing_z_cm=3.0,  # replace with measured values before hardware use
    safe_z_cm=10.0,
)

with Arm(mock=True) as arm:
    drawing = DrawingController(arm, workspace, speed_cm_s=1.0)
    drawing.draw_line((16.0, -6.0), (18.0, -6.0))
```

`draw_line`, `draw_curve`, and `draw_branch` validate every input point before
the first command. A failed robot command triggers a stop and requires explicit
operator recovery; the controller does not guess a recovery trajectory.

## Running the scripts

```
python3 scripts/example.py --mock                 # no hardware needed
python3 scripts/example.py --port /dev/ttyTHS1
python3 scripts/example.py --port /tmp/ttyMyCobot
```

`scripts/example.py` (ported from `armik/scripts/example.py`) is a
template, not a library import -- copy it for new scripts. It walks
through homing, full/partial-pose moves, a reachability check via
`plan_coords()`, the raw-pymycobot escape hatch (`arm.conn.raw`/
`arm.conn.lock`), and trajectory logging (`arm.last_execution.to_csv`).
`scripts/draw_square.py`/`draw_circle.py` trace a square/circle from
fixed corner/center+radius constants.

## Development checks (no hardware needed)

```
python3 -m drawing_robot.preflight
python3 -m pytest -q
python3 -m py_compile drawing_robot/*.py scripts/*.py
python3 -c "from drawing_robot import Arm, config, ik, kinematics"
python3 scripts/example.py --mock
```

## `future/`

Everything from the Objectives.md layer that was built on top of the
earlier from-scratch `drawing_robot`, parked here since it imports the
old `drawing_robot.robot`/`drawing_robot.shapes` modules that no longer
exist:
- `robot.py` -- the `Robot` wrapper (`home`/`get_pose`/`grab_pen`/
  `release_pen`/`pen_up`/`pen_down`/`move_to`)
- `shapes.py` -- `draw_point`/`draw_line`/`draw_curve`/`draw_branch` and
  the `branch_points`/`circle_points` geometry helpers
- `scripts/` -- `pen_setup.py`, `draw_point.py`, `draw_line.py`,
  `draw_curve.py`, `draw_branch.py`, `draw_square.py`, `draw_circle.py`,
  `repeatability_check.py`, `square_drawing.py`, `calibrate.py`

Re-integrating this on top of the new `drawing_robot` clone (once it's
confirmed working on hardware) is the next step after this one.

## Known limitations

- `drawing_robot/config.py`'s `TOOL_OFFSET_MM`/`TOOL_RPY_DEG`/
  `JOINT_LIMITS_DEG` etc. are `armik`'s original values for the same
  gripper+pen mount -- parity with what's proven working, not yet
  independently re-verified under this clone.
- No camera tracking, growth logic, or adaptive behavior -- out of scope
  for M1 per `Objectives.md`.

## License

No license has been selected yet. Until one is added, the source remains
copyrighted and no reuse permission is granted beyond GitHub's standard terms.

## For AI coding agents

`CLAUDE.md` in this repo serves the same purpose as the `AGENTS.md`
convention used by other terminal-based AI coding agents (Codex CLI,
etc.) -- project-specific guidance for an AI agent working in this
codebase. Read it before making changes here.

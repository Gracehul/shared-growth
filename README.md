# Bao_Drawings

Milestone 1 of the "Shared Growth" project (see `Objectives.md`): get the
myCobot 280 JN reliably drawing simple shapes on paper. This is separate
from the thesis project.

**Current state:** `bao_arm/` is a trimmed **clone** of the thesis
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

## Hardware setup

- myCobot 280 JN, adaptive gripper holding a pen
- Serial port: `bao_arm.config.DEFAULT_PORT` (`/dev/ttyTHS1`) by default,
  or `/tmp/ttyMyCobot` (socat/udev bridge) via `--port`
- Baud rate: `1000000` by default (`bao_arm.config.DEFAULT_BAUDRATE`)

## Using `bao_arm`

```python
from bao_arm import Arm

with Arm(port="/dev/ttyTHS1") as arm:
    if not arm.conn.is_power_on():
        arm.conn.power_on()

    print(arm.get_coords())                      # cm and degrees
    arm.send_coords(x=20, y=-6, z=15, speed=4)   # 1 on success, 0 on failure
    arm.send_coords(z=12)                         # z only, rest held steady
```

Offline, with no hardware: `Arm(mock=True)`.

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

## Development checks (no hardware needed)

```
python3 -m py_compile bao_arm/*.py scripts/*.py
python3 -c "from bao_arm import Arm, config, ik, kinematics"
python3 scripts/example.py --mock
```

## `future/`

Everything from the Objectives.md layer that was built on top of the
earlier from-scratch `bao_arm`, parked here since it imports the old
`bao_arm.robot`/`bao_arm.shapes` modules that no longer exist:
- `robot.py` -- the `Robot` wrapper (`home`/`get_pose`/`grab_pen`/
  `release_pen`/`pen_up`/`pen_down`/`move_to`)
- `shapes.py` -- `draw_point`/`draw_line`/`draw_curve`/`draw_branch` and
  the `branch_points`/`circle_points` geometry helpers
- `scripts/` -- `pen_setup.py`, `draw_point.py`, `draw_line.py`,
  `draw_curve.py`, `draw_branch.py`, `draw_square.py`, `draw_circle.py`,
  `repeatability_check.py`, `square_drawing.py`, `calibrate.py`

Re-integrating this on top of the new `bao_arm` clone (once it's
confirmed working on hardware) is the next step after this one.

## Known limitations

- `bao_arm/config.py`'s `TOOL_OFFSET_MM`/`TOOL_RPY_DEG`/`JOINT_LIMITS_DEG`
  etc. are `armik`'s original values for the same gripper+pen mount --
  parity with what's proven working, not yet independently re-verified
  under this clone.
- No camera tracking, growth logic, or adaptive behavior -- out of scope
  for M1 per `Objectives.md`.

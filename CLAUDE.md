# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Milestone 1 of the "Shared Growth" project (see `Objectives.md` for the full spec): drive a
myCobot 280 JN to reliably draw simple shapes on paper via `drawing_robot`. No camera tracking,
human-robot interaction, or growth-algorithm logic yet — out of scope for this milestone.

## Environment

- `pyproject.toml` defines the `drawing_robot` package (`pip install -e .`); `scripts/` and
  `future/` are not part of the installed package. This dev environment's venv at
  `/home/tomas/.venv-py/general/bin/python3` already has `pymycobot`, `numpy`, and
  `drawing_robot` (editable) installed.
- No hardware is attached in this dev environment. Verify changes with `--mock` (every script
  accepts it); `--port` targets real hardware (`/dev/ttyTHS1` for the Jetson's onboard serial,
  or `/tmp/ttyMyCobot` for a socat/udev USB bridge).

## Commands

```
python3 -m py_compile drawing_robot/*.py scripts/*.py                # syntax check
python3 -c "from drawing_robot import Arm, config, ik, kinematics"   # import check
python3 scripts/example.py --mock                              # full pipeline, no hardware
python3 scripts/<name>.py --mock                                # same, for any script
```

There is no test framework, linter, or formatter configured. The compile/import/mock-run
checks above are the verification loop — always run the relevant script with `--mock` after
touching `drawing_robot/` before considering a change done.

## Architecture

### `drawing_robot/` is a vendored, trimmed clone of the thesis `armik` library — not a dependency

`armik` (at `/home/tomas/University/Thesis/MyCobot_Pkg/Thesis_python_MyCobot_pkg/thesis_armik_pkg/armik/`)
is a separate, already-hardware-validated library from an unrelated thesis project. `drawing_robot`
copies its `Arm`/`ArmConnection`/kinematics/IK code directly, with single-joint execution and
jerk-injection (deliberate jitter simulation) removed — those were the only two features cut.
This is a one-time vendored copy: `armik` is never imported at runtime and must never be
modified. A from-scratch reimplementation was tried first and kept silently diverging from
`armik`'s real behavior even after its kinematics were checked bit-for-bit equivalent — the
actual bugs were in low-level execution details (serial `fresh_mode`, `get_angles()` retry
logic, deg/s-to-firmware-speed conversion) that a rewrite-from-spec kept missing. **When
extending `drawing_robot`, port the corresponding `armik` source as a reference rather than
reimplementing from scratch**, and never reintroduce single-joint mode or jerk-injection.

### Kinematics are computed client-side, never via firmware

`pymycobot`'s own `send_coords()`/`get_coords()`/`solve_inv_kinematics()` ask the robot's
firmware to do FK/IK over the serial link, which was found unreliable in testing. `drawing_robot`
computes forward kinematics itself (`kinematics.py`, a DH-chain transform from
`config.DH_TABLE`) and inverse kinematics itself (`ik.py`, damped least squares /
Levenberg-Marquardt over a finite-difference task Jacobian, seeded from the arm's live joint
angles to stay on the same IK branch). Only `pymycobot`'s `send_angles()`/`send_angle()`
(joint-space) are ever called to actually move the arm. `config.DH_TABLE` and
`config.JOINT_LIMITS_DEG` are physical facts about this specific myCobot 280, not tunable
parameters.

### Plan-then-execute, with partial pose constraints

`Arm.send_coords()`/`send_path()` fully plan a motion (IK-solve every waypoint, check joint
limits and per-joint speed ceilings) before sending a single byte — see the `Plan`/`Execution`
dataclasses in `arm.py`. On failure the arm does not move at all; inspect `arm.last_error` /
`arm.last_plan` / `arm.last_execution`. `send_coords(x=.., z=..)` lets you constrain only some
of `x/y/z/rx/ry/rz`; unconstrained ones are soft-anchored to their current value
(`config.FREE_ANCHOR_WEIGHT`), not left to drift freely — this partial-constraint behavior is
the main reason `drawing_robot` exists instead of calling `pymycobot` directly.

### Unit boundary: millimetres/degrees internally, centimetres/degrees at the public API

Everything inside `drawing_robot` (DH table, kinematics, IK) works in mm/deg. The mm↔cm conversion
happens in exactly two functions in `arm.py` (`_cm_to_mm`/`_mm_to_cm`) — `Arm.send_coords()`,
`get_coords()`, etc. take/return **centimetres**. This project has previously had a silent
10x bug from this exact boundary being crossed carelessly; the two new `DRAWING_PLANE_Z`/
`SAFE_Z` constants at the bottom of `config.py` are deliberately in cm (unlike every other
constant in that file, which is mm) to match the API scripts actually call, and are commented
as such.

### `future/` — parked code, currently broken, kept for reference

`future/robot.py`, `future/shapes.py`, and everything in `future/scripts/` are the
Objectives.md-driven drawing layer (`Robot.home/get_pose/grab_pen/release_pen/pen_up/pen_down/move_to`,
`shapes.py`'s point/line/curve/branch primitives) built on top of an earlier version of
`drawing_robot`. They import `drawing_robot.robot`/`drawing_robot.shapes`, which no longer exist after the pivot
to the `armik` clone — don't try to run them as-is. They're kept to be re-integrated on top of
the new `Arm`-based core once it's confirmed working on hardware; that re-integration is the
next planned step, not yet done.

### `scripts/` vs `future/scripts/`

`scripts/` holds current, working scripts against the new `drawing_robot.Arm` API
(`example.py` is the template — "copy this, don't import it" — plus `draw_square.py`/
`draw_circle.py`). `future/scripts/` holds the old scripts against the parked `Robot` API and
will not run until that layer is re-integrated.

### Calibration constants in `drawing_robot/config.py`

Most of the file (DH table, joint limits, `TOOL_OFFSET_MM`/`TOOL_RPY_DEG`, IK tuning) is
`armik`'s original values for the same physical gripper+pen mount — a known-working starting
point carried over as-is, not yet independently re-verified under this clone. Re-measuring
any of it requires real hardware.

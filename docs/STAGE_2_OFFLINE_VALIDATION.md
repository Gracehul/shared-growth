# Stage 2 — Offline Motion Validation

Stage 2 answers one bounded question:

> Is this sampled trajectory valid according to the current robot model and
> project constraints, and can it proceed to simulated execution?

A `VALID` result is not evidence that a trajectory is safe on physical
hardware. The tool transform and drawing workspace are still provisional, the
drawing plane is not calibrated, and the approximately 11 mm discrepancy
between Python FK and the firmware flange pose is deliberately not corrected
here.

## Boundary

```text
CartesianTrajectory
        |
SequentialIKSolver     previous joint result seeds the next IK sample
        |
JointTrajectory
        |
TrajectoryValidator    timestamps, limits, steps, dynamics, FK,
        |               workspace, Jacobian diagnostic
ValidationResult
        |
SimRobot (Stage 3, not part of Stage 2)
```

The package `drawing_robot.stage2` is offline-only. It does not open a serial
device, create a runtime robot connection, read telemetry, or issue motion.
An architecture test protects that boundary.

## Minimal use

```python
from drawing_robot.stage2 import (
    CartesianTrajectory,
    SequentialIKSolver,
    Stage2Config,
    TrajectoryRejected,
    TrajectoryValidator,
)

cartesian = CartesianTrajectory.from_arrays(times_s, poses_mm_deg)
configuration = Stage2Config()

joint_trajectory = SequentialIKSolver(configuration).solve(
    cartesian,
    seed=start_configuration_deg,
)
result = TrajectoryValidator(configuration).validate(
    cartesian,
    joint_trajectory,
)

if not result.valid:
    raise TrajectoryRejected(result)
```

All `sample_index` and `joint_index` fields are zero-based. Human-readable
messages use the conventional joint names J1 through J6.

## Checks and units

| Check | Rejects on | Representative issue code |
| --- | --- | --- |
| Timestamps | non-finite, non-increasing, or too-small spacing | `TIMESTAMP_NONFINITE`, `TIMESTAMP_NOT_INCREASING`, `TIMESTEP_TOO_SMALL` |
| Sequential IK | missing, malformed, non-finite, or out-of-limit solution | `IK_FAILED`, `IK_JOINT_COUNT_INVALID`, `IK_SOLUTION_NONFINITE` |
| Joint limits | any angle outside the firmware range plus 3° margin | `JOINT_LIMIT_EXCEEDED` |
| Continuity | sampled joint step above the configured threshold | `JOINT_DISCONTINUITY` |
| Velocity | `dq / actual_dt` above a project limit | `VELOCITY_EXCEEDED` |
| Acceleration | velocity change divided by interval-midpoint time | `ACCELERATION_EXCEEDED` |
| FK consistency | reconstructed pose outside configured tolerance | `FK_POSITION_ERROR`, `FK_ORIENTATION_ERROR` |
| Workspace | sampled position outside configured bounds | `WORKSPACE_EXCEEDED` |
| Sample density | Cartesian step above configured maximum | `CARTESIAN_STEP_EXCEEDED` |
| Jacobian | numerical failure | `SINGULARITY_NUMERICAL_FAILURE` |

Low minimum singular value is diagnostic-only and produces
`LOW_SINGULARITY_MARGIN`, a warning rather than a rejection.

Positions are millimetres, joint angles are degrees, time is seconds, velocity
is degrees per second, and acceleration is degrees per second squared. FK
position error is the Euclidean distance. FK orientation error is the largest
absolute wrapped Euler-component error in the project's existing convention.

## Configuration provenance

Important limits are represented by `LimitValue(value, unit, source,
verified)`. The exact project joint limits are:

```text
J1 [-165, 165]   J2 [-137, 137]   J3 [-147, 147]
J4 [-147, 147]   J5 [-152, 157]   J6 [-177, 177] deg
```

Velocity, acceleration, continuity, sampling, FK and singularity defaults are
project assumptions, not manufacturer safety values. `ValidationResult`
therefore reports their names in `metrics["unverified_assumptions"]` and emits
an `UNVERIFIED_PROJECT_LIMITS` warning. The default workspace is intentionally
unset instead of inventing safe coordinates. This rejects validation with
`WORKSPACE_UNDEFINED` until provisional or measured bounds are supplied.

## Result model

`ValidationResult.valid` is false exactly when at least one `ERROR` exists.
Warnings do not reject the trajectory. Metrics include extrema and their worst
sample/joint indices: joint margin, joint step, velocity, acceleration, FK
errors, Cartesian spacing, and minimum Jacobian singular value.

The validator evaluates the supplied samples only. It does not prove what
happens between samples, which is why both maximum joint step and maximum
Cartesian step are enforced.

## Verification

Run the complete hardware-free suite:

```bash
python -m pytest -q
```

Reference tests cover a valid small motion, limit and branch violations,
velocity and acceleration, malformed values and timestamps, IK failure,
non-uniform timing, FK mismatch, workspace/sample spacing, singularity
diagnostics, and a Cartesian S-curve processed through sequential IK.

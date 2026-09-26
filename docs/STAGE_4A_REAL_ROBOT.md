# Stage 4A — Real Robot Integration

Stage 4A reuses the validated `JointTrajectory`, `MotionExecutor`, `RobotState`
and versioned `ExecutionLog` from Stages 2 and 3. `MyCobotRobot` is the only
adapter visible to the executor. It delegates all hardware work to the existing
single-owner `RobotService`/`RobotIO` path.

## Safety boundary

The implementation is ready for conservative, supervised free-space
integration; it is not a safety controller. A run requires all of the
following:

- valid Stage-2 result and Stage-4A envelope;
- fresh angles, temperatures and controller state;
- temperature below the project 60 °C abort threshold;
- local operator supervision, a reachable physical stop and a visibly clear
  workspace;
- both `RUN_REAL_ROBOT_TESTS=1` and the CLI `--execute` flag.

READY and PARK remain deliberately unconfirmed while their calibrated joint
configurations are `None`. PARK never performs an implicit recovery movement.
The 55/60 °C thresholds and motion limits are project assumptions, not
manufacturer-certified safety limits.

## Architecture

```text
MotionExecutor → MyCobotRobot → RobotService → RobotIO → pymycobot/UART
                                      ↑             │
                                      └─ telemetry ─┘
```

`RobotIO` owns the exclusive process lock and records queue, start and end
times for every UART transaction. The hardware adapter preserves planned,
sent and measured values separately and never invents a physical activation
timestamp.

## Verification levels

Normal `pytest` is hardware independent. `tests/hardware/` is skipped unless
the opt-in environment variable is present. The read-only hardware smoke test
does not move the robot.

A supervised trajectory run on the Nano is intentionally a separate action:

```bash
RUN_REAL_ROBOT_TESTS=1 python scripts/run_hardware_stage4a.py bundle.json \
  --output run.json --execute --operator-supervising \
  --physical-stop-accessible --workspace-clear \
  --workspace-bounds XMIN XMAX YMIN YMAX ZMIN ZMAX
```

The workspace numbers are mandatory because the project calibration is still
unset; the CLI will not invent them. Operator-supplied bounds are logged as an
unverified project assumption.

Run the exact same bundle through `scripts/run_simulation.py`, then compare:

```bash
python scripts/compare_sim_real.py sim.json run.json --output comparison.json
```

Until that supervised run succeeds, Stage 4A is **offline-complete but not
hardware-qualified**.

# Stage 3 — SimRobot Execution

Stage 3 answers:

> Does the execution software handle timing, commands, robot state, stops and
> faults correctly before real hardware is connected?

It simulates execution behaviour, not mechanics, dynamics, collision, contact,
temperature, torque, or hardware latency.

## Data flow

```text
accepted Stage-2 JointTrajectory
              |
       MotionExecutor
              |
         JointCommand
              |
       RobotInterface
              |
          SimRobot
              |
         RobotState
              |
        ExecutionLog
```

`MotionExecutor` imports and calls only `RobotInterface`. `SimRobot` is one
implementation of that protocol and can later be replaced without changing
the scheduling contract.

## Timing model

No Stage-3 module reads wall-clock time. `SimulationClock.advance(dt)` executes
registered phases in fixed order:

1. scheduled backend faults;
2. delayed command activation;
3. rate-limited state evolution;
4. state publication;
5. executor observation and logging.

Trajectory timestamps, command activation, simulation timestep, and state
publication rate remain separate. Commands are sent open-loop when their
trajectory samples become due. State freshness, faults, final tolerance, and
completion timeout are monitored closed-loop.

## State and completion

Robot states use one exclusive status: `IDLE`, `MOVING`, `STOPPING`, `STOPPED`,
or `FAULT`. For each joint, simulated movement per update is limited to
`velocity_limit * dt` and is clipped at the target, so it cannot overshoot.

Completion requires all of the following:

- every trajectory command was issued;
- the final command ID is active;
- every actual joint lies within the configured final tolerance;
- no stop, fault, stale-state condition, or timeout is active.

Sending the last sample is therefore not completion.

## Latency, jitter, stop and faults

`ExecutionConfig` centralizes all Stage-3 values. Command latency supports:

- `ideal`: zero delay;
- `fixed`: configured base delay;
- `jittered`: base delay plus seeded uniform offset.

The log records planned, send and activation time plus effective delay for
every command. A fixed seed makes event order and final state reproducible.

A stop request rejects future commands while the active target remains in
effect for `stop_delay_s`. When that delay expires, the simulated state freezes
and `STOP_APPLIED` records additional joint movement after the request. This is
not a braking or certified safety-stop model.

Configured dropped commands are accepted but never activated. Stale-state
faults stop fresh publications; the executor—not the backend—detects the age
timeout. A backend fault moves the state to `FAULT`. A missing final activation
or unreachable final target ends in completion timeout.

## Execution log and replay

The versioned `shared-growth/execution/v1` log contains:

- run and trajectory IDs;
- complete simulation configuration and seed;
- initial state;
- command records;
- published states;
- stable-sequence events;
- final result.

`replay_log()` iterates recorded states and events without running SimRobot.
Replay is evidence inspection, not re-simulation. Deterministic rerun is tested
separately by executing the same trajectory, configuration, initial state and
seed twice and comparing commands, events and final state.

## CLI

Run a validated Stage-2B JSON bundle:

```bash
python scripts/run_simulation.py trajectory.json \
  --output data/stage3/run.json \
  --trajectory-id branch-001
```

Inject repeatable command timing:

```bash
python scripts/run_simulation.py trajectory.json \
  --output data/stage3/jitter.json \
  --latency-mode jittered \
  --base-delay-ms 100 \
  --jitter-min-ms -20 \
  --jitter-max-ms 20 \
  --seed 42
```

Inspect the recorded evidence without rerunning:

```bash
python scripts/replay_simulation.py data/stage3/run.json
```

## Interpretation boundary

A completed Stage-3 run demonstrates deterministic execution-software
behaviour under the selected test configuration. It does not establish
physical feasibility, collision safety, thermal safety, real latency, braking
distance, or human-interaction quality.

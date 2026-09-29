# Robot service and UART ownership

## Invariant

`drawing_robot/robot/io.py` is the only active module allowed to import or
construct `MyCobot280`. A repository test enforces this rule. On Linux,
`RobotIO` also claims a non-blocking lock for the selected serial device, so a
dashboard and a standalone motion process cannot silently share the UART.

## Runtime path

```text
application
    -> RobotService
        -> priority queue
            0  stop
            10 motion/control
            20 essential telemetry
            30 diagnostic telemetry
        -> RobotIO
            -> MyCobot280
                -> UART
```

The Stage-4A runtime profile deliberately keeps the motion loop lean:

- critical at 5 Hz: `get_angles()` and `get_error_information()`;
- health at 1 Hz: `get_servo_temps()`;
- startup/end/on-demand only: voltage, servo status, power, servo-enable state
  and firmware pose.

`get_servo_speeds()` is not used in the critical motion loop. Hardware
characterization showed high and highly variable blocking latency on the
single UART, so motion cessation is confirmed from successive fresh angle
samples instead. Stale polls are coalesced rather than allowed to build a
queue behind motion commands.

## Policy and qualification

Runtime policy exposes structured gate results with two categories. `HARD`
results cover safety/runtime availability and can block motion. `PERFORMANCE`
results retain READY, completion and settling misses as qualification evidence
without mislabelling them as controller faults. Missing hard/runtime evidence
is represented as `HARD/BLOCK`; performance evidence uses `NOMINAL`, `WARN`,
or `DEGRADED`.

`SystemSnapshotAdapter` provides the read-only cockpit contract. It aggregates
the authoritative `RobotState`, optional Stage-2 validation and an existing
`ExecutionLog`; it does not expose commands, `RobotIO`, UART or pymycobot.

## Motion authorization

The service begins DISARMED. A motion command is rejected until
`arm_motion(locally_confirmed=True)` completes a fresh fast and slow preflight.
A gate failure disarms motion. Recovery and parking are distinct motion classes
and still require local confirmation.

`stop_motion()` requests the controller's software stop and retains servo
support. `disable_torque(locally_supported=True)` is deliberately separate and
refuses to run without an explicit assertion that the arm is mechanically
supported. Neither operation is presented as a certified emergency stop.

## Planner/executor boundary

The planner may generate an arbitrarily detailed offline joint trajectory.
Stage 2 validates timing and motion constraints before the backend-independent
`drawing_robot.execution.MotionExecutor` schedules `JointCommand` objects
through `RobotInterface`. With real hardware, `MyCobotRobot` adapts those
commands to `RobotService` and records command/API evidence in `ExecutionLog`.
This preserves three distinct objects for analysis:

1. planned trajectory;
2. commanded trajectory;
3. observed telemetry trajectory.

## Dashboard

The dashboard creates a read-only `RobotService` and subscribes to its published
state. It does not import pymycobot or call the UART itself. It exposes no
motion endpoint. Because standalone motion scripts are separate processes, the
dashboard service must still be stopped before one is launched; the device lock
enforces that operational rule.

## Current limits

- The service is not a safety-rated controller.
- UART calls cannot be preempted once the firmware transaction has begun.
- Existing version-1 hardware JSON files retain their original schemas and are
  historical evidence. Current Stage-3/Stage-4 runs use the versioned
  `drawing_robot.execution.ExecutionLog`; historical files are not rewritten.
- Cartesian hardware execution is disabled until READY and PARK states are
  measured and thermally/mechanically validated.
- Hardware behavior must be revalidated after deploying this refactor.

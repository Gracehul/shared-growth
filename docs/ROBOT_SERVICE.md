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

Fast telemetry contains joint angles, reported joint speeds and controller
errors. Slow telemetry contains temperatures, voltages, servo status, power,
servo-enable state and firmware pose. Stale polls are coalesced rather than
allowed to build a queue behind motion.

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
`MotionExecutor` validates its timebase and configured command rate, submits
the scheduled joint targets to `RobotService`, and records the actual command
issue/return times. It requests a software stop after an execution exception
and returns to DISARMED by default. This preserves three distinct objects for
analysis:

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
- Existing version-1 hardware JSON files retain their original schemas. New
  unified logging is available through `drawing_robot.runlog.RunLog`; migration
  should happen per runner without rewriting historical evidence.
- Cartesian hardware execution is disabled until READY and PARK states are
  measured and thermally/mechanically validated.
- Hardware behavior must be revalidated after deploying this refactor.

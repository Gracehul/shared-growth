# Safety protocol

This protocol is a project checklist, not a substitute for the myCobot manual,
institutional laboratory rules, supervision, or a formal risk assessment.
Hardware-specific limits that have not been measured are marked `TBD`.

## Required setup

- Mount the robot on a stable surface.
- Fix the drawing surface so it cannot move into the robot workspace.
- Secure the pen with a compliant or spring-loaded holder where possible.
- Remove loose clothing, cables, and unrelated objects from the workspace.
- Keep the emergency stop and power control reachable.
- Establish a participant boundary outside the robot's swept volume.
- Confirm the active tool, coordinate frame, workspace bounds, safe Z height,
  speed limit, and force/compliance assumptions before motion.

## Before every session

1. Inspect the robot, mount, gripper, pen holder, cables, and paper fixture.
2. Verify the configured robot and serial connection.
3. Confirm that the drawing area and exclusion zone are clear.
4. Test stop behavior without a participant present.
5. Run the planned path offline or in mock mode.
6. Run a reduced-speed, pen-up dry run.
7. Approach the paper only after the dry run succeeds.
8. Record operator, configuration version, calibration identifier, and any
   anomaly in the session log.

## Motion rules

- Use the minimum practical speed and acceleration during development.
- Plan the full trajectory and validate joint/workspace limits before motion.
- Enter and leave the drawing plane vertically using a verified safe Z height.
- Never allow a participant's hand inside the robot workspace during robot
  motion.
- Do not bypass a failed reachability, joint-limit, or calibration check.
- Stop after unexpected contact, sound, vibration, pose, or communication loss.

## Stop conditions

Trigger a safe stop when any of the following occurs:

- a person or object enters the exclusion zone;
- the pen, holder, paper, or robot mount moves unexpectedly;
- telemetry is missing or inconsistent;
- the executed trajectory exceeds the permitted error (`TBD` threshold);
- the control process crashes, times out, or loses the robot connection;
- the operator or participant requests a stop.

## Recovery

1. Stop command output and disable further automatic transitions.
2. If motion continues or safety is uncertain, use the emergency stop or power
   control according to the manufacturer and lab procedure.
3. Ask the participant to move away before entering the workspace.
4. Record the event and preserve relevant non-identifying logs.
5. Inspect the physical setup and identify the failure cause.
6. Re-home only when the path is clear and the robot state is known.
7. Repeat offline and pen-up checks before resuming.

## Values to validate on hardware

| Item | Value |
| --- | --- |
| Maximum development speed | TBD |
| Maximum study speed | TBD |
| Safe Z height | TBD |
| Drawing-plane Z | TBD |
| Drawing workspace bounds | TBD |
| Maximum permitted path error | TBD |
| Stop timeout | TBD |
| Pen compliance/travel | TBD |

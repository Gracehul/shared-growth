# Robot Interaction Workstation

Browser client for the Shared Growth cockpit operation contract.

## Boundaries

- Runtime values come from `SystemSnapshotAdapter` through the local
  `/api/workstation/status` endpoint.
- High-level requests go through `OperationController`; the browser has no raw
  motion or hardware endpoint.
- No `pymycobot`, UART, `RobotIO`, FK, validator or executor implementation.
- Selecting an operation never sends it. Choosing the RUN workflow submits the
  selected prepared operation and renders the authoritative receipt.
- The existing Python M1 cockpit and validated execution core remain authoritative.

## Run locally

From the repository root, start the safe offline integration:

```bash
python scripts/robot_interaction_workstation.py
```

Then open `http://127.0.0.1:8766`. The default composition uses `SimRobot` and
does not import or connect to hardware. Opening `index.html` directly leaves
the UI visibly offline and disables operation submission.

## API boundary

```text
GET  /api/workstation/status
POST /api/workstation/mode
POST /api/workstation/operations
POST /api/workstation/stop
```

Only prepared operation IDs cross this boundary. Raw joint angles, Cartesian
targets, UART and pymycobot are not exposed.

## Interaction model

Operating modes:

```text
OBSERVE | MANUAL | SHARED GROWTH
```

Workflow context:

```text
AUTHOR | PREVIEW | RUN | REVIEW
```

Primary explanatory chain:

```text
PERCEIVE → INTEND → GATE → ACT
```

`MANUAL` invokes prepared operations rather than exposing raw joint or Cartesian
jog controls. Gate names, decisions, execution state and evidence come from the
backend contracts and are not recomputed in the browser.

The measured current robot pose is sourced from authoritative `RobotState` and
does not depend on workspace calibration. `CALIBRATE WORKSPACE` is reserved for
establishing or validating the robot/tool/camera/drawing-surface relationship;
the workstation reports it as `NOT AVAILABLE` until a backend contract exists.

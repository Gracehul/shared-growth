# Robot Interaction Workstation UI prototype

Static interaction-design prototype for the Shared Growth cockpit.

## Boundaries

- Frontend only: HTML, CSS, JavaScript and a local SVG asset.
- All displayed runtime values are explicit mock data.
- No network requests.
- No `pymycobot`, UART, `RobotIO`, FK, validator or executor implementation.
- Buttons demonstrate UI state transitions only and never command a robot.
- The existing Python M1 cockpit and validated execution core remain authoritative.

## Run locally

Open `index.html` in a browser, or serve this directory with any static file
server.

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
jog controls. Technical gate names and semantics must come from the backend
contract when integration begins.

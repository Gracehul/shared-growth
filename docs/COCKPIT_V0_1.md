# Shared Growth Cockpit M1

The cockpit is a read-only desktop observer. It does not introduce a robot
control path, a second `RobotState`, or a second `MotionExecutor`.

## Application shell

The v0.1 RUN/OBSERVE screen now lives in one shared shell:

```text
ApplicationShell
├── AUTHOR / PREVIEW / RUN / REVIEW workspace header
├── InputSensingPanel
├── ParameterInspector (M1 extension point)
├── WorldView
│   ├── camera
│   └── existing Stage-2B MotionVisualizer
├── MachineStatePanel
├── SignalTimeline (existing ExecutionLog events or empty state)
└── PipelineView
```

`RUN` is the functional v0.1 workspace. The other workspaces change information
emphasis but intentionally do not claim functionality that has not been built.

## Data mapping

| UI element | Authoritative source | Data |
|---|---|---|
| Camera preview | `CameraSource`, following `camera.capture` conventions | BGR frame, resolution, FPS, age, failed/dropped frames |
| Robot state | `RobotService.subscribe()` | existing `drawing_robot.robot.state.RobotState` |
| Actual joints | `RobotState.angles_deg` | degrees |
| Target joints | optional `ExecutionLog.commands` | last recorded command target |
| Temperature/fault/freshness | `RobotState` | temperature, error/fault, critical sample age |
| Trajectory view | Stage-2B bundle | `CartesianTrajectory`, `JointTrajectory` |
| Validation | Stage-2B bundle | existing `ValidationResult` |
| Run result | optional `ExecutionLog` | recorded result, not live telemetry |
| Simulation state | SimRobot `ExecutionLog` | same recorded state/event contract |

Camera-to-motion sensing and mapping are visibly marked **NOT INTEGRATED**.

## Safe startup

Offline shell with no camera or robot connection:

```bash
python scripts/shared_growth_cockpit.py
```

Complete mock display:

```bash
python scripts/shared_growth_cockpit.py --robot mock --camera mock \
  --trajectory outputs/physical-demo-plan.json
```

Nano camera plus one read-only `RobotService` owner:

```bash
python scripts/shared_growth_cockpit.py --robot read-only --camera auto \
  --trajectory <stage2b-bundle.json> --execution-log <run.json>
```

Do not start the read-only hardware mode while another process owns the UART.
The cockpit contains no command or stop buttons.

The optional camera dependencies are installed with:

```bash
pip install -e '.[cockpit]'
```

`--check` prints the source mapping without opening a window.

## Minimal future data contract proposal

M1 does not add new runtime data classes. The compatible minimum for later
milestones is:

- **Preset** (M2): editable authoring values only. It should resolve into the
  existing configuration types rather than replacing them.
- **RunConfig** (M2/M3): an immutable snapshot of the existing resolved
  `ExecutionConfig` / Stage-4A config plus trajectory and configuration hashes.
- **RunRecord** (M3): the frozen RunConfig provenance plus the existing
  versioned `ExecutionLog` and its result.

`ExecutionLog` already contains simulation configuration, provenance, commands,
states, telemetry, UART timing, events and result. A second run-record or event
system would duplicate it, so none is introduced in M1.

The M1 timeline can show the existing event kinds (`COMMAND_SENT`,
`STATE_PUBLISHED`, stop/fault/timeout/run events). Continuous synchronized
camera, human-input and experimental channels do not yet exist; M3 will need
explicit timestamped channel references in the RunRecord before those can be
shown honestly.

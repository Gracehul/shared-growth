# Stage 2B — Motion Visualization

Stage 2B provides offline visual inspection of an existing Stage-2 plan:

> Does the planned motion look spatially and temporally plausible before
> simulated execution?

It is a visualization, not a dynamics simulator, safety check, trajectory
repair tool, or hardware-readiness approval.

## Authoritative inputs

| View | Source |
| --- | --- |
| 3D robot pose | `JointTrajectory` through the existing FK/frame chain |
| Complete end-effector path | `CartesianTrajectory` |
| Six joint curves | `JointTrajectory` |
| Status, issue codes and metrics | `ValidationResult` |

`MotionVisualizer` does not implement kinematics or validation. It calls the
existing `drawing_robot.kinematics.frame_chain` and
`forward_kinematics` functions solely to obtain display geometry. Current FK
error and Jacobian values are read from the per-sample metrics Stage 2 already
computed.

## Python API

```python
from drawing_robot.stage2.visualization import MotionVisualizer

view = MotionVisualizer(
    cartesian_trajectory,
    joint_trajectory,
    validation_result,
    config=stage2_config,
)
view.select_sample(43)  # zero-based API index
view.show()
```

The display includes:

- a fixed-scale 3D base/link/tool representation;
- visible base origin and X/Y/Z axes in millimetres;
- the complete Cartesian path plus start, end and selected sample;
- all six joint angles, their configured Stage-2 limits and current time;
- Play, Pause and Restart controls using actual timestamp spacing;
- explicit `VALID`, `VALID WITH WARNINGS`, or `INVALID` text;
- separate global metrics and selected-sample details.

Displayed sample numbers are one-based for humans. Python selection indices
are zero-based.

## Input contract

Visualization rejects, without repairing:

- empty trajectories;
- different sample counts;
- non-finite or non-increasing timestamps;
- timestamps that do not match exactly;
- malformed or non-finite six-value Cartesian/joint samples;
- FK geometry that cannot be generated.

These are display-input checks only. An invalid `ValidationResult` can still be
opened for diagnosis and remains prominently labelled `INVALID`.

## CLI and JSON format

Interactive inspection:

```bash
python scripts/visualize_trajectory.py trajectory.json
```

Headless PNG export:

```bash
python scripts/visualize_trajectory.py trajectory.json --save stage2b.png
```

Timestamp-driven MP4 export:

```bash
python scripts/visualize_trajectory.py trajectory.json \
  --save-animation stage2b.mp4 --fps 30 --playback-speed 1
```

The MP4 uses a constant video frame rate but maps each frame back to the latest
planned timestamp. Samples are held between timestamps; no extra robot states
are interpolated. `--playback-speed 2` produces a 2× review without changing
the underlying trajectory.

Optional initial selection:

```bash
python scripts/visualize_trajectory.py trajectory.json --sample 43
```

The JSON interchange format is intentionally a direct representation of the
three Stage-2 artifacts:

```json
{
  "cartesian_trajectory": {
    "timestamps_s": [0.0, 0.1],
    "poses_mm_deg": [[100, 0, 150, 180, 0, 0], [101, 0, 150, 180, 0, 0]]
  },
  "joint_trajectory": {
    "timestamps_s": [0.0, 0.1],
    "positions_deg": [[0, 20, -40, 20, 0, 0], [0.2, 20, -40, 20, 0, 0]]
  },
  "validation_result": {
    "valid": true,
    "errors": [],
    "warnings": [],
    "metrics": {}
  }
}
```

Issue objects use `code`, `severity`, `message`, and optional zero-based
`sample_index` and `joint_index` fields.

## Interpretation boundary

The slider replays planned samples only. It does not represent motor dynamics,
controller timing, contact, latency, or execution. Stable 3D bounds are
derived once from the complete Cartesian path and all FK display geometry, so
the camera scale does not change between samples. Equal XYZ scaling prevents
geometric distortion.

An accepted plan may proceed to Stage 3 `SimRobot`. Physical execution still
requires the later calibration, safety, recovery and hardware gates.

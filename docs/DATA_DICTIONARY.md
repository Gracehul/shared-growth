# Data dictionary — draft

## Logging principles

- Use newline-delimited JSON or an equivalent event table as the canonical log.
- Use a monotonic timestamp for durations and ordering.
- Also record UTC time in ISO 8601 format for cross-device alignment.
- Record the intended condition separately from what physically occurred.
- Use centimetres/degrees at the public robot-control boundary and state units
  explicitly for every geometry field.
- Never store names, email addresses, consent forms, or direct identifiers in
  the event log.

Hardware characterization uses the shared `RunLog` envelope:

```text
schema, run_id, started_utc, hardware, software, config, timeline, result
```

The common runtime timeline uses `COMMAND_SENT`, `STATE_SAMPLE`, `TEMP_SAMPLE`,
`FAULT`, `STOP_REQUESTED`, `MOTION_DETECTED`, and `MOTION_SETTLED`. Individual
tests may add domain-specific payload fields without changing those event
semantics. Planned, commanded, and observed trajectories must remain separate.

## Core event fields

| Field | Type | Unit/example | Description |
| --- | --- | --- | --- |
| `schema_version` | string | `0.1.0` | Log schema version |
| `session_id` | string | pseudonymous UUID | One experimental session |
| `participant_id` | string | pseudonymous code | No direct identity information |
| `trial_id` | string | UUID | One condition trial |
| `turn_index` | integer | `0, 1, ...` | Human-robot turn within trial |
| `event_id` | string | UUID | Unique event record |
| `event_type` | enum | see below | Event category |
| `timestamp_monotonic_ns` | integer | ns | Ordering and latency calculation |
| `timestamp_utc` | string | ISO 8601 UTC | Cross-device/session alignment |
| `source` | enum | `human`, `robot`, `camera`, `controller`, `sensor` | Event producer |
| `condition` | enum | `baseline`, `fixed_delay`, `jitter` | Assigned timing condition |
| `software_version` | string | Git commit SHA | Reproducible controller version |
| `calibration_id` | string | UUID/hash | Calibration used for the event |
| `valid` | boolean | `true` | Whether the event passed validation |
| `error_code` | string/null | controlled vocabulary | Failure or exclusion reason |

## Stroke and trajectory fields

| Field | Type | Unit | Description |
| --- | --- | --- | --- |
| `start_x`, `start_y` | number | cm | Stroke start in drawing-surface frame |
| `end_x`, `end_y` | number | cm | Stroke end in drawing-surface frame |
| `direction_rad` | number | rad | Direction from start to end |
| `length_cm` | number | cm | Stroke or trajectory length |
| `duration_ms` | number | ms | Detected event duration |
| `mean_speed_cm_s` | number | cm/s | Mean drawing speed |
| `path_points` | array | cm + relative time | Ordered geometry when required |
| `frame_id` | string | e.g. `drawing_surface` | Coordinate frame for geometry |

## Timing fields

| Field | Type | Unit | Description |
| --- | --- | --- | --- |
| `intended_delay_ms` | number | ms | Delay requested by the condition |
| `stroke_end_ns` | integer | ns | Human stroke-completion timestamp |
| `response_release_ns` | integer | ns | Controller releases robot action |
| `robot_motion_start_ns` | integer | ns | First observed robot motion |
| `robot_motion_end_ns` | integer | ns | Observed completion of robot motion |
| `processing_latency_ms` | number | ms | Stroke end to response readiness |
| `release_latency_ms` | number | ms | Stroke end to controller release |
| `physical_response_latency_ms` | number | ms | Stroke end to actual robot motion |
| `timing_error_ms` | number | ms | Observed minus intended response time |

## Suggested event types

`session_start`, `trial_start`, `human_stroke_start`, `human_stroke_end`,
`stroke_extracted`, `trajectory_generated`, `delay_started`, `response_released`,
`robot_motion_start`, `robot_motion_end`, `safety_stop`, `error`, `trial_end`, and
`session_end`.

## Case B and C extensions

Derived coordination or adaptation features must include their input window,
algorithm version, and confidence/quality flag. Physiological data requires a
separate approved schema, sampling metadata, synchronization evidence, and
access controls; it must not be added casually to the core event file.

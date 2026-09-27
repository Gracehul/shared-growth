# Cockpit core contract

The cockpit consumes the existing runtime through the read-only exports in
`drawing_robot.hardware`. It must not import or expose `RobotIO`, pymycobot,
UART operations or joint-command methods.

## Adapter

`SystemSnapshotAdapter` is the integration boundary. The cockpit may call:

```python
snapshot = adapter.get_system_snapshot()
gates = adapter.get_gate_results()
latest_event = adapter.get_latest_execution_event()
thresholds = adapter.get_thresholds()
```

`SystemSnapshot` aggregates the authoritative `drawing_robot.robot.RobotState`,
optional Stage-2 `ValidationResult`, target values and an existing
`ExecutionLog`. It is not a replacement robot-state model.

## Gates

Every `GateResult` contains:

```text
name
category: HARD | PERFORMANCE
status: PASS | WARN | BLOCK | UNKNOWN
value / limit / unit / reason
```

`HARD` `BLOCK` or `UNKNOWN` prevents execution. A `PERFORMANCE` miss remains a
visible qualification result and must not be presented as a controller or
hardware fault. `UNKNOWN` must never be rendered as `PASS`.

## Thresholds

`get_thresholds()` returns the current unchanged Stage-4A values together with
unit, source and characterization status. The cockpit displays this metadata;
it does not edit or reinterpret it.

## Execution events

`get_latest_execution_event()` returns the latest versioned `ExecutionLog`
event as:

```text
sequence
timestamp_s
kind
data
```

The aggregate execution phase uses the stable display vocabulary `IDLE`,
`PRECHECK`, `READY_CHECK`, `RUNNING`, `SETTLING`, `STOPPING`, `STOPPED`,
`COMPLETED`, and `FAILED`. `STOPPED` is valid only when measured cessation was
confirmed. A successful stop API call alone is insufficient.

## Evidence attribution

`analyze_execution_log()` preserves `run_id`, `trajectory_id`, schema version,
hardware flag, safety profile, telemetry profile and provenance beside every
derived metric. Cockpit comparisons must retain those source fields.

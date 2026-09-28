# Cockpit high-level operation contract

The cockpit imports `drawing_robot.operations` and submits identifiers, never
motion primitives. The existing read side remains `SystemSnapshotAdapter`.

## Composition

```text
Cockpit
  -> OperationController
      -> existing GateResult provider
      -> MotionExecutor
          -> RobotInterface
              -> existing hardware or simulation backend
```

The real application supplies a `gate_provider` backed by the current robot
core policy. If no provider exists, the controller returns
`RUNTIME_GATES_UNKNOWN` and does not execute. The provider must return only the
gate evidence applicable to the requested operation; the controller does not
recalculate hardware policy. Stage-2 trajectory validity is always taken from
the resolved `PreparedTrajectory`.

## Public operations

```python
request = OperationRequest(
    request_id="request-42",
    mode=InteractionMode.MANUAL,
    operation=Operation.EXECUTE_TRAJECTORY,
    trajectory_id="demo_phrase_01",
)
receipt = controller.request(request)
```

The public operations are `GO_READY`, `EXECUTE_TRAJECTORY`,
`START_SHARED_GROWTH`, and `STOP`. `PreparedTrajectory` objects are populated by
the trusted application composition, not assembled from cockpit sliders or
raw coordinates.

`SHARED_GROWTH` exposes only session start/stop in this milestone. A direct UI
`EXECUTE_TRAJECTORY` request in that mode is rejected as internal-only. Future
perception/mapping code must resolve a validated prepared trajectory and enter
through this same controller/executor path.

## Receipt and evidence

`OperationReceipt` is authoritative for `ACCEPTED`,
`ACCEPTED_WITH_WARNING` or `REJECTED`, state, reason and gate results. Only a
`HARD/BLOCK` rejects a request; non-nominal performance results remain attached
to an accepted receipt. The cockpit must not infer a rejection reason. `OperationEvidence`
retains perception, intention, command and actual-state references. Missing
perception or intention is represented as `ABSENT`; unavailable actual evidence
is `UNKNOWN`.

Execution receipts retain `request_id`, `run_id`, every `command_id`, and the
final `RobotState.timestamp_s`. `latest_execution_event` uses the existing
versioned `ExecutionLog` event contract.

The workstation keeps three records separate:

```text
CURRENT GATE       current SystemSnapshot gate results
OPERATION DECISION latest request receipt, including rejected requests
LAST ACTION        latest operation that actually entered execution
```

A rejected request updates OPERATION DECISION but never overwrites LAST ACTION.

## Stop

`STOP` bypasses interaction-mode permission checks but not the robot backend:

```text
Cockpit -> OperationController.stop() -> RobotInterface.stop_motion()
```

The existing hardware backend continues through `RobotService` and prioritized
`RobotIO`. A successful API call that has not produced measured cessation is
reported as `STOPPING` / `STOP_PENDING`, never `STOPPED`.

## Forbidden UI surface

Do not expose raw joint angles, raw Cartesian coordinates, servo identifiers,
firmware speed, UART calls, pymycobot methods, torque release or joint jogging.

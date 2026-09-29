from drawing_robot.execution import (
    ExecutionConfig,
    ExecutionLog,
    JointCommand,
    RobotState,
    RobotStatus,
    replay_log,
)


def make_log():
    state = RobotState(0, [0] * 6, [0] * 6, RobotStatus.IDLE)
    log = ExecutionLog("trajectory-1", ExecutionConfig().to_dict(), 7, state, "run-1")
    log.record_state(state)
    log.add_event("STOP_REQUESTED", 0.1)
    stopped = RobotState(0.2, [0] * 6, [0] * 6, RobotStatus.STOPPED)
    log.record_state(stopped)
    log.add_event("STOP_APPLIED", 0.2, additional_movement_norm_deg=0.0)
    log.finish("stopped", 0.2)
    return log


def test_execution_log_round_trip(tmp_path) -> None:
    original = make_log()
    path = original.write(tmp_path / "execution.json")
    loaded = ExecutionLog.read(path)
    assert loaded.to_dict() == original.to_dict()
    assert loaded.schema_version == "shared-growth/execution/v2"


def test_replay_uses_recorded_data_without_simulation() -> None:
    frames = replay_log(make_log())
    assert [frame.timestamp_s for frame in frames] == [0.0, 0.1, 0.2]
    assert frames[-1].state["status"] == "STOPPED"
    assert any(event["kind"] == "STOP_APPLIED" for event in frames[-1].events)


def test_command_evidence_is_updated_through_public_contract() -> None:
    log = make_log()
    command = JointCommand("command-1", (1, 2, 3, 4, 5, 6), 0.25)
    log.command_sent(command, 0.3)

    assert log.record_command_evidence(
        "command-1", api_duration_s=0.04, success=True
    )
    assert not log.record_command_evidence("absent", success=False)
    assert log.commands[-1]["api_duration_s"] == 0.04
    assert log.commands[-1]["success"] is True

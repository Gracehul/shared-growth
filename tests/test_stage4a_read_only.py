from dataclasses import replace

from drawing_robot.hardware import Stage4AConfig
from drawing_robot.hardware.read_only_validation import validate_read_only_session
from drawing_robot.robot import RobotService


def broad_config():
    return replace(
        Stage4AConfig(),
        allowed_workspace_mm=((-500, 500), (-500, 500), (-100, 500)),
        workspace_source="test",
    )


def test_strict_read_only_requires_consecutive_valid_samples_and_allowlist() -> None:
    with RobotService(mock=True, read_only=True, telemetry=False) as service:
        result = validate_read_only_session(
            service,
            broad_config(),
            physical_preflight_pass=True,
            min_consecutive_samples=3,
            sample_period_s=0.02,
        )
        assert result.decision == "GO"
        assert len(result.log.telemetry) == 3
        assert [sample["local_sample_sequence"] for sample in result.log.telemetry] == [1, 2, 3]
        assert result.log.result["motion_commands_sent"] == 0
        assert result.log.result["servo_release_calls"] == 0
        assert result.log.result["state_changing_calls"] == 0
        assert all(item["method"].startswith("get_") or item["method"].startswith("is_") for item in result.log.uart_transactions)


def test_raw_error_17_is_logged_and_blocks_without_clearing() -> None:
    with RobotService(mock=True, read_only=True, telemetry=False) as service:
        service.io._backend.get_error_information = lambda: 17
        result = validate_read_only_session(
            service,
            broad_config(),
            physical_preflight_pass=True,
            min_consecutive_samples=2,
            sample_period_s=0.02,
        )
        assert result.decision == "NO_GO"
        assert "BLOCKING_CONTROLLER_FAULT_RAW_17" in result.abort_reason
        assert all(sample["raw_error_state"] == 17 for sample in result.log.telemetry)
        assert result.log.result["state_changing_calls"] == 0


def test_current_pose_outside_provisional_box_blocks_motion_escalation() -> None:
    config = replace(
        Stage4AConfig(),
        allowed_workspace_mm=((900, 1000), (900, 1000), (900, 1000)),
    )
    with RobotService(mock=True, read_only=True, telemetry=False) as service:
        result = validate_read_only_session(
            service,
            config,
            physical_preflight_pass=True,
            min_consecutive_samples=2,
            sample_period_s=0.02,
        )
        assert result.decision == "NO_GO"
        assert "CURRENT_POSE_OUTSIDE_PROVISIONAL_WORKSPACE" in result.abort_reason

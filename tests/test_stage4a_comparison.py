import pytest

from drawing_robot.execution import ExecutionLog, JointCommand, RobotState, RobotStatus
from drawing_robot.hardware.comparison import compare_sim_to_real, summarize_execution


def make_log(*, hardware, trajectory_id="same"):
    initial = RobotState(0, [0] * 6, [0] * 6, RobotStatus.IDLE)
    log = ExecutionLog(
        trajectory_id, {}, 0, initial,
        hardware_run=hardware,
        safety_profile="stage4a_conservative" if hardware else None,
    )
    first = JointCommand("a", [0] * 6, 0.0)
    second = JointCommand("b", [1, 0, 0, 0, 0, 0], 0.1)
    log.command_sent(first, 10.0)
    log.command_sent(second, 10.12)
    log.record_state(RobotState(10.2, second.target_angles_deg, [0.9, 0, 0, 0, 0, 0], RobotStatus.IDLE, "b"))
    log.finish("completed", 10.2)
    return log


def test_hardware_summary_keeps_planned_sent_and_measured_comparable() -> None:
    summary = summarize_execution(make_log(hardware=True))
    assert summary["max_final_joint_error_deg"] == pytest.approx(0.1)
    assert summary["max_planned_to_sent_lag_s"] == pytest.approx(0.02)


def test_sim_real_comparison_requires_matching_roles_and_trajectory() -> None:
    result = compare_sim_to_real(make_log(hardware=False), make_log(hardware=True))
    assert result["trajectory_id"] == "same"
    with pytest.raises(ValueError, match="same trajectory"):
        compare_sim_to_real(
            make_log(hardware=False), make_log(hardware=True, trajectory_id="other")
        )

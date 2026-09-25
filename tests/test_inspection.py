import pytest

from drawing_robot import ArmConnection, ArmError
from drawing_robot.inspection import compare_fk, inspect_robot


def test_read_only_snapshot_issues_no_control_commands() -> None:
    with ArmConnection(mock=True, read_only=True) as connection:
        raw = connection.mock_backend
        snapshot = inspect_robot(connection)

    assert snapshot.expected_model == "myCobot 280 JN"
    assert len(snapshot.joint_angles_deg) == 6
    assert raw.fresh_mode_commands == []
    assert raw.commands == []
    assert raw.gripper_commands == []
    assert raw.stop_calls == 0


@pytest.mark.parametrize(
    "operation",
    [
        lambda connection: connection.power_on(),
        lambda connection: connection.set_fresh_mode(1),
        lambda connection: connection.send_angles([0, 0, -90, 0, 0, 0], 1),
        lambda connection: connection.send_angle(1, 0, 1),
        lambda connection: connection.set_gripper_value(50),
        lambda connection: connection.stop(),
        lambda connection: connection.release_all_servos(),
        lambda connection: connection.focus_all_servos(),
    ],
)
def test_read_only_connection_refuses_writes(operation) -> None:
    with ArmConnection(mock=True, read_only=True) as connection:
        with pytest.raises(ArmError, match="read-only"):
            operation(connection)


def test_mock_fk_comparison_passes() -> None:
    with ArmConnection(mock=True, read_only=True) as connection:
        snapshot = inspect_robot(connection)
    result = compare_fk(snapshot)
    assert result.passed
    assert result.position_error_norm_mm == pytest.approx(0.0)
    assert result.orientation_error_max_deg == pytest.approx(0.0)

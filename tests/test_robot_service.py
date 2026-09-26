import time

import pytest

from drawing_robot.robot import RobotIO, RobotIOError, RobotMode, RobotService
from drawing_robot.robot.safety import MotionClass, motion_permission
from drawing_robot.robot.state import RobotState


def wait_for_complete_state(service: RobotService, timeout_s: float = 1.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = service.latest_state()
        if len(state.angles_deg) == 6 and len(state.temperatures_c) == 6:
            return state
        time.sleep(0.01)
    raise AssertionError("service did not publish a complete state")


def test_robot_io_read_only_refuses_control() -> None:
    with RobotIO(mock=True, read_only=True) as io:
        assert len(io.call("get_angles")) == 6
        with pytest.raises(RobotIOError, match="read-only"):
            io.call("send_angles", [0] * 6, 5)


def test_robot_io_records_transaction_queue_and_api_timing() -> None:
    with RobotIO(mock=True, read_only=True) as io:
        io.call("get_angles")
        transaction = io.transactions()[-1]
        assert transaction.method == "get_angles"
        assert transaction.queued_at_s <= transaction.io_start_s <= transaction.io_end_s
        assert transaction.status == "ok"


def test_service_publishes_one_state_source() -> None:
    with RobotService(mock=True, read_only=True) as service:
        state = wait_for_complete_state(service)
        assert state.connected is True
        assert state.mode is RobotMode.READY
        assert state.controller_error == 0


def test_complete_refresh_stays_on_service_uart_worker() -> None:
    with RobotService(mock=True, read_only=True, telemetry=False) as service:
        state = service.refresh_state()
        assert len(state.angles_deg) == len(state.temperatures_c) == 6
        assert state.sequence >= 2


def test_service_refuses_motion_until_locally_armed() -> None:
    with RobotService(mock=True, telemetry=False) as service:
        with pytest.raises(RuntimeError, match="DISARMED"):
            service.send_angles([0, 0, -90, 0, 0, 0], 5)
        service.arm_motion(locally_confirmed=True)
        assert service.send_angles([0, 0, -89, 0, 0, 0], 5) == 1


def test_recovery_permission_requires_local_confirmation() -> None:
    with RobotService(mock=True, telemetry=True) as service:
        state = wait_for_complete_state(service)
        decision = motion_permission(
            state, motion_class=MotionClass.RECOVERY, locally_confirmed=False
        )
        assert decision.allowed is False
        assert "local confirmation" in decision.reasons[-1]


def test_disable_torque_requires_mechanical_support() -> None:
    with RobotService(mock=True, telemetry=False) as service:
        with pytest.raises(RuntimeError, match="mechanical support"):
            service.disable_torque()


def test_motion_permission_rejects_incomplete_state() -> None:
    decision = motion_permission(RobotState(connected=True), locally_confirmed=True)
    assert decision.allowed is False
    assert any("telemetry is unavailable" in reason for reason in decision.reasons)

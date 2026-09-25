import pytest

from drawing_robot.motion import MotionExecutor
from drawing_robot.robot import RobotService


def test_executor_keeps_planned_and_commanded_trajectories_separate() -> None:
    with RobotService(mock=True, telemetry=False) as service:
        service.arm_motion(locally_confirmed=True)
        executor = MotionExecutor(service, command_rate_hz=20.0)
        report = executor.execute_joint_plan(
            [[0, 0, -90, 0, 0, 0], [0, 0, -89, 0, 0, 0]],
            [0.0, 0.05],
            firmware_speed=5,
        )
        assert report.planned_waypoints == 2
        assert len(report.commanded) == 2
        assert report.commanded[-1].target_angles_deg[2] == -89
        with pytest.raises(RuntimeError, match="DISARMED"):
            service.send_angles([0, 0, -88, 0, 0, 0], 5)


def test_executor_rejects_plan_faster_than_command_rate() -> None:
    with RobotService(mock=True, telemetry=False) as service:
        executor = MotionExecutor(service, command_rate_hz=4.0)
        with pytest.raises(ValueError, match="command rate"):
            executor.execute_joint_plan(
                [[0, 0, -90, 0, 0, 0], [0, 0, -89, 0, 0, 0]],
                [0.0, 0.1],
                firmware_speed=5,
            )

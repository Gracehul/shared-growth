from dataclasses import replace

import pytest

from drawing_robot.execution import (
    CommandAcknowledgement,
    CommandLatencyConfig,
    ExecutionConfig,
    ExecutionLog,
    ExecutorStatus,
    FaultSettings,
    LatencyMode,
    MotionExecutor,
    RobotState,
    RobotStatus,
    SimRobot,
    SimulationClock,
)
from drawing_robot.stage2 import JointTrajectory, ValidationResult


def trajectory(values=(0, 5, 10), times=(0.0, 0.2, 0.4)):
    return JointTrajectory.from_arrays(times, [[value, 0, 0, 0, 0, 0] for value in values])


def run(config, plan=None, stop_at_s=None):
    plan = plan or trajectory()
    clock = SimulationClock()
    initial_angles = plan.samples[0].positions_deg
    initial = RobotState(0, initial_angles, initial_angles, RobotStatus.IDLE)
    log = ExecutionLog("traj", config.to_dict(), config.random_seed, initial, "run")
    robot = SimRobot(clock, config, initial_angles, log)
    executor = MotionExecutor(robot, clock, config, log)
    result = executor.execute(
        plan, ValidationResult(True), trajectory_id="traj", stop_at_s=stop_at_s
    )
    return result, log, robot, clock


def base_config(**changes):
    config = ExecutionConfig(
        simulation_timestep_s=0.01,
        state_update_rate_hz=50,
        joint_velocity_limits_deg_s=(20,) * 6,
        position_tolerance_deg=0.05,
        state_freshness_timeout_s=0.1,
        trajectory_completion_timeout_s=2.0,
    )
    return replace(config, **changes)


def test_ideal_execution_waits_for_final_target() -> None:
    result, log, _, _ = run(base_config())
    assert result.status is ExecutorStatus.COMPLETED
    assert result.final_state.actual_angles_deg[0] == pytest.approx(10)
    assert log.result["timestamp_s"] > 0.4
    assert [event.kind for event in log.events].count("COMMAND_SENT") == 3
    assert [event.kind for event in log.events].count("COMMAND_ACTIVATED") == 3
    assert log.result["status"] == "completed"


def test_fixed_latency_shifts_activation_predictably() -> None:
    config = base_config(
        command_latency=CommandLatencyConfig(LatencyMode.FIXED, 0.1)
    )
    result, log, _, _ = run(config)
    assert result.status is ExecutorStatus.COMPLETED
    assert [record["effective_delay_s"] for record in log.commands] == pytest.approx(
        [0.1, 0.1, 0.1]
    )


def test_stop_during_motion_has_request_then_application() -> None:
    config = base_config(stop_delay_s=0.15)
    plan = trajectory(values=(0, 30), times=(0.0, 0.1))
    result, log, _, _ = run(config, plan, stop_at_s=0.2)
    assert result.status is ExecutorStatus.STOPPED
    kinds = [event.kind for event in log.events]
    assert kinds.index("STOP_REQUESTED") < kinds.index("STOP_APPLIED")
    applied = next(event for event in log.events if event.kind == "STOP_APPLIED")
    assert applied.data["additional_movement_norm_deg"] > 0


def test_dropped_intermediate_command_is_logged_and_run_remains_defined() -> None:
    config = base_config(faults=FaultSettings(dropped_command_ids=("traj:000001",)))
    result, log, _, _ = run(config)
    assert result.status is ExecutorStatus.COMPLETED
    dropped = [event for event in log.events if event.kind == "COMMAND_DROPPED"]
    assert [event.data["command_id"] for event in dropped] == ["traj:000001"]
    record = next(item for item in log.commands if item["command_id"] == "traj:000001")
    assert record["activation_time_s"] is None
    assert record["dropped"] is True


def test_dropped_final_command_causes_completion_timeout() -> None:
    config = base_config(
        trajectory_completion_timeout_s=0.3,
        faults=FaultSettings(dropped_command_ids=("traj:000002",)),
    )
    result, log, _, _ = run(config)
    assert result.status is ExecutorStatus.FAILED
    assert "completion timeout" in result.message
    assert "TIMEOUT" in [event.kind for event in log.events]


def test_stale_state_is_detected_by_executor() -> None:
    config = base_config(
        state_freshness_timeout_s=0.08,
        faults=FaultSettings(stale_state_at_s=0.15),
    )
    result, log, _, _ = run(config)
    assert result.status is ExecutorStatus.FAILED
    assert result.message == "state freshness timeout"
    assert "STATE_STALE" in [event.kind for event in log.events]


def test_backend_fault_fails_cleanly() -> None:
    config = base_config(faults=FaultSettings(fault_at_s=0.15, fault_message="boom"))
    result, log, _, _ = run(config)
    assert result.status is ExecutorStatus.FAILED
    assert result.message == "boom"
    assert "FAULT" in [event.kind for event in log.events]


class FakeRobot:
    """Deliberately has no SimRobot methods or attributes."""

    def __init__(self, clock):
        self.clock = clock
        self.state = RobotState(0, [0] * 6, [0] * 6, RobotStatus.IDLE)

    def send_joint_command(self, command):
        self.state = RobotState(
            self.clock.now(),
            command.target_angles_deg,
            command.target_angles_deg,
            RobotStatus.IDLE,
            command.command_id,
        )
        return CommandAcknowledgement(True, self.clock.now(), command.command_id)

    def get_state(self):
        return self.state

    def stop_motion(self):
        self.state = RobotState(
            self.clock.now(),
            self.state.commanded_angles_deg,
            self.state.actual_angles_deg,
            RobotStatus.STOPPED,
            self.state.last_command_id,
        )


def test_executor_depends_only_on_robot_interface() -> None:
    clock = SimulationClock()
    config = base_config()
    fake = FakeRobot(clock)
    executor = MotionExecutor(fake, clock, config)
    plan = JointTrajectory.from_arrays([0.0], [[0] * 6])
    result = executor.execute(plan, ValidationResult(True), trajectory_id="fake")
    assert result.status is ExecutorStatus.COMPLETED


def test_deterministic_rerun_reproduces_events_and_final_state() -> None:
    config = base_config(
        command_latency=CommandLatencyConfig(
            LatencyMode.JITTERED, 0.08, -0.02, 0.02
        ),
        random_seed=1234,
    )
    first_result, first_log, _, _ = run(config)
    second_result, second_log, _, _ = run(config)
    first_events = [
        (event.timestamp_s, event.kind, event.data) for event in first_log.events
    ]
    second_events = [
        (event.timestamp_s, event.kind, event.data) for event in second_log.events
    ]
    assert first_events == second_events
    assert first_result.final_state == second_result.final_state
    assert first_log.commands == second_log.commands


def test_events_at_same_timestamp_have_stable_order() -> None:
    _, log, _, _ = run(base_config())
    at_second_command = [
        event.kind for event in log.events if event.timestamp_s == pytest.approx(0.2)
    ]
    assert at_second_command.index("COMMAND_SENT") < at_second_command.index(
        "COMMAND_ACTIVATED"
    )
    assert at_second_command.index("COMMAND_ACTIVATED") < at_second_command.index(
        "STATE_PUBLISHED"
    )

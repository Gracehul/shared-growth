import numpy as np
import pytest

from drawing_robot.execution import (
    CommandLatencyConfig,
    ExecutionConfig,
    ExecutionLog,
    JointCommand,
    LatencyMode,
    RobotState,
    RobotStatus,
    SimRobot,
    SimulationClock,
)


def make_robot(config=None):
    config = config or ExecutionConfig(
        simulation_timestep_s=0.1,
        state_update_rate_hz=10,
        joint_velocity_limits_deg_s=(10,) * 6,
    )
    clock = SimulationClock()
    initial = RobotState(0, [0] * 6, [0] * 6, RobotStatus.IDLE)
    log = ExecutionLog("test", config.to_dict(), config.random_seed, initial, "run")
    return clock, SimRobot(clock, config, [0] * 6, log), log


def test_rate_limited_motion_does_not_overshoot() -> None:
    clock, robot, log = make_robot()
    command = JointCommand("c1", [1.5, -1.5, 0, 0, 0, 0], 0)
    log.command_sent(command, 0)
    assert robot.send_joint_command(command).accepted
    clock.advance(0.1)
    assert robot.actual_angles_deg[:2] == pytest.approx((1.0, -1.0))
    clock.advance(0.1)
    assert robot.actual_angles_deg[:2] == pytest.approx((1.5, -1.5))
    assert robot.get_state().status is RobotStatus.IDLE


def test_fixed_latency_delays_activation() -> None:
    config = ExecutionConfig(
        simulation_timestep_s=0.05,
        state_update_rate_hz=20,
        command_latency=CommandLatencyConfig(LatencyMode.FIXED, 0.1),
    )
    clock, robot, log = make_robot(config)
    command = JointCommand("c1", [1, 0, 0, 0, 0, 0], 0)
    log.command_sent(command, 0)
    robot.send_joint_command(command)
    clock.advance(0.05)
    assert robot.get_state().last_command_id is None
    clock.advance(0.05)
    assert robot.get_state().last_command_id == "c1"
    assert log.commands[0]["effective_delay_s"] == pytest.approx(0.1)


def test_latency_mode_string_is_normalized() -> None:
    latency = CommandLatencyConfig("fixed", 0.1)
    assert latency.mode is LatencyMode.FIXED


def test_stop_delay_allows_bounded_additional_motion_then_freezes() -> None:
    config = ExecutionConfig(
        simulation_timestep_s=0.1,
        state_update_rate_hz=10,
        joint_velocity_limits_deg_s=(10,) * 6,
        stop_delay_s=0.2,
    )
    clock, robot, log = make_robot(config)
    command = JointCommand("c1", [20, 0, 0, 0, 0, 0], 0)
    log.command_sent(command, 0)
    robot.send_joint_command(command)
    clock.advance(0.1)
    before = robot.actual_angles_deg[0]
    robot.stop_motion()
    assert robot._status is RobotStatus.STOPPING
    clock.advance(0.1)
    clock.advance(0.1)
    stopped = robot.actual_angles_deg[0]
    assert stopped > before
    assert robot.get_state().status is RobotStatus.STOPPED
    clock.advance(0.5)
    assert robot.actual_angles_deg[0] == pytest.approx(stopped)
    event = next(event for event in log.events if event.kind == "STOP_APPLIED")
    assert event.data["additional_movement_norm_deg"] == pytest.approx(2.0)
    rejected = robot.send_joint_command(JointCommand("late", [0] * 6, 1.0))
    assert not rejected.accepted


def test_seeded_jitter_is_reproducible() -> None:
    def delays():
        config = ExecutionConfig(
            simulation_timestep_s=0.01,
            state_update_rate_hz=100,
            command_latency=CommandLatencyConfig(
                LatencyMode.JITTERED, 0.1, -0.02, 0.02
            ),
            random_seed=42,
        )
        clock, robot, log = make_robot(config)
        for index in range(3):
            command = JointCommand(f"c{index}", [index] * 6, index * 0.1)
            log.command_sent(command, clock.now())
            robot.send_joint_command(command)
            clock.advance(0.2)
        return [record["effective_delay_s"] for record in log.commands]

    assert delays() == delays()


def test_simulation_timestep_and_publish_rate_are_independent() -> None:
    config = ExecutionConfig(simulation_timestep_s=0.01, state_update_rate_hz=5)
    clock, robot, _ = make_robot(config)
    original = robot.get_state().timestamp_s
    for _ in range(19):
        clock.advance(0.01)
    assert robot.get_state().timestamp_s == original
    clock.advance(0.01)
    assert robot.get_state().timestamp_s == pytest.approx(0.2)

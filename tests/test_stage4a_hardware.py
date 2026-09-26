import time
from dataclasses import replace

import pytest

from drawing_robot.execution import (
    ExecutionConfig,
    ExecutionLog,
    JointCommand,
    MotionExecutor,
    RobotInterface,
    RobotState as ExecutionState,
    RobotStatus,
    SimulationClock,
)
from drawing_robot.hardware import (
    LifecycleState,
    MyCobotRobot,
    PreflightRejected,
    Stage4AConfig,
    motion_envelope_gate,
    verify_park,
    verify_ready,
)
from drawing_robot.robot.state import RobotMode, RobotState
from drawing_robot.stage2 import JointTrajectory, ValidationResult


class FakeIO:
    def transactions(self, since=0):
        return ()


class FakeService:
    backend_class_name = "FakeMyCobot"
    port = "/dev/fake"
    baudrate = 1_000_000

    def __init__(self, state=None):
        self.state = state or complete_state()
        self.io = FakeIO()
        self.armed = False
        self.stop_calls = 0
        self.commands = []

    def latest_state(self):
        return self.state

    def arm_motion(self, *, locally_confirmed):
        assert locally_confirmed
        self.armed = True
        return self.state

    def disarm_motion(self):
        self.armed = False

    def send_angles(self, angles, speed):
        self.commands.append((tuple(angles), speed))
        self.state = replace(
            self.state,
            monotonic_s=time.monotonic(),
            sequence=self.state.sequence + 1,
            angles_deg=tuple(angles),
            speeds=(0.0,) * 6,
        )
        return 1

    def stop_motion(self):
        self.stop_calls += 1
        return 1

    def refresh_critical_state(self):
        self.state = replace(
            self.state,
            monotonic_s=time.monotonic(),
            critical_monotonic_s=time.monotonic(),
            sequence=self.state.sequence + 1,
        )
        return self.state


def complete_state(**changes):
    state = RobotState(
        monotonic_s=time.monotonic(),
        sequence=1,
        mode=RobotMode.READY,
        connected=True,
        powered=True,
        servos_enabled=True,
        angles_deg=(0, 0, -90, 0, 0, 0),
        speeds=(0,) * 6,
        temperatures_c=(35,) * 6,
        voltages_v=(12,) * 6,
        servo_status=(0,) * 6,
        controller_error=0,
    )
    return replace(state, **changes)


def plan(values=(0, 1), times=(0.0, 0.1)):
    return JointTrajectory.from_arrays(
        times,
        [[value, 0, -90, 0, 0, 0] for value in values],
    )


def hardware_config():
    return Stage4AConfig(allowed_workspace_mm=((-300, 300), (-300, 300), (0, 400)))


def arm(robot, trajectory=None):
    robot.preflight(
        trajectory or plan(),
        ValidationResult(True),
        operator_supervising=True,
        physical_stop_accessible=True,
        workspace_clear=True,
    )


def test_mycobot_robot_satisfies_stage3_contract() -> None:
    assert isinstance(MyCobotRobot(FakeService(), hardware_config()), RobotInterface)


def test_unknown_temperature_blocks_preflight() -> None:
    robot = MyCobotRobot(FakeService(complete_state(temperatures_c=())), hardware_config())
    with pytest.raises(PreflightRejected, match="TEMPERATURE_UNKNOWN"):
        arm(robot)


def test_operator_and_physical_clearance_are_explicit_gates() -> None:
    robot = MyCobotRobot(FakeService(), hardware_config())
    with pytest.raises(PreflightRejected, match="OPERATOR_REQUIRED"):
        robot.preflight(
            plan(), ValidationResult(True), operator_supervising=False,
            physical_stop_accessible=True, workspace_clear=True,
        )


def test_conservative_motion_envelope_rejects_large_step() -> None:
    decision = motion_envelope_gate(plan((0, 3)), Stage4AConfig())
    assert not decision.allowed
    assert "STAGE4A_JOINT_STEP_EXCEEDED" in decision.reasons


def test_real_cartesian_envelope_requires_explicit_workspace() -> None:
    from drawing_robot.stage2 import CartesianTrajectory
    cartesian = CartesianTrajectory.from_arrays([0.0, 0.1], [[0, 0, 100, 0, 0, 0]] * 2)
    decision = motion_envelope_gate(
        plan(), replace(Stage4AConfig(), allowed_workspace_mm=None), cartesian
    )
    assert "STAGE4A_WORKSPACE_UNCONFIGURED" in decision.reasons


def test_command_records_real_api_timing_and_measured_state() -> None:
    service = FakeService()
    initial = ExecutionState(0, service.state.angles_deg, service.state.angles_deg, RobotStatus.IDLE)
    log = ExecutionLog("hardware", {}, 0, initial, hardware_run=True, safety_profile="stage4a_conservative")
    robot = MyCobotRobot(service, hardware_config(), execution_log=log)
    arm(robot)
    command = JointCommand("cmd-1", (1, 0, -90, 0, 0, 0), 0.1)
    log.command_sent(command, time.monotonic())
    acknowledgement = robot.send_joint_command(command)
    state = robot.get_state()
    assert acknowledgement.accepted
    assert state.actual_angles_deg[0] == 1
    assert state.is_fresh is True
    assert log.commands[0]["api_duration_s"] >= 0
    assert log.commands[0]["observed_motion_onset_s"] is None


def test_stop_disarms_and_rejects_future_commands() -> None:
    service = FakeService()
    robot = MyCobotRobot(service, hardware_config())
    arm(robot)
    robot.stop_motion()
    acknowledgement = robot.send_joint_command(JointCommand("later", (1, 0, -90, 0, 0, 0), 0.1))
    assert not acknowledgement.accepted
    assert service.stop_calls == 1


def test_stop_confirmation_uses_successive_angle_samples_without_speeds() -> None:
    service = FakeService(complete_state(speeds=()))
    config = replace(hardware_config(), stopped_samples_required=1)
    robot = MyCobotRobot(service, config)
    arm(robot)
    robot.stop_motion()
    assert robot.get_state().status is RobotStatus.STOPPED


def test_ready_and_park_are_blocked_until_configured() -> None:
    state = complete_state()
    config = replace(Stage4AConfig(), ready_angles_deg=None)
    assert verify_ready(state, config).state is LifecycleState.READY_UNCONFIRMED
    assert verify_park(state, Stage4AConfig()).state is LifecycleState.PARK_BLOCKED


def test_stage3_executor_runs_unchanged_against_real_adapter_contract() -> None:
    service = FakeService()
    robot = MyCobotRobot(service, hardware_config())
    trajectory = plan()
    arm(robot, trajectory)
    config = ExecutionConfig(
        simulation_timestep_s=0.01,
        state_update_rate_hz=20,
        position_tolerance_deg=0.1,
        state_freshness_timeout_s=1.0,
        trajectory_completion_timeout_s=1.0,
    )
    result = MotionExecutor(robot, SimulationClock(), config).execute(
        trajectory, ValidationResult(True), trajectory_id="contract"
    )
    assert result.status.value == "COMPLETED"
    assert len(service.commands) == 2

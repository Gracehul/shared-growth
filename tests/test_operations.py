import ast
from pathlib import Path

from drawing_robot.execution import (
    CommandAcknowledgement,
    ExecutionConfig,
    RobotState,
    RobotStatus,
    SimRobot,
    SimulationClock,
)
from drawing_robot.hardware import GateCategory, GateStatus
from drawing_robot.hardware.policy import hard_result, performance_result
from drawing_robot.operations import (
    EvidenceStatus,
    InteractionMode,
    Operation,
    OperationController,
    OperationDecision,
    OperationRequest,
    PreparedTrajectory,
)
from drawing_robot.stage2 import JointTrajectory, ValidationResult


def prepared(identifier="demo", *, valid=True, perception_id=None, intent_id=None):
    return PreparedTrajectory(
        identifier,
        JointTrajectory.from_arrays(
            [0.0, 0.1],
            [[0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0]],
        ),
        ValidationResult(valid),
        perception_id,
        intent_id,
    )


def pass_gates():
    return (hard_result("runtime", GateStatus.PASS),)


def controller(mode=InteractionMode.MANUAL, items=None, gates=pass_gates):
    clock = SimulationClock()
    config = ExecutionConfig(
        simulation_timestep_s=0.01,
        state_update_rate_hz=50,
        joint_velocity_limits_deg_s=(20,) * 6,
        position_tolerance_deg=0.05,
        trajectory_completion_timeout_s=1.0,
    )
    robot = SimRobot(clock, config, [0] * 6)
    items = items or {"demo": prepared()}
    return OperationController(
        robot,
        clock,
        config,
        items,
        gate_provider=gates,
        ready_trajectory_id="demo",
        mode=mode,
    )


def request(identifier, mode, operation, trajectory_id=None, **values):
    return OperationRequest(
        identifier, mode, operation, trajectory_id=trajectory_id, **values
    )


def test_observe_rejects_motion_request() -> None:
    receipt = controller(InteractionMode.OBSERVE).request(
        request("observe-1", InteractionMode.OBSERVE, Operation.GO_READY)
    )
    assert not receipt.accepted
    assert receipt.reason == "MODE_PERMISSION_DENIED"
    assert receipt.run_id is None


def test_manual_accepts_valid_prepared_trajectory_and_traces_ids() -> None:
    receipt = controller().request(
        request(
            "manual-1",
            InteractionMode.MANUAL,
            Operation.EXECUTE_TRAJECTORY,
            "demo",
            perception_id="perception-7",
            intent_id="intent-9",
        )
    )
    assert receipt.accepted
    assert receipt.state == "COMPLETED"
    assert receipt.run_id
    assert receipt.evidence.command_ids == ("demo:000000", "demo:000001")
    assert receipt.evidence.perception_id == "perception-7"
    assert receipt.evidence.intent_id == "intent-9"
    assert receipt.evidence.actual is EvidenceStatus.PRESENT
    assert receipt.latest_execution_event["kind"] == "RUN_COMPLETED"


def test_manual_rejects_invalid_or_unvalidated_trajectory() -> None:
    invalid = prepared("invalid", valid=False)
    receipt = controller(items={"invalid": invalid}).request(
        request(
            "manual-invalid",
            InteractionMode.MANUAL,
            Operation.EXECUTE_TRAJECTORY,
            "invalid",
        )
    )
    assert not receipt.accepted
    assert receipt.reason == "TRAJECTORY_REJECTED"
    validation = next(
        result
        for result in receipt.gate_results
        if result.name == "trajectory_validity"
    )
    assert validation.category is GateCategory.HARD
    assert validation.status is GateStatus.BLOCK


def test_go_ready_resolves_only_the_prepared_ready_trajectory() -> None:
    receipt = controller().request(
        request("ready-1", InteractionMode.MANUAL, Operation.GO_READY)
    )
    assert receipt.accepted
    assert receipt.state == "COMPLETED"
    assert receipt.evidence.command_ids[0].startswith("demo:")


def test_shared_growth_session_request_is_distinguishable_and_has_no_fake_evidence() -> (
    None
):
    receipt = controller(InteractionMode.SHARED_GROWTH).request(
        request(
            "shared-1",
            InteractionMode.SHARED_GROWTH,
            Operation.START_SHARED_GROWTH,
            session_id="session-1",
        )
    )
    assert receipt.accepted
    assert receipt.state == "SHARED_GROWTH_ACTIVE"
    assert receipt.session_id == "session-1"
    assert receipt.run_id is None
    assert receipt.evidence.perception is EvidenceStatus.ABSENT
    assert receipt.evidence.intended is EvidenceStatus.ABSENT
    assert receipt.evidence.commanded is EvidenceStatus.ABSENT
    assert receipt.evidence.actual is EvidenceStatus.UNKNOWN


def test_shared_growth_direct_trajectory_request_is_internal_only() -> None:
    receipt = controller(InteractionMode.SHARED_GROWTH).request(
        request(
            "shared-direct",
            InteractionMode.SHARED_GROWTH,
            Operation.EXECUTE_TRAJECTORY,
            "demo",
        )
    )
    assert not receipt.accepted
    assert receipt.reason == "MODE_PERMISSION_DENIED"


def test_hard_block_rejects_operation_authoritatively() -> None:
    blocked = lambda: (
        hard_result(
            "temperature",
            GateStatus.BLOCK,
            value=61,
            limit=60,
            unit="degC",
            reason="TEMPERATURE_ABORT: threshold reached",
        ),
    )
    receipt = controller(gates=blocked).request(
        request(
            "blocked-1",
            InteractionMode.MANUAL,
            Operation.EXECUTE_TRAJECTORY,
            "demo",
        )
    )
    assert not receipt.accepted
    assert receipt.reason == "TEMPERATURE_ABORT"
    assert receipt.gate_results[-1].value == 61


def test_performance_degradation_is_accepted_with_warning_and_attributable() -> None:
    performance = lambda: (
        performance_result(
            "ready_error",
            GateStatus.DEGRADED,
            value=0.79,
            limit=0.75,
            unit="deg",
            reason="READY_TARGET_MISSED",
        ),
    )
    receipt = controller(gates=performance).request(
        request("ready-performance", InteractionMode.MANUAL, Operation.GO_READY)
    )
    assert receipt.accepted
    assert receipt.decision is OperationDecision.ACCEPTED_WITH_WARNING
    assert receipt.state == "COMPLETED"
    gate = receipt.gate_results[-1]
    assert gate.category is GateCategory.PERFORMANCE
    assert gate.value == 0.79
    assert gate.limit == 0.75


def test_hard_only_admission_keeps_performance_warning_non_blocking() -> None:
    warning = lambda: (
        performance_result(
            "thermal_margin",
            GateStatus.WARN,
            value=56,
            limit=55,
            unit="degC",
            reason="TEMPERATURE_WARNING",
        ),
    )
    receipt = controller(gates=warning).request(
        request("warning", InteractionMode.MANUAL, Operation.GO_READY)
    )
    assert receipt.accepted
    assert receipt.decision is OperationDecision.ACCEPTED_WITH_WARNING
    assert receipt.run_id is not None


class StopBackend:
    def __init__(self):
        self.stop_calls = 0
        self.state = RobotState(2.0, [0] * 6, [0] * 6, RobotStatus.IDLE)

    def send_joint_command(self, command):
        return CommandAcknowledgement(True, 2.0, command.command_id)

    def get_state(self):
        return self.state

    def stop_motion(self):
        self.stop_calls += 1
        self.state = RobotState(2.1, [0] * 6, [0] * 6, RobotStatus.STOPPED)


def test_stop_is_global_and_uses_existing_robot_interface_path() -> None:
    backend = StopBackend()
    control = OperationController(
        backend,
        SimulationClock(2.0),
        ExecutionConfig(),
        {},
        gate_provider=None,
        mode=InteractionMode.OBSERVE,
    )
    receipt = control.request(
        request("stop-1", InteractionMode.OBSERVE, Operation.STOP)
    )
    assert receipt.accepted
    assert receipt.state == "STOPPED"
    assert backend.stop_calls == 1
    assert receipt.evidence.robot_state_timestamp_s == 2.1


def test_command_contract_has_no_direct_hardware_dependency() -> None:
    path = Path(__file__).parents[1] / "drawing_robot" / "operations.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    imported = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported.append(node.module or "")
    text = path.read_text(encoding="utf-8")
    assert not any(
        name.startswith(("pymycobot", "serial", "drawing_robot.robot.io"))
        for name in imported
    )
    assert "RobotIO" not in text
    assert "send_angles" not in text
    assert "release_all_servos" not in text


def test_backend_operation_gates_run_before_executor_and_remain_authoritative() -> None:
    calls = []
    clock = SimulationClock()
    config = ExecutionConfig(
        simulation_timestep_s=0.01,
        state_update_rate_hz=50,
        joint_velocity_limits_deg_s=(20,) * 6,
        position_tolerance_deg=0.05,
        trajectory_completion_timeout_s=1.0,
    )
    robot = SimRobot(clock, config, [0] * 6)
    control = OperationController(
        robot,
        clock,
        config,
        {"demo": prepared()},
        gate_provider=pass_gates,
        mode=InteractionMode.MANUAL,
        operation_gate_provider=lambda item: (
            calls.append(item.trajectory_id)
            or hard_result(
                "backend_preflight",
                GateStatus.BLOCK,
                reason="BACKEND_PREFLIGHT_BLOCKED",
            ),
        ),
    )
    receipt = control.request(
        request(
            "backend-block",
            InteractionMode.MANUAL,
            Operation.EXECUTE_TRAJECTORY,
            "demo",
        )
    )
    assert calls == ["demo"]
    assert not receipt.accepted
    assert receipt.reason == "BACKEND_PREFLIGHT_BLOCKED"
    assert receipt.evidence.command_ids == ()


def test_execution_log_can_be_attached_to_existing_backend_adapter() -> None:
    logs = []
    clock = SimulationClock()
    config = ExecutionConfig(
        simulation_timestep_s=0.01,
        state_update_rate_hz=50,
        joint_velocity_limits_deg_s=(20,) * 6,
        position_tolerance_deg=0.05,
        trajectory_completion_timeout_s=1.0,
    )
    robot = SimRobot(clock, config, [0] * 6)
    control = OperationController(
        robot,
        clock,
        config,
        {"demo": prepared()},
        gate_provider=pass_gates,
        mode=InteractionMode.MANUAL,
        execution_log_sink=logs.append,
    )
    receipt = control.request(
        request("logged", InteractionMode.MANUAL, Operation.EXECUTE_TRAJECTORY, "demo")
    )
    assert receipt.state == "COMPLETED"
    assert len(logs) == 1
    assert logs[0].run_id == receipt.run_id

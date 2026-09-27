import time
from dataclasses import replace

import pytest

from drawing_robot.execution import (
    ExecutionConfig,
    ExecutionLog,
    JointCommand,
    RobotState as ExecutionRobotState,
    RobotStatus,
)
from drawing_robot.hardware import (
    ExecutionPhase,
    GateCategory,
    GateStatus,
    Stage4AConfig,
    SystemSnapshotAdapter,
    analyze_execution_log,
    runtime_gate,
    verify_ready,
)
from drawing_robot.hardware.telemetry import execution_status
from drawing_robot.robot.state import RobotMode, RobotState
from drawing_robot.stage2 import ValidationResult


def service_state(**changes):
    now = 10.0
    state = RobotState(
        monotonic_s=now,
        critical_monotonic_s=now,
        sequence=4,
        mode=RobotMode.READY,
        connected=True,
        powered=True,
        servos_enabled=True,
        angles_deg=(1.0, 50.0, -70.0, -63.0, -3.0, 10.0),
        temperatures_c=(35.0,) * 6,
        servo_status=(0,) * 6,
        controller_error=0,
    )
    return replace(state, **changes)


def test_hard_gate_blocks_and_unknown_is_not_pass() -> None:
    config = Stage4AConfig()
    hot = runtime_gate(
        service_state(temperatures_c=(35, 35, 35, 61, 35, 35)),
        config,
        now_s=10.1,
    )
    temperature = next(result for result in hot.results if result.name == "temperature")
    assert temperature.category is GateCategory.HARD
    assert temperature.status is GateStatus.BLOCK
    assert not hot.allowed

    unknown = runtime_gate(
        service_state(controller_error=None), config, now_s=10.1
    )
    controller = next(result for result in unknown.results if result.name == "controller_fault")
    assert controller.status is GateStatus.UNKNOWN
    assert not unknown.allowed


def test_ready_miss_is_performance_not_controller_fault() -> None:
    config = replace(
        Stage4AConfig(),
        ready_angles_deg=(0, 50, -70, -63, -3, 10),
        ready_tolerance_deg=0.75,
    )
    result = verify_ready(service_state(), config)
    assert not result.confirmed
    assert result.gate_result.category is GateCategory.PERFORMANCE
    assert result.gate_result.status is GateStatus.BLOCK
    assert result.gate_result.value == pytest.approx(1.0)
    assert result.gate_result.limit == pytest.approx(0.75)


def test_threshold_metadata_keeps_values_and_provenance() -> None:
    thresholds = Stage4AConfig().threshold_metadata()
    assert thresholds["ready_tolerance_deg"].value == 0.75
    assert thresholds["completion_tolerance_deg"].value == 0.5
    assert thresholds["telemetry_freshness_timeout_s"].value == 1.0
    assert thresholds["temperature_abort_c"].value == 60.0
    assert thresholds["ready_tolerance_deg"].status == "under characterization"


def test_snapshot_aggregates_authoritative_state_and_policy() -> None:
    state = service_state()
    config = replace(
        Stage4AConfig(),
        ready_angles_deg=(0, 50, -70, -63, -3, 10),
    )
    adapter = SystemSnapshotAdapter(
        lambda: state,
        config=config,
        validation_provider=lambda: ValidationResult(True),
        target_provider=lambda: (0, 50, -70, -63, -3, 10),
        phase_provider=lambda: ExecutionPhase.READY_CHECK,
        now_provider=lambda: 10.2,
    )
    snapshot = adapter.get_system_snapshot()
    assert snapshot.q_actual_deg is state.angles_deg
    assert snapshot.q_target_deg == (0, 50, -70, -63, -3, 10)
    assert snapshot.joint_error_deg[0] == pytest.approx(-1.0)
    assert snapshot.execution_phase is ExecutionPhase.READY_CHECK
    ready = next(result for result in snapshot.gate_results if result.name == "ready_error")
    assert ready.category is GateCategory.PERFORMANCE
    assert ready.status is GateStatus.BLOCK
    assert all(not hasattr(adapter, name) for name in ("send_angles", "robot_io", "uart"))


def test_stopped_requires_measured_confirmation() -> None:
    state = service_state(mode=RobotMode.STOPPING)
    assert execution_status(
        state, moving=False, stop_requested=True, stop_confirmed=False
    ) is RobotStatus.STOPPING
    assert execution_status(
        state, moving=False, stop_requested=True, stop_confirmed=True
    ) is RobotStatus.STOPPED


def test_logged_stop_without_confirmation_remains_unknown() -> None:
    initial = ExecutionRobotState(0, [0] * 6, [0] * 6, RobotStatus.IDLE)
    log = ExecutionLog("stop", ExecutionConfig().to_dict(), 0, initial, "run-stop")
    log.add_event("STOP_REQUESTED", 0.1)
    log.finish(
        "failed",
        0.2,
        stop_confirmed=False,
        robot_motion_unknown=True,
    )
    adapter = SystemSnapshotAdapter(log_provider=lambda: log, now_provider=lambda: 0.2)
    snapshot = adapter.get_system_snapshot()
    assert snapshot.execution_phase is ExecutionPhase.FAILED
    assert snapshot.execution_phase is not ExecutionPhase.STOPPED


def test_log_evidence_computes_tracking_freshness_settling_and_uart() -> None:
    initial = ExecutionRobotState(0, [0] * 6, [0] * 6, RobotStatus.IDLE, state_age_s=0.01)
    log = ExecutionLog("probe", ExecutionConfig().to_dict(), 0, initial, "evidence")
    command = JointCommand("probe:0", [1, 0, 0, 0, 0, 0], 0)
    log.command_sent(command, 1.0)
    log.commands[-1]["send_end_s"] = 1.02
    log.record_state(
        ExecutionRobotState(
            1.1,
            [1, 0, 0, 0, 0, 0],
            [0.8, 0, 0, 0, 0, 0],
            RobotStatus.MOVING,
            state_age_s=0.02,
        )
    )
    log.record_phase(
        "single_joint_probe",
        "PASS",
        "GO",
        final_error_deg_per_joint=[-0.2, 0, 0, 0, 0, 0],
    )
    log.record_uart_transaction(
        {
            "method": "get_angles",
            "queued_at_s": 1.0,
            "io_start_s": 1.01,
            "io_end_s": 1.03,
        }
    )
    log.finish("passed", 1.2)
    evidence = analyze_execution_log(log)
    assert evidence.run_id == "evidence"
    assert evidence.trajectory_id == "probe"
    assert evidence.schema_version == log.schema_version
    assert evidence.probe_endpoint_error_deg == pytest.approx(0.2)
    assert evidence.max_tracking_error_deg == pytest.approx(0.2)
    assert evidence.max_state_age_s == pytest.approx(0.02)
    assert evidence.settling_duration_s == pytest.approx(0.18)
    assert evidence.uart_by_operation["get_angles"].mean_duration_ms == pytest.approx(20)


def test_evidence_retains_hardware_and_telemetry_profile_attribution() -> None:
    initial = ExecutionRobotState(0, [0] * 6, [0] * 6, RobotStatus.IDLE)
    log = ExecutionLog(
        "hardware-probe",
        {
            "profile": "stage4a_conservative",
            "critical_telemetry_hz": 5.0,
            "temperature_telemetry_hz": 1.0,
            "telemetry_freshness_timeout_s": 1.0,
        },
        0,
        initial,
        "source-run",
        hardware_run=True,
        safety_profile="stage4a_conservative",
        provenance={"git_commit": "abc123"},
    )
    evidence = analyze_execution_log(log)
    assert evidence.hardware_run is True
    assert evidence.safety_profile == "stage4a_conservative"
    assert evidence.telemetry_profile["critical_telemetry_hz"] == 5.0
    assert evidence.provenance == {"git_commit": "abc123"}


def test_failed_log_does_not_invent_settling_duration() -> None:
    initial = ExecutionRobotState(0, [0] * 6, [0] * 6, RobotStatus.IDLE)
    log = ExecutionLog("failed", {}, 0, initial)
    command = JointCommand("failed:0", [1] * 6, 0)
    log.command_sent(command, 1.0)
    log.finish("failed", 3.0, reason="communication lost")
    assert analyze_execution_log(log).settling_duration_s is None


def test_ui_adapter_reads_latest_event_from_log_without_hardware_access() -> None:
    initial = ExecutionRobotState(0, [0] * 6, [0] * 6, RobotStatus.IDLE)
    log = ExecutionLog("offline", {}, 0, initial, "ui-run")
    log.add_event("RUN_FAILED", 1.0, reason="offline fixture")
    log.finish("failed", 1.0)
    adapter = SystemSnapshotAdapter(log_provider=log, now_provider=lambda: 1.0)
    assert adapter.get_latest_execution_event()["kind"] == "RUN_FAILED"
    assert adapter.get_system_snapshot().run_id == "ui-run"

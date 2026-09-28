#!/usr/bin/env python3
"""Run the Robot Interaction Workstation with a sim or guarded hardware backend."""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from drawing_robot.execution import (
    ExecutionConfig,
    RobotStatus,
    SimRobot,
    SimulationClock,
    WallClock,
)
from drawing_robot.hardware import (
    ExecutionPhase,
    GateResult,
    GateStatus,
    MyCobotRobot,
    PreflightRejected,
    Stage4AConfig,
    SystemSnapshotAdapter,
)
from drawing_robot.hardware.policy import hard_result
from drawing_robot.hardware.read_only_validation import validate_read_only_session
from drawing_robot.kinematics import pose_coords
from drawing_robot.operations import (
    InteractionMode,
    Operation,
    OperationController,
    PreparedTrajectory,
)
from drawing_robot.robot import RobotService
from drawing_robot.robot.state import RobotMode, RobotState as ServiceRobotState
from drawing_robot.stage2 import (
    CartesianTrajectory,
    JointTrajectory,
    LimitValue,
    Stage2Config,
    TrajectoryValidator,
    ValidationResult,
    WorkspaceBounds,
)
from drawing_robot.workstation import (
    WorkstationApplication,
    WorkstationOperation,
    create_workstation_server,
)

ROOT = Path(__file__).resolve().parents[1]
STATIC_DIRECTORY = ROOT / "frontend" / "robot-interaction-workstation"


def _mode(status: RobotStatus) -> RobotMode:
    return {
        RobotStatus.IDLE: RobotMode.READY,
        RobotStatus.MOVING: RobotMode.EXECUTING,
        RobotStatus.STOPPING: RobotMode.STOPPING,
        RobotStatus.STOPPED: RobotMode.PARKED,
        RobotStatus.FAULT: RobotMode.FAULT,
    }[status]


def build_offline_application() -> WorkstationApplication:
    """Compose real core contracts with SimRobot; no hardware is opened."""
    clock = SimulationClock(1.0)
    config = ExecutionConfig(
        simulation_timestep_s=0.01,
        state_update_rate_hz=20.0,
        joint_velocity_limits_deg_s=(15.0,) * 6,
        position_tolerance_deg=0.1,
        trajectory_completion_timeout_s=5.0,
    )
    robot = SimRobot(clock, config, [0.0] * 6)
    validation = ValidationResult(
        True, metrics={"source": "offline workstation fixture"}
    )
    ready = PreparedTrajectory(
        "ready_offline",
        JointTrajectory.from_arrays([0.0, 0.5], [[0.0] * 6, [0.0] * 6]),
        validation,
    )
    demo = PreparedTrajectory(
        "demo_phrase_01",
        JointTrajectory.from_arrays(
            [0.0, 0.5, 1.0, 1.5],
            [
                [0, 0, 0, 0, 0, 0],
                [1, 0.5, 0, 0, 0, 0.5],
                [1.5, 0.5, -0.5, 0, 0.5, 1],
                [1, 0, 0, 0, 0, 0.5],
            ],
        ),
        validation,
    )

    def service_state() -> ServiceRobotState:
        measured = robot.get_state()
        return ServiceRobotState(
            timestamp_utc=datetime.now(timezone.utc).isoformat(),
            monotonic_s=clock.now(),
            critical_monotonic_s=clock.now(),
            sequence=int(round(clock.now() * 100)),
            mode=_mode(measured.status),
            connected=True,
            powered=True,
            servos_enabled=True,
            angles_deg=tuple(measured.actual_angles_deg),
            temperatures_c=(35.0,) * 6,
            servo_status=(0,) * 6,
            controller_error=0,
            fault=measured.fault,
        )

    def phase():
        return {
            RobotStatus.IDLE: ExecutionPhase.IDLE,
            RobotStatus.MOVING: ExecutionPhase.RUNNING,
            RobotStatus.STOPPING: ExecutionPhase.STOPPING,
            RobotStatus.STOPPED: ExecutionPhase.STOPPED,
            RobotStatus.FAULT: ExecutionPhase.FAILED,
        }[robot.get_state().status]

    snapshot = SystemSnapshotAdapter(
        service_state,
        validation_provider=lambda: validation,
        target_provider=lambda: tuple(robot.get_state().commanded_angles_deg),
        phase_provider=phase,
        now_provider=clock.now,
    )
    controller = OperationController(
        robot,
        clock,
        config,
        {ready.trajectory_id: ready, demo.trajectory_id: demo},
        gate_provider=snapshot.get_gate_results,
        ready_trajectory_id=ready.trajectory_id,
        mode=InteractionMode.OBSERVE,
    )
    operations = (
        WorkstationOperation(
            "ready",
            "GO TO READY",
            Operation.GO_READY,
            description="Prepared offline READY operation",
        ),
        WorkstationOperation(
            "demo",
            "TEST MOTION",
            Operation.EXECUTE_TRAJECTORY,
            trajectory_id=demo.trajectory_id,
            description="Stage-2 validation fixture through MotionExecutor",
        ),
        WorkstationOperation(
            "shared-growth",
            "START SHARED GROWTH",
            Operation.START_SHARED_GROWTH,
            description="Session boundary only; sensing and mapping remain absent",
        ),
    )
    return WorkstationApplication(
        snapshot, controller, operations, backend_label="SimRobot · offline core"
    )


def _validator(config: Stage4AConfig) -> TrajectoryValidator:
    bounds = config.allowed_workspace_mm
    if bounds is None:
        raise RuntimeError("Stage-4A workspace is not configured")
    stage2 = replace(
        Stage2Config(),
        workspace_bounds=LimitValue(
            WorkspaceBounds(bounds[0], bounds[1], bounds[2]),
            "mm",
            config.workspace_source,
            config.workspace_verified,
        ),
        max_joint_velocity=LimitValue(
            (config.max_joint_velocity.value,) * 6,
            "deg/s",
            config.max_joint_velocity.source,
            config.max_joint_velocity.verified,
        ),
        max_joint_acceleration=LimitValue(
            (config.max_joint_acceleration.value,) * 6,
            "deg/s^2",
            config.max_joint_acceleration.source,
            config.max_joint_acceleration.verified,
        ),
        max_joint_step_deg=LimitValue(
            config.max_joint_step.value,
            "deg",
            config.max_joint_step.source,
            config.max_joint_step.verified,
        ),
    )
    return TrajectoryValidator(stage2)


def _prepared_path(
    identifier, start, target, duration_s, validator
) -> PreparedTrajectory:
    delta = float(np.max(np.abs(np.asarray(target) - np.asarray(start))))
    samples = max(3, math.ceil(delta) + 1, math.ceil(duration_s / 0.1) + 1)
    times = np.linspace(0.0, duration_s, samples)
    joints = np.linspace(np.asarray(start), np.asarray(target), samples)
    joint = JointTrajectory.from_arrays(times, joints)
    cartesian = CartesianTrajectory.from_arrays(times, [pose_coords(q) for q in joints])
    return PreparedTrajectory(identifier, joint, validator.validate(cartesian, joint))


def build_hardware_application(
    service: RobotService,
    *,
    motion_enabled: bool,
    operator_supervising: bool,
    physical_stop_accessible: bool,
    workspace_clear: bool,
    config: Stage4AConfig | None = None,
) -> WorkstationApplication:
    """Compose only MotionExecutor -> MyCobotRobot -> RobotService -> RobotIO."""
    config = config or Stage4AConfig()
    state = service.refresh_state()
    if len(state.angles_deg) != 6:
        raise RuntimeError("hardware did not provide a finite six-joint state")
    current = tuple(float(value) for value in state.angles_deg)
    if config.ready_angles_deg is None:
        raise RuntimeError("READY is not configured")
    ready_target = tuple(float(value) for value in config.ready_angles_deg)
    validator = _validator(config)
    ready_delta = float(np.max(np.abs(np.asarray(ready_target) - np.asarray(current))))
    ready = _prepared_path(
        "stage4a_ready", current, ready_target, max(2.0, ready_delta / 3.0), validator
    )
    probe_target = list(ready_target)
    probe_target[config.probe_joint - 1] += config.probe_displacement_deg
    probe = _prepared_path(
        "stage4a_j6_probe",
        ready_target,
        tuple(probe_target),
        config.probe_duration_s,
        validator,
    )
    execution_config = ExecutionConfig(
        simulation_timestep_s=0.02,
        state_update_rate_hz=10.0,
        joint_velocity_limits_deg_s=(config.max_joint_velocity.value,) * 6,
        position_tolerance_deg=config.probe_tolerance_deg,
        state_freshness_timeout_s=config.telemetry_freshness_timeout_s,
        trajectory_completion_timeout_s=config.execution_timeout_s,
        stop_confirmation_timeout_s=config.stop_confirmation_timeout_s,
    )
    robot = MyCobotRobot(service, config)
    validation_holder = {"value": ready.validation}

    def operation_gates(prepared: PreparedTrajectory) -> tuple[GateResult, ...]:
        validation_holder["value"] = prepared.validation
        if not motion_enabled:
            return (
                hard_result(
                    "hardware_motion_authorization",
                    GateStatus.BLOCK,
                    reason="HARDWARE_MOTION_NOT_ENABLED",
                ),
            )
        try:
            decision = robot.preflight(
                prepared.trajectory,
                prepared.validation,
                operator_supervising=operator_supervising,
                physical_stop_accessible=physical_stop_accessible,
                workspace_clear=workspace_clear,
            )
        except PreflightRejected as exc:
            return tuple(exc.gate_results)
        return tuple(decision.results)

    def attach_log(log) -> None:
        robot.log = log

    snapshot = SystemSnapshotAdapter(
        service.latest_state,
        config=config,
        validation_provider=lambda: validation_holder["value"],
        target_provider=lambda: tuple(robot.get_state().commanded_angles_deg),
        now_provider=time.monotonic,
    )
    controller = OperationController(
        robot,
        WallClock(),
        execution_config,
        {ready.trajectory_id: ready, probe.trajectory_id: probe},
        gate_provider=snapshot.get_gate_results,
        ready_trajectory_id=ready.trajectory_id,
        mode=InteractionMode.OBSERVE,
        operation_gate_provider=operation_gates,
        execution_log_sink=attach_log,
        receipt_sink=lambda _receipt: robot.disarm(),
    )
    operations = ()
    if motion_enabled:
        operations = (
            WorkstationOperation(
                "ready",
                "GO TO READY",
                Operation.GO_READY,
                description="Stage-2-validated conservative READY trajectory",
            ),
            WorkstationOperation(
                "j6-probe",
                "RUN J6 +2° PROBE",
                Operation.EXECUTE_TRAJECTORY,
                trajectory_id=probe.trajectory_id,
                description="Prepared Stage-2-validated Stage-4A probe",
            ),
        )
    label = "myCobot 280 JN · Stage-4A hardware" + (
        "" if motion_enabled else " · READ ONLY"
    )
    return WorkstationApplication(snapshot, controller, operations, backend_label=label)


def _hardware_service(args, *, read_only: bool) -> RobotService:
    return RobotService(
        args.serial_port,
        args.baudrate,
        read_only=read_only,
        telemetry=True,
        runtime_profile=True,
        fast_hz=5.0,
        slow_hz=1.0,
    )


def _parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--backend", choices=("sim", "hardware"), default="sim")
    parser.add_argument("--serial-port", default="/dev/ttyTHS1")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--physical-preflight-pass", action="store_true")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--operator-supervising", action="store_true")
    parser.add_argument("--physical-stop-accessible", action="store_true")
    parser.add_argument("--workspace-clear", action="store_true")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    service = None
    if args.backend == "sim":
        application = build_offline_application()
        backend_message = "SimRobot offline core; no hardware connection is opened."
    else:
        if not args.physical_preflight_pass:
            raise SystemExit(
                "hardware backend requires --physical-preflight-pass for this session"
            )
        confirmations = (
            args.operator_supervising,
            args.physical_stop_accessible,
            args.workspace_clear,
        )
        if args.enable_motion and not all(confirmations):
            raise SystemExit(
                "--enable-motion also requires --operator-supervising, "
                "--physical-stop-accessible and --workspace-clear"
            )
        strict = _hardware_service(args, read_only=True)
        try:
            result = validate_read_only_session(
                strict, Stage4AConfig(), physical_preflight_pass=True
            )
            if result.decision != "GO" and args.enable_motion:
                raise SystemExit(
                    "hardware read-only gate returned NO_GO: "
                    + str(result.abort_reason)
                )
            if args.enable_motion:
                strict.close()
                strict = None
                service = _hardware_service(args, read_only=False)
            else:
                service = strict
                strict = None
            application = build_hardware_application(
                service,
                motion_enabled=args.enable_motion,
                operator_supervising=args.operator_supervising,
                physical_stop_accessible=args.physical_stop_accessible,
                workspace_clear=args.workspace_clear,
            )
        finally:
            if strict is not None:
                strict.close()
        backend_message = (
            "myCobot Stage-4A hardware; prepared motion enabled."
            if args.enable_motion
            else (
                "myCobot Stage-4A hardware; strict read-only workstation; "
                f"startup gate {result.decision}."
            )
        )

    server = create_workstation_server(
        application, STATIC_DIRECTORY, host=args.host, port=args.port
    )
    print(f"Robot Interaction Workstation: http://{args.host}:{args.port}")
    print(f"Backend: {backend_message}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if service is not None:
            service.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

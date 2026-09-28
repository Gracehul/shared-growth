#!/usr/bin/env python3
"""Run the integrated workstation against the deterministic offline backend."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
from pathlib import Path

from drawing_robot.execution import ExecutionConfig, RobotStatus, SimRobot, SimulationClock
from drawing_robot.hardware import ExecutionPhase, GateStatus, SystemSnapshotAdapter
from drawing_robot.operations import (
    InteractionMode,
    Operation,
    OperationController,
    PreparedTrajectory,
)
from drawing_robot.robot.state import RobotMode, RobotState as ServiceRobotState
from drawing_robot.stage2 import JointTrajectory, ValidationResult
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
    """Compose real core contracts with SimRobot; no hardware is imported."""

    # Start above zero because zero is reserved by the state contract for an
    # unavailable monotonic telemetry timestamp.
    clock = SimulationClock(1.0)
    config = ExecutionConfig(
        simulation_timestep_s=0.01,
        state_update_rate_hz=20.0,
        joint_velocity_limits_deg_s=(15.0,) * 6,
        position_tolerance_deg=0.1,
        trajectory_completion_timeout_s=5.0,
    )
    robot = SimRobot(clock, config, [0.0] * 6)
    validation = ValidationResult(True, metrics={"source": "offline workstation fixture"})
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
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [1.0, 0.5, 0.0, 0.0, 0.0, 0.5],
                [1.5, 0.5, -0.5, 0.0, 0.5, 1.0],
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.5],
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

    def target():
        return tuple(robot.get_state().commanded_angles_deg)

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
        target_provider=target,
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
        snapshot,
        controller,
        operations,
        backend_label="SimRobot · offline core",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    args = parser.parse_args()
    application = build_offline_application()
    server = create_workstation_server(
        application, STATIC_DIRECTORY, host=args.host, port=args.port
    )
    print(f"Robot Interaction Workstation: http://{args.host}:{args.port}")
    print("Backend: SimRobot offline core; no hardware connection is opened.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

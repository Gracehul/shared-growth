#!/usr/bin/env python3
"""Run one validated conservative trajectory on a supervised real robot."""

from __future__ import annotations

import argparse
import os
import time
from dataclasses import replace
from hashlib import sha256
from pathlib import Path

import numpy as np

from drawing_robot.execution import ExecutionConfig, ExecutionLog, MotionExecutor, WallClock
from drawing_robot.hardware import Limit, MyCobotRobot, Stage4AConfig, motion_envelope_gate
from drawing_robot.robot import RobotService
from drawing_robot.stage2.visualization import load_visualization_bundle


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", help="validated Stage-2B JSON bundle")
    parser.add_argument("--output", required=True, help="hardware execution log JSON")
    parser.add_argument("--trajectory-id", default="stage4a-trajectory")
    parser.add_argument("--port", default="/dev/ttyTHS1")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--max-duration", type=float, default=15.0)
    parser.add_argument("--position-tolerance", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true", help="explicitly permit motion")
    parser.add_argument("--operator-supervising", action="store_true")
    parser.add_argument("--physical-stop-accessible", action="store_true")
    parser.add_argument("--workspace-clear", action="store_true")
    parser.add_argument(
        "--workspace-bounds", type=float, nargs=6,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        help="explicit provisional XYZ free-space bounds in mm",
    )
    return parser.parse_args()


def wait_for_telemetry(service: RobotService, timeout_s: float = 5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = service.latest_state()
        if len(state.angles_deg) == 6 and len(state.temperatures_c) == 6:
            return state
        time.sleep(0.05)
    raise RuntimeError("complete fresh telemetry was not available within 5 seconds")


def verify_final_settling(service, robot, target, tolerance_deg, samples=5):
    records = []
    previous = None
    for index in range(samples):
        raw = service.refresh_critical_state()
        state = robot.get_state()
        actual = np.asarray(raw.angles_deg, dtype=float)
        delta = None if previous is None else float(np.max(np.abs(actual - previous)))
        records.append({
            "sample": index + 1,
            "angles_deg": actual.tolist(),
            "max_target_error_deg": float(np.max(np.abs(actual - target))),
            "max_angle_delta_deg": delta,
            "temperature_max_c": max(raw.temperatures_c),
            "controller_error": raw.controller_error,
            "is_fresh": state.is_fresh,
        })
        previous = actual
        if index + 1 < samples:
            time.sleep(0.2)
    passed = all(
        item["is_fresh"]
        and item["max_target_error_deg"] <= tolerance_deg
        and item["controller_error"] == 0
        and item["temperature_max_c"] < 60.0
        and (
            item["max_angle_delta_deg"] is None
            or item["max_angle_delta_deg"] <= 0.05
        )
        for item in records
    )
    return passed, records


def main() -> int:
    args = parse_args()
    if not args.execute or os.environ.get("RUN_REAL_ROBOT_TESTS") != "1":
        raise SystemExit(
            "Motion blocked. Require both --execute and RUN_REAL_ROBOT_TESTS=1."
        )
    cartesian, trajectory, validation = load_visualization_bundle(args.trajectory)
    hardware_cfg = Stage4AConfig()
    hardware_cfg = replace(
        hardware_cfg,
        max_trajectory_duration=Limit(
            args.max_duration, "s", "operator_supplied_project_assumption", False
        ),
        position_tolerance_deg=args.position_tolerance,
    )
    if args.workspace_bounds:
        values = args.workspace_bounds
        hardware_cfg = replace(
            hardware_cfg,
            allowed_workspace_mm=((values[0], values[1]), (values[2], values[3]), (values[4], values[5])),
            workspace_source="operator_supplied_project_assumption",
            workspace_verified=False,
        )
    envelope = motion_envelope_gate(trajectory, hardware_cfg, cartesian)
    if not envelope.allowed:
        raise SystemExit("Stage-4A envelope rejected trajectory: " + "; ".join(envelope.reasons))
    execution_cfg = ExecutionConfig(
        simulation_timestep_s=0.02,
        state_update_rate_hz=10.0,
        joint_velocity_limits_deg_s=(hardware_cfg.max_joint_velocity.value,) * 6,
        position_tolerance_deg=hardware_cfg.position_tolerance_deg,
        state_freshness_timeout_s=hardware_cfg.telemetry_freshness_timeout_s,
        trajectory_completion_timeout_s=5.0,
        stop_confirmation_timeout_s=hardware_cfg.stop_confirmation_timeout_s,
    )
    with RobotService(
        args.port,
        args.baud,
        telemetry=True,
        runtime_profile=True,
        fast_hz=hardware_cfg.critical_telemetry_hz,
        slow_hz=hardware_cfg.temperature_telemetry_hz,
    ) as service:
        wait_for_telemetry(service)
        robot = MyCobotRobot(service, hardware_cfg)
        initial = robot.get_state()
        metadata = robot.hardware_metadata(service)
        metadata["firmware_version"] = service.get_system_version()
        metadata["stage4a_config"] = hardware_cfg.to_dict()
        metadata["trajectory_sha256"] = sha256(Path(args.trajectory).read_bytes()).hexdigest()
        log = ExecutionLog(
            args.trajectory_id,
            execution_cfg.to_dict(),
            execution_cfg.random_seed,
            initial,
            backend=metadata,
            hardware_run=True,
            safety_profile="stage4a_conservative",
        )
        robot.log = log
        try:
            robot.preflight(
                trajectory,
                validation,
                operator_supervising=args.operator_supervising,
                physical_stop_accessible=args.physical_stop_accessible,
                workspace_clear=args.workspace_clear,
            )
            result = MotionExecutor(robot, WallClock(), execution_cfg, log).execute(
                trajectory, validation, trajectory_id=args.trajectory_id
            )
            settled = False
            settling_records = []
            if result.status.value == "COMPLETED":
                settled, settling_records = verify_final_settling(
                    service,
                    robot,
                    np.asarray(trajectory.samples[-1].positions_deg),
                    hardware_cfg.position_tolerance_deg,
                )
                log.record_phase(
                    "final_settling",
                    "PASS" if settled else "FAIL",
                    "GO" if settled else "NO_GO",
                    samples=settling_records,
                )
                service.refresh_state()
                robot.get_state()
                if not settled:
                    robot.stop_motion()
        finally:
            log.write(args.output)
        final_status = result.status.value if settled else "FAILED_SETTLING"
        print(f"Stage-4A result: {final_status}; log={args.output}")
        return 0 if result.status.value == "COMPLETED" and settled else 1


if __name__ == "__main__":
    raise SystemExit(main())

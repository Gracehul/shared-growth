#!/usr/bin/env python3
"""Validate READY, one J6 probe and one gated multi-joint probe on hardware."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import time
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

from drawing_robot.execution import (
    ExecutionConfig,
    ExecutionLog,
    MotionExecutor,
    RobotState,
    RobotStatus,
    WallClock,
)
from drawing_robot.hardware import MyCobotRobot, Stage4AConfig, motion_envelope_gate
from drawing_robot.kinematics import pose_coords
from drawing_robot.robot import RobotService
from drawing_robot.stage2 import (
    CartesianTrajectory,
    JointTrajectory,
    LimitValue,
    Stage2Config,
    TrajectoryValidator,
    WorkspaceBounds,
)


def sampled_joint_path(start, target, duration_s, samples=11):
    times = np.linspace(0.0, duration_s, samples)
    joints = np.linspace(np.asarray(start, dtype=float), np.asarray(target, dtype=float), samples)
    joint = JointTrajectory.from_arrays(times, joints)
    cartesian = CartesianTrajectory.from_arrays(times, [pose_coords(q) for q in joints])
    return cartesian, joint


def stage2_validator(config: Stage4AConfig):
    bounds = config.allowed_workspace_mm
    if bounds is None:
        raise RuntimeError("temporary workspace is not configured")
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


def trajectory_hash(cartesian, joint):
    payload = {
        "cartesian": [asdict(sample) for sample in cartesian.samples],
        "joint": [asdict(sample) for sample in joint.samples],
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def config_hash(config):
    encoded = json.dumps(config.to_dict(), sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def execution_config(config, *, position_tolerance_deg=None):
    return ExecutionConfig(
        simulation_timestep_s=0.02,
        state_update_rate_hz=10.0,
        joint_velocity_limits_deg_s=(config.max_joint_velocity.value,) * 6,
        position_tolerance_deg=(
            config.position_tolerance_deg
            if position_tolerance_deg is None
            else position_tolerance_deg
        ),
        state_freshness_timeout_s=config.telemetry_freshness_timeout_s,
        trajectory_completion_timeout_s=5.0,
        stop_confirmation_timeout_s=config.stop_confirmation_timeout_s,
    )


def make_log(service, robot, config, trajectory_id, trajectory_digest, tolerance_deg):
    initial = robot.get_state()
    backend = robot.hardware_metadata(service)
    backend.update({
        "firmware_version": service.get_system_version(),
        "stage4a_config": config.to_dict(),
    })
    return ExecutionLog(
        trajectory_id,
        execution_config(config, position_tolerance_deg=tolerance_deg).to_dict(),
        0,
        initial,
        backend=backend,
        hardware_run=True,
        safety_profile=config.profile,
        provenance={
            "git_commit": os.environ.get("SHARED_GROWTH_GIT_COMMIT", "unknown"),
            "configuration_hash_sha256": config_hash(config),
            "trajectory_hash_sha256": trajectory_digest,
            "physical_preflight": "PASS",
        },
    )


def settle_and_measure(service, robot, config, target, log, tolerance_deg):
    samples = []
    previous_angles = None
    for index in range(config.ready_settling_samples):
        started = time.monotonic()
        raw = service.refresh_critical_state()
        state = robot.get_state()
        error = np.asarray(raw.angles_deg) - np.asarray(target)
        angle_delta = (
            None
            if previous_angles is None
            else float(np.max(np.abs(np.asarray(raw.angles_deg) - previous_angles)))
        )
        previous_angles = np.asarray(raw.angles_deg)
        sample = {
            "settling_sample": index + 1,
            "actual_angles_deg": list(raw.angles_deg),
            "error_deg": error.tolist(),
            "max_error_deg": float(np.max(np.abs(error))),
            "temperatures_c": list(raw.temperatures_c),
            "controller_error": raw.controller_error,
            "servo_status": list(raw.servo_status),
            "state_age_s": state.state_age_s,
            "is_fresh": state.is_fresh,
            "max_angle_delta_deg": angle_delta,
        }
        samples.append(sample)
        if index + 1 < config.ready_settling_samples:
            time.sleep(max(0.0, config.verification_sample_period_s - (time.monotonic() - started)))
    passed = all(
        sample["is_fresh"]
        and sample["max_error_deg"] <= tolerance_deg
        and sample["controller_error"] == 0
        and len(sample["temperatures_c"]) == 6
        and max(sample["temperatures_c"]) < config.temperature_abort_c
        and len(sample["servo_status"]) == 6
        and all(value == 0 for value in sample["servo_status"])
        and (
            sample["max_angle_delta_deg"] is None
            or sample["max_angle_delta_deg"] <= config.cessation_angle_delta_deg
        )
        for sample in samples
    )
    for sample in samples:
        log.record_hardware_telemetry({"verification": sample})
    return passed, samples


def uart_metrics(log):
    if not log.uart_transactions:
        return {"count": 0}
    durations = [item["io_end_s"] - item["io_start_s"] for item in log.uart_transactions]
    queue_delays = [item["io_start_s"] - item["queued_at_s"] for item in log.uart_transactions]
    return {
        "count": len(durations),
        "api_duration_mean_s": float(np.mean(durations)),
        "api_duration_max_s": float(np.max(durations)),
        "queue_delay_mean_s": float(np.mean(queue_delays)),
        "queue_delay_max_s": float(np.max(queue_delays)),
        "errors": sum(item["status"] != "ok" for item in log.uart_transactions),
    }


def execute_phase(service, robot, config, cartesian, joint, validation, phase, output):
    digest = trajectory_hash(cartesian, joint)
    tolerance_deg = (
        config.ready_tolerance_deg
        if phase == "provisional_ready"
        else config.probe_tolerance_deg
        if phase == "single_joint_probe"
        else config.position_tolerance_deg
    )
    log = make_log(service, robot, config, phase, digest, tolerance_deg)
    robot.log = log
    log.record_phase("stage2_validation", "PASS", "GO", metrics=validation.metrics)
    try:
        robot.preflight(
            joint,
            validation,
            operator_supervising=True,
            physical_stop_accessible=True,
            workspace_clear=True,
        )
        result = MotionExecutor(
            robot,
            WallClock(),
            execution_config(config, position_tolerance_deg=tolerance_deg),
            log,
        ).execute(joint, validation, trajectory_id=phase)
        settled, settling = settle_and_measure(
            service, robot, config, joint.samples[-1].positions_deg, log, tolerance_deg
        ) if result.status.value == "COMPLETED" else (False, [])
        if result.status.value == "COMPLETED":
            service.refresh_state()
            robot.get_state()
        final_actual = settling[-1]["actual_angles_deg"] if settling else list(result.final_state.actual_angles_deg)
        final_error = (
            np.asarray(final_actual) - np.asarray(joint.samples[-1].positions_deg)
        ).tolist()
        phase_pass = result.status.value == "COMPLETED" and settled
        details = {
            "execution_status": result.status.value,
            "settled": settled,
            "settling_samples": settling,
            "final_error_deg_per_joint": final_error,
            "max_final_error_deg": max(abs(value) for value in final_error),
            "temperature_min_c": min((min(item["temperatures_c"]) for item in settling), default=None),
            "temperature_max_c": max((max(item["temperatures_c"]) for item in settling), default=None),
            "fault_count": sum(item["controller_error"] != 0 for item in settling),
            "uart": uart_metrics(log),
        }
        log.record_phase(phase, "PASS" if phase_pass else "FAIL", "GO" if phase_pass else "NO_GO", **details)
        log.finish("passed" if phase_pass else "failed", time.monotonic(), **details)
        if not phase_pass:
            robot.disarm()
        return phase_pass, details
    except Exception as exc:
        robot.disarm()
        details = {
            "execution_status": "FAILED",
            "settled": False,
            "abort_reason": f"{type(exc).__name__}: {exc}",
            "uart": uart_metrics(log),
        }
        log.record_phase(phase, "FAIL", "NO_GO", **details)
        log.finish("failed", time.monotonic(), **details)
        return False, details
    finally:
        log.write(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--port", default="/dev/ttyTHS1")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--operator-supervising", action="store_true")
    parser.add_argument("--physical-stop-accessible", action="store_true")
    parser.add_argument("--workspace-clear", action="store_true")
    args = parser.parse_args()
    confirmations = (
        args.execute,
        args.operator_supervising,
        args.physical_stop_accessible,
        args.workspace_clear,
        os.environ.get("RUN_REAL_ROBOT_TESTS") == "1",
    )
    if not all(confirmations):
        raise SystemExit("NO_GO: all supervision flags and RUN_REAL_ROBOT_TESTS=1 are required")

    config = Stage4AConfig()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "schema_version": "shared-growth/stage4a-probe/v1",
        "git_commit": os.environ.get("SHARED_GROWTH_GIT_COMMIT", "unknown"),
        "configuration_hash_sha256": config_hash(config),
        "workspace_mm": config.allowed_workspace_mm,
        "ready_target_deg": config.ready_angles_deg,
        "probe": {
            "joint": config.probe_joint,
            "displacement_deg": config.probe_displacement_deg,
            "duration_s": config.probe_duration_s,
            "completion_tolerance_deg": config.probe_tolerance_deg,
        },
        "multi_joint": {
            "offsets_deg": config.multi_joint_offsets_deg,
            "duration_s": config.multi_joint_duration_s,
            "completion_tolerance_deg": config.position_tolerance_deg,
        },
        "physical_preflight": "PASS",
        "overall_decision": "NO_GO",
        "phases": [],
    }
    summary_path = output_dir / "summary.json"
    try:
        with RobotService(
            args.port,
            args.baud,
            telemetry=True,
            runtime_profile=True,
            fast_hz=config.critical_telemetry_hz,
            slow_hz=config.temperature_telemetry_hz,
        ) as service:
            raw = service.refresh_state()
            current = tuple(raw.angles_deg)
            ready = config.ready_angles_deg
            if ready is None:
                raise RuntimeError("READY is unconfigured")
            probe = list(ready)
            probe[config.probe_joint - 1] += config.probe_displacement_deg
            multi_joint_target = [
                value + offset
                for value, offset in zip(probe, config.multi_joint_offsets_deg)
            ]
            ready_cart, ready_joint = sampled_joint_path(current, ready, 1.0, samples=2)
            probe_cart, probe_joint = sampled_joint_path(ready, probe, config.probe_duration_s)
            multi_cart, multi_joint = sampled_joint_path(
                probe,
                multi_joint_target,
                config.multi_joint_duration_s,
            )
            validator = stage2_validator(config)
            ready_validation = validator.validate(ready_cart, ready_joint)
            probe_validation = validator.validate(probe_cart, probe_joint)
            multi_validation = validator.validate(multi_cart, multi_joint)
            ready_envelope = motion_envelope_gate(ready_joint, config, ready_cart)
            probe_envelope = motion_envelope_gate(probe_joint, config, probe_cart)
            multi_envelope = motion_envelope_gate(multi_joint, config, multi_cart)
            planning_pass = all((
                ready_validation.valid,
                probe_validation.valid,
                ready_envelope.allowed,
                probe_envelope.allowed,
                multi_validation.valid,
                multi_envelope.allowed,
            ))
            summary["phases"].append({
                "phase": "offline_planning",
                "phase_result": "PASS" if planning_pass else "FAIL",
                "decision": "GO" if planning_pass else "NO_GO",
                "current_angles_deg": current,
                "current_tcp_pose_mm_deg": pose_coords(current).tolist(),
                "ready_errors": [issue.code for issue in ready_validation.errors],
                "probe_errors": [issue.code for issue in probe_validation.errors],
                "multi_joint_errors": [issue.code for issue in multi_validation.errors],
                "ready_envelope_reasons": ready_envelope.reasons,
                "probe_envelope_reasons": probe_envelope.reasons,
                "multi_joint_envelope_reasons": multi_envelope.reasons,
            })
            if not planning_pass:
                return 2
            robot = MyCobotRobot(service, config)
            ready_pass, ready_details = execute_phase(
                service, robot, config, ready_cart, ready_joint, ready_validation,
                "provisional_ready", output_dir / "ready.json",
            )
            summary["phases"].append({"phase": "provisional_ready", "phase_result": "PASS" if ready_pass else "FAIL", "decision": "GO" if ready_pass else "NO_GO", **ready_details})
            if not ready_pass:
                return 3
            probe_pass, probe_details = execute_phase(
                service, robot, config, probe_cart, probe_joint, probe_validation,
                "single_joint_probe", output_dir / "probe.json",
            )
            summary["phases"].append({"phase": "single_joint_probe", "phase_result": "PASS" if probe_pass else "FAIL", "decision": "GO" if probe_pass else "NO_GO", **probe_details})
            if not probe_pass:
                return 4
            multi_pass, multi_details = execute_phase(
                service, robot, config, multi_cart, multi_joint, multi_validation,
                "multi_joint_free_space", output_dir / "multi_joint.json",
            )
            summary["phases"].append({"phase": "multi_joint_free_space", "phase_result": "PASS" if multi_pass else "FAIL", "decision": "GO" if multi_pass else "NO_GO", **multi_details})
            summary["overall_decision"] = "GO" if multi_pass else "NO_GO"
            return 0 if multi_pass else 5
    except Exception as exc:
        summary["abort_reason"] = f"{type(exc).__name__}: {exc}"
        return 5
    finally:
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    raise SystemExit(main())

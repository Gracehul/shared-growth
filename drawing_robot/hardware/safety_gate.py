"""Separate hard runtime blocks from performance/qualification evidence."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..robot.state import RobotState as ServiceState
from ..stage2 import CartesianTrajectory, JointTrajectory, ValidationResult
from .config import Stage4AConfig
from .policy import GateResult, GateStatus, hard_result


@dataclass(frozen=True)
class GateDecision:
    results: tuple[GateResult, ...] = ()

    @property
    def allowed(self) -> bool:
        return not any(result.hard_blocking for result in self.results)

    @property
    def qualified(self) -> bool:
        return self.allowed and not any(
            result.qualification_miss for result in self.results
        )

    @property
    def reasons(self) -> tuple[str, ...]:
        return tuple(
            result.reason or result.name
            for result in self.results
            if result.hard_blocking
        )

    @property
    def warnings(self) -> tuple[str, ...]:
        return tuple(
            result.reason or result.name
            for result in self.results
            if result.status is GateStatus.WARN or result.qualification_miss
        )


def _boolean_gate(name: str, passed: bool, failure: str) -> GateResult:
    return hard_result(
        name,
        GateStatus.PASS if passed else GateStatus.BLOCK,
        value=passed,
        reason=None if passed else failure,
    )


def telemetry_gate_results(
    state: ServiceState, cfg: Stage4AConfig
) -> tuple[GateResult, ...]:
    results: list[GateResult] = [
        _boolean_gate(
            "connection", state.connected, "CONNECTION_FAILURE: robot is disconnected"
        )
    ]
    results.append(
        hard_result(
            "joint_telemetry",
            GateStatus.PASS if len(state.angles_deg) == 6 else GateStatus.UNKNOWN,
            value=len(state.angles_deg),
            limit=6,
            unit="joints",
            reason=None
            if len(state.angles_deg) == 6
            else "STALE_TELEMETRY: joint angles unavailable",
        )
    )
    if len(state.temperatures_c) != 6:
        results.append(
            hard_result(
                "temperature",
                GateStatus.UNKNOWN,
                limit=cfg.temperature_abort_c,
                unit="degC",
                reason="TEMPERATURE_UNKNOWN: six joint temperatures required",
            )
        )
    else:
        maximum = max(state.temperatures_c)
        status = (
            GateStatus.BLOCK
            if maximum >= cfg.temperature_abort_c
            else GateStatus.WARN
            if maximum >= cfg.temperature_warning_c
            else GateStatus.PASS
        )
        results.append(
            hard_result(
                "temperature",
                status,
                value=maximum,
                limit=cfg.temperature_abort_c,
                unit="degC",
                reason=(
                    "TEMPERATURE_ABORT: project abort threshold reached"
                    if status is GateStatus.BLOCK
                    else "TEMPERATURE_WARNING: project warning threshold reached"
                    if status is GateStatus.WARN
                    else None
                ),
            )
        )
    controller_status = (
        GateStatus.UNKNOWN
        if state.controller_error is None
        else GateStatus.PASS
        if state.controller_error == 0
        else GateStatus.BLOCK
    )
    results.append(
        hard_result(
            "controller_fault",
            controller_status,
            value=state.controller_error,
            limit=0,
            reason=(
                "STALE_TELEMETRY: controller fault state unavailable"
                if state.controller_error is None
                else f"FAULT_REPORTED: controller error {state.controller_error}"
                if state.controller_error != 0
                else None
            ),
        )
    )
    results.append(
        hard_result(
            "backend_fault",
            GateStatus.BLOCK if state.fault else GateStatus.PASS,
            value=state.fault,
            reason=f"FAULT_REPORTED: {state.fault}" if state.fault else None,
        )
    )
    results.extend(
        (
            _boolean_gate(
                "power",
                state.powered is True,
                "POWER_UNAVAILABLE: robot power is not confirmed",
            ),
            _boolean_gate(
                "servos_enabled",
                state.servos_enabled is True,
                "SERVOS_UNAVAILABLE: servo enable is not confirmed",
            ),
        )
    )
    if len(state.servo_status) != 6:
        results.append(
            hard_result(
                "servo_status",
                GateStatus.UNKNOWN,
                reason="STALE_TELEMETRY: servo status unavailable",
            )
        )
    else:
        bad_status = any(value != 0 for value in state.servo_status)
        results.append(
            hard_result(
                "servo_status",
                GateStatus.BLOCK if bad_status else GateStatus.PASS,
                value=tuple(state.servo_status),
                limit=0,
                reason="FAULT_REPORTED: non-zero servo status" if bad_status else None,
            )
        )
    return tuple(results)


def validation_gate_result(validation: ValidationResult | None) -> GateResult:
    if validation is None:
        return hard_result(
            "trajectory_validity",
            GateStatus.UNKNOWN,
            reason="TRAJECTORY_UNKNOWN: Stage-2 validation unavailable",
        )
    valid = validation.valid and not validation.errors
    return hard_result(
        "trajectory_validity",
        GateStatus.PASS if valid else GateStatus.BLOCK,
        value=valid,
        reason=None if valid else "TRAJECTORY_REJECTED: Stage-2 validation failed",
    )


def freshness_gate_result(
    state: ServiceState, cfg: Stage4AConfig, *, now_s: float
) -> GateResult:
    critical_timestamp = state.critical_monotonic_s or state.monotonic_s
    age = now_s - critical_timestamp if critical_timestamp > 0 else None
    status = (
        GateStatus.UNKNOWN
        if age is None
        else GateStatus.PASS
        if age <= cfg.telemetry_freshness_timeout_s
        else GateStatus.BLOCK
    )
    return hard_result(
        "telemetry_freshness",
        status,
        value=age,
        limit=cfg.telemetry_freshness_timeout_s,
        unit="s",
        reason=None
        if status is GateStatus.PASS
        else "STALE_TELEMETRY: critical timestamp unavailable"
        if age is None
        else f"STALE_TELEMETRY: state age {age:.3f}s",
    )


def preflight_gate(
    state: ServiceState,
    cfg: Stage4AConfig,
    validation: ValidationResult,
    *,
    operator_supervising: bool,
    physical_stop_accessible: bool,
    workspace_clear: bool,
    now_s: float,
) -> GateDecision:
    results = list(telemetry_gate_results(state, cfg))
    results.extend(
        (
            validation_gate_result(validation),
            freshness_gate_result(state, cfg, now_s=now_s),
            _boolean_gate(
                "operator_supervision",
                operator_supervising,
                "OPERATOR_REQUIRED: local supervision is not confirmed",
            ),
            _boolean_gate(
                "physical_stop_access",
                physical_stop_accessible,
                "PHYSICAL_STOP_REQUIRED: stop access is not confirmed",
            ),
            _boolean_gate(
                "physical_workspace_clear",
                workspace_clear,
                "WORKSPACE_NOT_CLEAR: physical clearance is not confirmed",
            ),
        )
    )
    return GateDecision(tuple(results))


def runtime_gate(
    state: ServiceState, cfg: Stage4AConfig, *, now_s: float
) -> GateDecision:
    return GateDecision(
        telemetry_gate_results(state, cfg)
        + (freshness_gate_result(state, cfg, now_s=now_s),)
    )


def motion_envelope_gate(
    trajectory: JointTrajectory,
    cfg: Stage4AConfig,
    cartesian: CartesianTrajectory | None = None,
) -> GateDecision:
    samples = trajectory.samples
    if not samples:
        return GateDecision(
            (hard_result("trajectory_samples", GateStatus.BLOCK, reason="EMPTY_TRAJECTORY"),)
        )
    results: list[GateResult] = []
    times = np.asarray([sample.time_s for sample in samples], dtype=float)
    joints = np.asarray([sample.positions_deg for sample in samples], dtype=float)
    duration = float(times[-1] - times[0])
    duration_ok = duration <= cfg.max_trajectory_duration.value
    results.append(
        hard_result(
            "trajectory_duration",
            GateStatus.PASS if duration_ok else GateStatus.BLOCK,
            value=duration,
            limit=cfg.max_trajectory_duration.value,
            unit="s",
            reason=None if duration_ok else "STAGE4A_DURATION_EXCEEDED",
        )
    )
    if len(samples) > 1:
        dt = np.diff(times)
        dq = np.diff(joints, axis=0)
        max_step = float(np.max(np.abs(dq)))
        step_ok = max_step <= cfg.max_joint_step.value
        results.append(
            hard_result(
                "joint_step",
                GateStatus.PASS if step_ok else GateStatus.BLOCK,
                value=max_step,
                limit=cfg.max_joint_step.value,
                unit="deg",
                reason=None if step_ok else "STAGE4A_JOINT_STEP_EXCEEDED",
            )
        )
        velocity = dq / dt[:, None]
        max_velocity = float(np.max(np.abs(velocity)))
        velocity_ok = max_velocity <= cfg.max_joint_velocity.value
        results.append(
            hard_result(
                "joint_velocity",
                GateStatus.PASS if velocity_ok else GateStatus.BLOCK,
                value=max_velocity,
                limit=cfg.max_joint_velocity.value,
                unit="deg/s",
                reason=None if velocity_ok else "STAGE4A_VELOCITY_EXCEEDED",
            )
        )
        if len(samples) > 2:
            accel_dt = 0.5 * (dt[1:] + dt[:-1])
            max_acceleration = float(
                np.max(np.abs(np.diff(velocity, axis=0) / accel_dt[:, None]))
            )
            acceleration_ok = max_acceleration <= cfg.max_joint_acceleration.value
            results.append(
                hard_result(
                    "joint_acceleration",
                    GateStatus.PASS if acceleration_ok else GateStatus.BLOCK,
                    value=max_acceleration,
                    limit=cfg.max_joint_acceleration.value,
                    unit="deg/s^2",
                    reason=None if acceleration_ok else "STAGE4A_ACCELERATION_EXCEEDED",
                )
            )
    if cartesian is not None and cfg.allowed_workspace_mm is None:
        results.append(
            hard_result(
                "workspace", GateStatus.UNKNOWN, reason="STAGE4A_WORKSPACE_UNCONFIGURED"
            )
        )
    elif cartesian is not None and cfg.allowed_workspace_mm is not None:
        points = np.asarray([sample.pose_mm_deg[:3] for sample in cartesian.samples], dtype=float)
        inside = all(
            not np.any((points[:, axis] < low) | (points[:, axis] > high))
            for axis, (low, high) in enumerate(cfg.allowed_workspace_mm)
        )
        results.append(
            hard_result(
                "workspace",
                GateStatus.PASS if inside else GateStatus.BLOCK,
                value=cfg.allowed_workspace_mm,
                unit="mm",
                reason=None if inside else "STAGE4A_WORKSPACE_EXCEEDED",
            )
        )
    return GateDecision(tuple(results))

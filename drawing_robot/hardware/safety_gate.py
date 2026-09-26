"""Separate preflight, runtime and conservative-envelope gates."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..robot.state import RobotState as ServiceState
from ..stage2 import CartesianTrajectory, JointTrajectory, ValidationResult
from .config import Stage4AConfig


@dataclass(frozen=True)
class GateDecision:
    allowed: bool
    reasons: tuple[str, ...] = ()
    warnings: tuple[str, ...] = ()


def _telemetry_reasons(state: ServiceState, cfg: Stage4AConfig) -> list[str]:
    reasons: list[str] = []
    if not state.connected:
        reasons.append("CONNECTION_FAILURE: robot is disconnected")
    if len(state.angles_deg) != 6:
        reasons.append("STALE_TELEMETRY: joint angles unavailable")
    if len(state.temperatures_c) != 6:
        reasons.append("TEMPERATURE_UNKNOWN: six joint temperatures required")
    elif max(state.temperatures_c) >= cfg.temperature_abort_c:
        reasons.append("TEMPERATURE_ABORT: project abort threshold reached")
    if state.controller_error is None:
        reasons.append("STALE_TELEMETRY: controller fault state unavailable")
    elif state.controller_error != 0:
        reasons.append(f"FAULT_REPORTED: controller error {state.controller_error}")
    if state.fault:
        reasons.append(f"FAULT_REPORTED: {state.fault}")
    if state.powered is not True:
        reasons.append("POWER_UNAVAILABLE: robot power is not confirmed")
    if state.servos_enabled is not True:
        reasons.append("SERVOS_UNAVAILABLE: servo enable is not confirmed")
    if len(state.servo_status) != 6:
        reasons.append("STALE_TELEMETRY: servo status unavailable")
    elif any(value != 0 for value in state.servo_status):
        reasons.append("FAULT_REPORTED: non-zero servo status")
    return reasons


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
    reasons = _telemetry_reasons(state, cfg)
    warnings: list[str] = []
    if not validation.valid or validation.errors:
        reasons.append("TRAJECTORY_REJECTED: Stage-2 validation failed")
    age = now_s - state.monotonic_s
    if state.monotonic_s <= 0 or age > cfg.telemetry_freshness_timeout_s:
        reasons.append(f"STALE_TELEMETRY: state age {age:.3f}s")
    if not operator_supervising:
        reasons.append("OPERATOR_REQUIRED: local supervision is not confirmed")
    if not physical_stop_accessible:
        reasons.append("PHYSICAL_STOP_REQUIRED: stop access is not confirmed")
    if not workspace_clear:
        reasons.append("WORKSPACE_NOT_CLEAR: physical clearance is not confirmed")
    if state.temperatures_c and max(state.temperatures_c) >= cfg.temperature_warning_c:
        warnings.append("TEMPERATURE_WARNING: project warning threshold reached")
    return GateDecision(not reasons, tuple(reasons), tuple(warnings))


def runtime_gate(state: ServiceState, cfg: Stage4AConfig, *, now_s: float) -> GateDecision:
    reasons = _telemetry_reasons(state, cfg)
    age = now_s - state.monotonic_s
    if state.monotonic_s <= 0 or age > cfg.telemetry_freshness_timeout_s:
        reasons.append(f"STALE_TELEMETRY: state age {age:.3f}s")
    return GateDecision(not reasons, tuple(reasons))


def motion_envelope_gate(
    trajectory: JointTrajectory,
    cfg: Stage4AConfig,
    cartesian: CartesianTrajectory | None = None,
) -> GateDecision:
    samples = trajectory.samples
    reasons: list[str] = []
    if not samples:
        return GateDecision(False, ("EMPTY_TRAJECTORY",))
    times = np.asarray([s.time_s for s in samples], dtype=float)
    joints = np.asarray([s.positions_deg for s in samples], dtype=float)
    if times[-1] - times[0] > cfg.max_trajectory_duration.value:
        reasons.append("STAGE4A_DURATION_EXCEEDED")
    if len(samples) > 1:
        dt = np.diff(times)
        dq = np.diff(joints, axis=0)
        if np.max(np.abs(dq)) > cfg.max_joint_step.value:
            reasons.append("STAGE4A_JOINT_STEP_EXCEEDED")
        velocity = dq / dt[:, None]
        if np.max(np.abs(velocity)) > cfg.max_joint_velocity.value:
            reasons.append("STAGE4A_VELOCITY_EXCEEDED")
        if len(samples) > 2:
            accel_dt = 0.5 * (dt[1:] + dt[:-1])
            accel = np.diff(velocity, axis=0) / accel_dt[:, None]
            if np.max(np.abs(accel)) > cfg.max_joint_acceleration.value:
                reasons.append("STAGE4A_ACCELERATION_EXCEEDED")
    if cartesian is not None and cfg.allowed_workspace_mm is None:
        reasons.append("STAGE4A_WORKSPACE_UNCONFIGURED")
    elif cartesian is not None and cfg.allowed_workspace_mm is not None:
        points = np.asarray([s.pose_mm_deg[:3] for s in cartesian.samples], dtype=float)
        for axis, (low, high) in enumerate(cfg.allowed_workspace_mm):
            if np.any((points[:, axis] < low) | (points[:, axis] > high)):
                reasons.append("STAGE4A_WORKSPACE_EXCEEDED")
                break
    return GateDecision(not reasons, tuple(reasons))

"""Explicit READY/PARK verification without hidden robot motion."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from ..robot.state import RobotState
from .config import Stage4AConfig


class LifecycleState(str, Enum):
    READY_UNCONFIRMED = "READY_UNCONFIRMED"
    READY_CONFIRMED = "READY_CONFIRMED"
    PARK_REQUESTED = "PARK_REQUESTED"
    PARKING = "PARKING"
    PARKED = "PARKED"
    PARK_BLOCKED = "PARK_BLOCKED"
    PARK_FAILED = "PARK_FAILED"


@dataclass(frozen=True)
class PoseVerification:
    confirmed: bool
    state: LifecycleState
    reason: str | None = None
    max_error_deg: float | None = None


def verify_ready(state: RobotState, cfg: Stage4AConfig) -> PoseVerification:
    if cfg.ready_angles_deg is None:
        return PoseVerification(False, LifecycleState.READY_UNCONFIRMED, "READY pose is not configured")
    if len(state.angles_deg) != 6:
        return PoseVerification(False, LifecycleState.READY_UNCONFIRMED, "joint telemetry unavailable")
    error = float(np.max(np.abs(np.asarray(state.angles_deg) - cfg.ready_angles_deg)))
    confirmed = error <= cfg.position_tolerance_deg
    return PoseVerification(
        confirmed,
        LifecycleState.READY_CONFIRMED if confirmed else LifecycleState.READY_UNCONFIRMED,
        None if confirmed else "START_STATE_MISMATCH",
        error,
    )


def verify_park(state: RobotState, cfg: Stage4AConfig) -> PoseVerification:
    if cfg.park_angles_deg is None:
        return PoseVerification(False, LifecycleState.PARK_BLOCKED, "PARK pose is not configured")
    if len(state.temperatures_c) != 6:
        return PoseVerification(False, LifecycleState.PARK_BLOCKED, "temperature unknown")
    if max(state.temperatures_c) >= cfg.temperature_abort_c:
        return PoseVerification(False, LifecycleState.PARK_BLOCKED, "temperature prohibits motion")
    if not state.connected or state.fault:
        return PoseVerification(False, LifecycleState.PARK_BLOCKED, "communication or fault state blocks PARK")
    if len(state.angles_deg) != 6:
        return PoseVerification(False, LifecycleState.PARK_FAILED, "joint telemetry unavailable")
    error = float(np.max(np.abs(np.asarray(state.angles_deg) - cfg.park_angles_deg)))
    confirmed = error <= cfg.position_tolerance_deg
    return PoseVerification(
        confirmed,
        LifecycleState.PARKED if confirmed else LifecycleState.PARK_REQUESTED,
        None if confirmed else "PARK trajectory required through MotionExecutor",
        error,
    )

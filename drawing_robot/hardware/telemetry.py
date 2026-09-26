"""Normalize RobotService telemetry into the Stage-3 RobotState contract."""

from __future__ import annotations

from ..execution import RobotState, RobotStatus
from ..robot.state import RobotMode, RobotState as ServiceState


def execution_status(
    state: ServiceState,
    *,
    moving: bool,
    stop_requested: bool,
    stop_confirmed: bool,
) -> RobotStatus:
    if state.mode is RobotMode.FAULT or state.fault:
        return RobotStatus.FAULT
    if stop_confirmed:
        return RobotStatus.STOPPED
    if stop_requested:
        return RobotStatus.STOPPING
    return RobotStatus.MOVING if moving else RobotStatus.IDLE


def normalize_state(
    state: ServiceState,
    *,
    now_s: float,
    freshness_timeout_s: float,
    commanded_angles_deg: tuple[float, ...],
    last_command_id: str | None,
    moving: bool,
    stop_requested: bool,
    stop_confirmed: bool,
) -> RobotState:
    critical_timestamp = state.critical_monotonic_s or state.monotonic_s
    age = max(0.0, now_s - critical_timestamp)
    fresh = critical_timestamp > 0 and age <= freshness_timeout_s
    return RobotState(
        timestamp_s=critical_timestamp,
        commanded_angles_deg=commanded_angles_deg,
        actual_angles_deg=state.angles_deg,
        status=execution_status(
            state,
            moving=moving,
            stop_requested=stop_requested,
            stop_confirmed=stop_confirmed,
        ),
        last_command_id=last_command_id,
        fault=state.fault,
        sample_sequence=state.sequence,
        state_age_s=age,
        is_fresh=fresh,
    )

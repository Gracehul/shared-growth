"""Project safety gates and explicit motion permissions."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .. import config
from .state import RobotState


class MotionClass(str, Enum):
    NORMAL = "normal"
    RECOVERY = "recovery"
    PARKING = "parking"


@dataclass(frozen=True)
class SafetyDecision:
    allowed: bool
    reasons: tuple[str, ...]
    alerts: tuple[dict[str, str], ...]


def evaluate_state(state: RobotState) -> SafetyDecision:
    reasons: list[str] = []
    alerts: list[dict[str, str]] = []
    if not state.connected:
        reasons.append("robot is disconnected")
    if state.controller_error not in (None, 0):
        reasons.append(f"controller error {state.controller_error}")
    for index, status in enumerate(state.servo_status, start=1):
        if status != 0:
            reasons.append(f"J{index} servo status is {status}")
    for index, temperature in enumerate(state.temperatures_c, start=1):
        if temperature >= config.TEMPERATURE_ABORT_C:
            reasons.append(f"J{index} temperature is {temperature:.0f} C")
            alerts.append({"severity": "danger", "title": f"J{index} temperature gate", "detail": reasons[-1]})
        elif temperature >= config.TEMPERATURE_WARNING_C:
            alerts.append({"severity": "warning", "title": f"J{index} temperature warning", "detail": f"J{index} is {temperature:.0f} C."})
    for index, angle in enumerate(state.angles_deg):
        low, high = config.JOINT_LIMITS_DEG[index]
        margin = min(angle - low, high - angle)
        if margin < config.JOINT_LIMIT_MARGIN_DEG:
            reasons.append(f"J{index + 1} has only {margin:.2f} deg limit margin")
    return SafetyDecision(not reasons, tuple(reasons), tuple(alerts))


def motion_permission(
    state: RobotState,
    motion_class: MotionClass = MotionClass.NORMAL,
    *,
    locally_confirmed: bool = False,
) -> SafetyDecision:
    decision = evaluate_state(state)
    reasons = list(decision.reasons)
    required = {
        "joint angles": state.angles_deg,
        "temperatures": state.temperatures_c,
        "servo status": state.servo_status,
    }
    for name, values in required.items():
        if len(values) != config.DOF:
            reasons.append(f"complete {name} telemetry is unavailable")
    if state.controller_error is None:
        reasons.append("controller error telemetry is unavailable")
    if not state.powered:
        reasons.append("robot power is off")
    if not state.servos_enabled:
        reasons.append("servos are not enabled")
    if not locally_confirmed:
        reasons.append(f"{motion_class.value} motion requires local confirmation")
    return SafetyDecision(not reasons, tuple(reasons), decision.alerts)

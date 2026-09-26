"""Stage 4A conservative real-robot integration."""

from .config import Limit, Stage4AConfig
from .errors import (
    CommunicationLost,
    HardwareIntegrationError,
    PreflightRejected,
    StaleTelemetry,
    StopFailed,
    TemperatureAbort,
)
from .lifecycle import LifecycleState, PoseVerification, verify_park, verify_ready
from .mycobot_robot import MyCobotRobot
from .safety_gate import GateDecision, motion_envelope_gate, preflight_gate, runtime_gate

__all__ = [
    "CommunicationLost", "GateDecision", "HardwareIntegrationError",
    "LifecycleState", "Limit", "MyCobotRobot", "PoseVerification",
    "PreflightRejected", "Stage4AConfig", "StaleTelemetry", "StopFailed",
    "TemperatureAbort", "motion_envelope_gate", "preflight_gate",
    "runtime_gate", "verify_park", "verify_ready",
]

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
from .evidence import LogEvidence, UartTimingSummary, analyze_execution_log
from .observability import SystemSnapshot, SystemSnapshotAdapter
from .policy import (
    ExecutionPhase,
    GateCategory,
    GateResult,
    GateStatus,
    ThresholdMetadata,
)
from .safety_gate import (
    GateDecision,
    freshness_gate_result,
    motion_envelope_gate,
    preflight_gate,
    runtime_gate,
    telemetry_gate_results,
    validation_gate_result,
)

__all__ = [
    "CommunicationLost", "ExecutionPhase", "GateCategory", "GateDecision",
    "GateResult", "GateStatus", "HardwareIntegrationError", "LifecycleState",
    "Limit", "LogEvidence", "MyCobotRobot", "PoseVerification",
    "PreflightRejected", "Stage4AConfig", "StaleTelemetry", "StopFailed",
    "SystemSnapshot", "SystemSnapshotAdapter", "TemperatureAbort",
    "ThresholdMetadata", "UartTimingSummary", "analyze_execution_log",
    "freshness_gate_result", "motion_envelope_gate", "preflight_gate",
    "runtime_gate", "telemetry_gate_results", "validation_gate_result",
    "verify_park", "verify_ready",
]

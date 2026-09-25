"""Immutable state published by the robot service."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from enum import Enum


class RobotMode(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    INITIALIZING = "INITIALIZING"
    READY = "READY"
    EXECUTING = "EXECUTING"
    STOPPING = "STOPPING"
    FAULT = "FAULT"
    PARKING = "PARKING"
    PARKED = "PARKED"


@dataclass(frozen=True)
class RobotState:
    timestamp_utc: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat()
    )
    monotonic_s: float = 0.0
    sequence: int = 0
    mode: RobotMode = RobotMode.DISCONNECTED
    connected: bool = False
    powered: bool | None = None
    servos_enabled: bool | None = None
    angles_deg: tuple[float, ...] = ()
    speeds: tuple[float, ...] = ()
    temperatures_c: tuple[float, ...] = ()
    voltages_v: tuple[float, ...] = ()
    servo_status: tuple[int, ...] = ()
    firmware_pose_mm_deg: tuple[float, ...] = ()
    controller_error: int | None = None
    alerts: tuple[dict[str, str], ...] = ()
    fault: str | None = None

    def to_dict(self) -> dict[str, object]:
        value = asdict(self)
        value["mode"] = self.mode.value
        return value

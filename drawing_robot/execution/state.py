"""Execution-layer state with mutually exclusive status values."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Sequence


class RobotStatus(str, Enum):
    IDLE = "IDLE"
    MOVING = "MOVING"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    FAULT = "FAULT"


@dataclass(frozen=True)
class RobotState:
    timestamp_s: float
    commanded_angles_deg: tuple[float, ...]
    actual_angles_deg: tuple[float, ...]
    status: RobotStatus
    last_command_id: str | None = None
    fault: str | None = None

    def __init__(
        self,
        timestamp_s: float,
        commanded_angles_deg: Sequence[float],
        actual_angles_deg: Sequence[float],
        status: RobotStatus,
        last_command_id: str | None = None,
        fault: str | None = None,
    ) -> None:
        object.__setattr__(self, "timestamp_s", float(timestamp_s))
        object.__setattr__(
            self, "commanded_angles_deg", tuple(float(v) for v in commanded_angles_deg)
        )
        object.__setattr__(
            self, "actual_angles_deg", tuple(float(v) for v in actual_angles_deg)
        )
        object.__setattr__(self, "status", RobotStatus(status))
        object.__setattr__(self, "last_command_id", last_command_id)
        object.__setattr__(self, "fault", fault)

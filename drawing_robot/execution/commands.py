"""Structured commands and transport acknowledgements for Stage 3."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class JointCommand:
    command_id: str
    target_angles_deg: tuple[float, ...]
    scheduled_time_s: float

    def __init__(
        self,
        command_id: str,
        target_angles_deg: Sequence[float],
        scheduled_time_s: float,
    ) -> None:
        object.__setattr__(self, "command_id", str(command_id))
        object.__setattr__(
            self, "target_angles_deg", tuple(float(v) for v in target_angles_deg)
        )
        object.__setattr__(self, "scheduled_time_s", float(scheduled_time_s))


@dataclass(frozen=True)
class CommandAcknowledgement:
    accepted: bool
    backend_timestamp_s: float
    command_id: str
    error: str | None = None

"""Small shared vocabulary for runtime policy and qualification evidence."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any


class GateCategory(str, Enum):
    HARD = "HARD"
    PERFORMANCE = "PERFORMANCE"


class GateStatus(str, Enum):
    PASS = "PASS"
    WARN = "WARN"
    BLOCK = "BLOCK"
    UNKNOWN = "UNKNOWN"


class ExecutionPhase(str, Enum):
    IDLE = "IDLE"
    PRECHECK = "PRECHECK"
    READY_CHECK = "READY_CHECK"
    RUNNING = "RUNNING"
    SETTLING = "SETTLING"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class ThresholdMetadata:
    value: float
    unit: str
    source: str
    status: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class GateResult:
    name: str
    category: GateCategory
    status: GateStatus
    value: Any = None
    limit: Any = None
    unit: str | None = None
    reason: str | None = None

    @property
    def hard_blocking(self) -> bool:
        return self.category is GateCategory.HARD and self.status in {
            GateStatus.BLOCK,
            GateStatus.UNKNOWN,
        }

    @property
    def qualification_miss(self) -> bool:
        return self.category is GateCategory.PERFORMANCE and self.status in {
            GateStatus.BLOCK,
            GateStatus.UNKNOWN,
        }

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["category"] = self.category.value
        value["status"] = self.status.value
        return value


def hard_result(
    name: str,
    status: GateStatus,
    *,
    value: Any = None,
    limit: Any = None,
    unit: str | None = None,
    reason: str | None = None,
) -> GateResult:
    return GateResult(name, GateCategory.HARD, status, value, limit, unit, reason)


def performance_result(
    name: str,
    status: GateStatus,
    *,
    value: Any = None,
    limit: Any = None,
    unit: str | None = None,
    reason: str | None = None,
) -> GateResult:
    return GateResult(
        name, GateCategory.PERFORMANCE, status, value, limit, unit, reason
    )


def tolerance_result(
    name: str,
    value: float | None,
    limit: float,
    *,
    unit: str,
    reason: str,
) -> GateResult:
    if value is None:
        return performance_result(
            name,
            GateStatus.UNKNOWN,
            limit=limit,
            unit=unit,
            reason=f"{reason}: value unavailable",
        )
    return performance_result(
        name,
        GateStatus.PASS if value <= limit else GateStatus.BLOCK,
        value=float(value),
        limit=float(limit),
        unit=unit,
        reason=None if value <= limit else reason,
    )

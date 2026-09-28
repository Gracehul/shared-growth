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
    BLOCK = "BLOCK"
    NOMINAL = "NOMINAL"
    WARN = "WARN"
    DEGRADED = "DEGRADED"


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

    def __post_init__(self) -> None:
        category = GateCategory(self.category)
        status = GateStatus(self.status)
        valid = {
            GateCategory.HARD: {GateStatus.PASS, GateStatus.BLOCK},
            GateCategory.PERFORMANCE: {
                GateStatus.NOMINAL,
                GateStatus.WARN,
                GateStatus.DEGRADED,
            },
        }
        if status not in valid[category]:
            raise ValueError(
                f"invalid gate category/status combination: "
                f"{category.value}/{status.value}"
            )
        object.__setattr__(self, "category", category)
        object.__setattr__(self, "status", status)

    @property
    def hard_blocking(self) -> bool:
        return self.category is GateCategory.HARD and self.status is GateStatus.BLOCK

    @property
    def performance_non_nominal(self) -> bool:
        return self.category is GateCategory.PERFORMANCE and self.status in {
            GateStatus.WARN,
            GateStatus.DEGRADED,
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
            GateStatus.DEGRADED,
            limit=limit,
            unit=unit,
            reason=f"{reason}: value unavailable",
        )
    return performance_result(
        name,
        GateStatus.NOMINAL if value <= limit else GateStatus.DEGRADED,
        value=float(value),
        limit=float(limit),
        unit=unit,
        reason=None if value <= limit else reason,
    )

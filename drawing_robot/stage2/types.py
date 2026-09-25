"""Hardware-independent trajectory and validation data models."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Iterable, Sequence


class Severity(str, Enum):
    ERROR = "ERROR"
    WARNING = "WARNING"
    INFO = "INFO"


@dataclass(frozen=True)
class CartesianSample:
    time_s: float
    pose_mm_deg: tuple[float, ...]

    def __init__(self, time_s: float, pose_mm_deg: Sequence[float]):
        object.__setattr__(self, "time_s", float(time_s))
        object.__setattr__(self, "pose_mm_deg", tuple(float(v) for v in pose_mm_deg))


@dataclass(frozen=True)
class JointSample:
    time_s: float
    positions_deg: tuple[float, ...]

    def __init__(self, time_s: float, positions_deg: Sequence[float]):
        object.__setattr__(self, "time_s", float(time_s))
        object.__setattr__(self, "positions_deg", tuple(float(v) for v in positions_deg))


@dataclass(frozen=True)
class CartesianTrajectory:
    samples: tuple[CartesianSample, ...]

    def __init__(self, samples: Iterable[CartesianSample]):
        object.__setattr__(self, "samples", tuple(samples))

    @classmethod
    def from_arrays(
        cls, times_s: Sequence[float], poses_mm_deg: Sequence[Sequence[float]]
    ) -> "CartesianTrajectory":
        if len(times_s) != len(poses_mm_deg):
            raise ValueError("times and poses must contain the same number of samples")
        return cls(CartesianSample(t, p) for t, p in zip(times_s, poses_mm_deg))


@dataclass(frozen=True)
class JointTrajectory:
    samples: tuple[JointSample, ...]

    def __init__(self, samples: Iterable[JointSample]):
        object.__setattr__(self, "samples", tuple(samples))

    @classmethod
    def from_arrays(
        cls, times_s: Sequence[float], positions_deg: Sequence[Sequence[float]]
    ) -> "JointTrajectory":
        if len(times_s) != len(positions_deg):
            raise ValueError("times and positions must contain the same number of samples")
        return cls(JointSample(t, q) for t, q in zip(times_s, positions_deg))


@dataclass(frozen=True)
class ValidationIssue:
    code: str
    severity: Severity
    message: str
    sample_index: int | None = None
    joint_index: int | None = None


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    errors: tuple[ValidationIssue, ...] = ()
    warnings: tuple[ValidationIssue, ...] = ()
    metrics: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_issues(
        cls, issues: Iterable[ValidationIssue], metrics: dict[str, Any] | None = None
    ) -> "ValidationResult":
        issues = tuple(issues)
        errors = tuple(i for i in issues if i.severity is Severity.ERROR)
        warnings = tuple(i for i in issues if i.severity is Severity.WARNING)
        return cls(not errors, errors, warnings, metrics or {})


class TrajectoryRejected(RuntimeError):
    """Raised by a caller when a validation result rejects a trajectory."""

    def __init__(self, result: ValidationResult):
        self.result = result
        codes = ", ".join(issue.code for issue in result.errors)
        super().__init__(f"trajectory rejected: {codes or 'unspecified validation error'}")


class SequentialIKError(RuntimeError):
    """Structured failure produced while converting Cartesian samples."""

    def __init__(self, issue: ValidationIssue):
        self.issue = issue
        super().__init__(f"{issue.code}: {issue.message}")

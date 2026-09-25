"""Configuration used exclusively by the offline Stage 2 gate."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Generic, TypeVar


T = TypeVar("T")


@dataclass(frozen=True)
class LimitValue(Generic[T]):
    value: T
    unit: str
    source: str
    verified: bool


@dataclass(frozen=True)
class WorkspaceBounds:
    x_mm: tuple[float, float]
    y_mm: tuple[float, float]
    z_mm: tuple[float, float]

    def contains(self, x: float, y: float, z: float) -> bool:
        return (
            self.x_mm[0] <= x <= self.x_mm[1]
            and self.y_mm[0] <= y <= self.y_mm[1]
            and self.z_mm[0] <= z <= self.z_mm[1]
        )


PROJECT_JOINT_LIMITS_DEG = (
    (-165.0, 165.0),
    (-137.0, 137.0),
    (-147.0, 147.0),
    (-147.0, 147.0),
    (-152.0, 157.0),
    (-177.0, 177.0),
)


@dataclass(frozen=True)
class Stage2Config:
    joint_limits: LimitValue[tuple[tuple[float, float], ...]] = field(
        default_factory=lambda: LimitValue(
            PROJECT_JOINT_LIMITS_DEG, "deg", "firmware_with_3_deg_project_margin", True
        )
    )
    joint_margin_deg: LimitValue[float] = field(
        default_factory=lambda: LimitValue(3.0, "deg", "project_assumption", False)
    )
    max_joint_velocity: LimitValue[tuple[float, ...]] = field(
        default_factory=lambda: LimitValue(
            (250.0, 250.0, 250.0, 300.0, 300.0, 350.0),
            "deg/s",
            "project_assumption",
            False,
        )
    )
    max_joint_acceleration: LimitValue[tuple[float, ...]] = field(
        default_factory=lambda: LimitValue(
            (500.0, 500.0, 500.0, 600.0, 600.0, 700.0),
            "deg/s^2",
            "project_assumption",
            False,
        )
    )
    max_joint_step_deg: LimitValue[float] = field(
        default_factory=lambda: LimitValue(5.0, "deg", "project_assumption", False)
    )
    workspace_bounds: LimitValue[WorkspaceBounds | None] = field(
        default_factory=lambda: LimitValue(None, "mm", "not_calibrated", False)
    )
    fk_position_tolerance_mm: LimitValue[float] = field(
        # The underlying IK stops when each Cartesian component is below
        # 0.5 mm; a 1 mm Euclidean reconstruction gate covers that component
        # convention without pretending to be a calibration tolerance.
        default_factory=lambda: LimitValue(1.0, "mm", "project_assumption", False)
    )
    fk_orientation_tolerance_deg: LimitValue[float] = field(
        default_factory=lambda: LimitValue(0.5, "deg", "project_assumption", False)
    )
    minimum_dt: LimitValue[float] = field(
        default_factory=lambda: LimitValue(0.02, "s", "project_assumption", False)
    )
    max_cartesian_step_mm: LimitValue[float] = field(
        default_factory=lambda: LimitValue(3.0, "mm", "project_assumption", False)
    )
    singularity_warning_threshold: LimitValue[float] = field(
        default_factory=lambda: LimitValue(0.05, "mixed_jacobian_units", "project_assumption", False)
    )
    tool_transform_verified: bool = False

    def assumptions(self) -> tuple[str, ...]:
        names = (
            "joint_margin_deg",
            "max_joint_velocity",
            "max_joint_acceleration",
            "max_joint_step_deg",
            "workspace_bounds",
            "fk_position_tolerance_mm",
            "fk_orientation_tolerance_deg",
            "minimum_dt",
            "max_cartesian_step_mm",
            "singularity_warning_threshold",
        )
        return tuple(name for name in names if not getattr(self, name).verified)

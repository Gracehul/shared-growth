"""Central conservative configuration for real Stage-4A execution."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import isfinite

from .. import config as project_config


@dataclass(frozen=True)
class Limit:
    value: float
    unit: str
    source: str = "project_assumption"
    verified: bool = False


@dataclass(frozen=True)
class Stage4AConfig:
    profile: str = "stage4a_conservative"
    firmware_speed: int = 5
    max_joint_velocity: Limit = Limit(15.0, "deg/s")
    max_joint_acceleration: Limit = Limit(60.0, "deg/s^2")
    max_joint_step: Limit = Limit(2.0, "deg")
    max_trajectory_duration: Limit = Limit(15.0, "s")
    position_tolerance_deg: float = 2.0
    start_tolerance_deg: float = 2.0
    telemetry_freshness_timeout_s: float = 0.5
    stop_confirmation_timeout_s: float = 2.0
    stopped_speed_threshold: float = 1.0
    stopped_samples_required: int = 2
    temperature_warning_c: float = project_config.TEMPERATURE_WARNING_C
    temperature_abort_c: float = project_config.TEMPERATURE_ABORT_C
    ready_angles_deg: tuple[float, ...] | None = project_config.READY_ANGLES_DEG
    park_angles_deg: tuple[float, ...] | None = project_config.PARK_ANGLES_DEG
    allowed_workspace_mm: tuple[tuple[float, float], ...] | None = (
        (180.0, 260.0),
        (-80.0, 80.0),
        (180.0, 320.0),
    )
    workspace_source: str = "project provisional"
    workspace_verified: bool = False

    def __post_init__(self) -> None:
        if not 1 <= self.firmware_speed <= 100:
            raise ValueError("firmware_speed must be in [1, 100]")
        for name in (
            "position_tolerance_deg", "start_tolerance_deg",
            "telemetry_freshness_timeout_s", "stop_confirmation_timeout_s",
            "stopped_speed_threshold",
        ):
            value = float(getattr(self, name))
            if not isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.stopped_samples_required < 1:
            raise ValueError("stopped_samples_required must be positive")
        for pose_name in ("ready_angles_deg", "park_angles_deg"):
            pose = getattr(self, pose_name)
            if pose is not None and (len(pose) != 6 or not all(isfinite(v) for v in pose)):
                raise ValueError(f"{pose_name} must contain six finite angles")
        if self.allowed_workspace_mm is not None:
            if len(self.allowed_workspace_mm) != 3 or any(
                len(bounds) != 2
                or not all(isfinite(v) for v in bounds)
                or bounds[0] >= bounds[1]
                for bounds in self.allowed_workspace_mm
            ):
                raise ValueError("allowed_workspace_mm must contain increasing XYZ bounds")

    def to_dict(self) -> dict[str, object]:
        return asdict(self)

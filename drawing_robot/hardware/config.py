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
    position_tolerance_deg: float = 0.5
    start_tolerance_deg: float = 1.0
    telemetry_freshness_timeout_s: float = 0.5
    stop_confirmation_timeout_s: float = 2.0
    stopped_speed_threshold: float = 1.0
    stopped_samples_required: int = 2
    temperature_warning_c: float = project_config.TEMPERATURE_WARNING_C
    temperature_abort_c: float = project_config.TEMPERATURE_ABORT_C
    ready_angles_deg: tuple[float, ...] | None = (
        1.05,
        50.71,
        -70.75,
        -63.01,
        -2.98,
        9.58,
    )
    ready_source: str = "measured_read_only_56cd257_provisional"
    ready_verified: bool = False
    park_angles_deg: tuple[float, ...] | None = project_config.PARK_ANGLES_DEG
    allowed_workspace_mm: tuple[tuple[float, float], ...] | None = (
        (20.0, 80.0),
        (-100.0, -40.0),
        (140.0, 190.0),
    )
    workspace_source: str = "project provisional"
    workspace_verified: bool = False
    ready_settling_samples: int = 5
    verification_sample_period_s: float = 0.1
    probe_joint: int = 6
    probe_displacement_deg: float = 2.0
    probe_duration_s: float = 2.0

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
        if self.ready_settling_samples < 2:
            raise ValueError("ready_settling_samples must be at least two")
        if self.verification_sample_period_s <= 0:
            raise ValueError("verification_sample_period_s must be positive")
        if not 1 <= self.probe_joint <= 6:
            raise ValueError("probe_joint must be in [1, 6]")
        if not isfinite(self.probe_displacement_deg) or self.probe_displacement_deg == 0:
            raise ValueError("probe_displacement_deg must be finite and non-zero")
        if not isfinite(self.probe_duration_s) or self.probe_duration_s <= 0:
            raise ValueError("probe_duration_s must be positive")
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

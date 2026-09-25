"""Central Stage-3 simulation and executor configuration."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from math import isfinite


class LatencyMode(str, Enum):
    IDEAL = "ideal"
    FIXED = "fixed"
    JITTERED = "jittered"


@dataclass(frozen=True)
class CommandLatencyConfig:
    mode: LatencyMode = LatencyMode.IDEAL
    base_delay_s: float = 0.0
    jitter_min_s: float = 0.0
    jitter_max_s: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "mode", LatencyMode(self.mode))


@dataclass(frozen=True)
class FaultSettings:
    dropped_command_ids: tuple[str, ...] = ()
    stale_state_at_s: float | None = None
    fault_at_s: float | None = None
    fault_message: str = "simulated backend fault"


@dataclass(frozen=True)
class ExecutionConfig:
    simulation_timestep_s: float = 0.01
    state_update_rate_hz: float = 20.0
    joint_velocity_limits_deg_s: tuple[float, ...] = (
        60.0,
        60.0,
        60.0,
        80.0,
        80.0,
        100.0,
    )
    position_tolerance_deg: float = 0.1
    command_latency: CommandLatencyConfig = field(default_factory=CommandLatencyConfig)
    stop_delay_s: float = 0.1
    state_freshness_timeout_s: float = 0.25
    trajectory_completion_timeout_s: float = 5.0
    faults: FaultSettings = field(default_factory=FaultSettings)
    random_seed: int = 0

    def __post_init__(self) -> None:
        if not isfinite(self.simulation_timestep_s) or self.simulation_timestep_s <= 0:
            raise ValueError("simulation_timestep_s must be positive")
        if not isfinite(self.state_update_rate_hz) or self.state_update_rate_hz <= 0:
            raise ValueError("state_update_rate_hz must be positive")
        if len(self.joint_velocity_limits_deg_s) != 6 or any(
            not isfinite(value) or value <= 0
            for value in self.joint_velocity_limits_deg_s
        ):
            raise ValueError("joint velocity limits must contain six positive values")
        if not isfinite(self.position_tolerance_deg) or self.position_tolerance_deg < 0:
            raise ValueError("position_tolerance_deg must be non-negative")
        if not isfinite(self.stop_delay_s) or self.stop_delay_s < 0:
            raise ValueError("stop_delay_s must be non-negative")
        if (
            not isfinite(self.state_freshness_timeout_s)
            or self.state_freshness_timeout_s <= 0
        ):
            raise ValueError("state freshness timeout must be positive")
        if (
            not isfinite(self.trajectory_completion_timeout_s)
            or self.trajectory_completion_timeout_s <= 0
        ):
            raise ValueError("completion timeout must be positive")
        latency = self.command_latency
        latency_values = (
            latency.base_delay_s,
            latency.jitter_min_s,
            latency.jitter_max_s,
        )
        if (
            not all(isfinite(value) for value in latency_values)
            or latency.base_delay_s < 0
            or latency.jitter_min_s > latency.jitter_max_s
        ):
            raise ValueError("invalid command latency configuration")
        if latency.mode is LatencyMode.JITTERED and (
            latency.base_delay_s + latency.jitter_min_s < 0
        ):
            raise ValueError("jittered command latency must never be negative")
        for name in ("stale_state_at_s", "fault_at_s"):
            value = getattr(self.faults, name)
            if value is not None and (not isfinite(value) or value < 0):
                raise ValueError(f"{name} must be finite and non-negative")

    def to_dict(self) -> dict[str, object]:
        value = asdict(self)
        value["command_latency"]["mode"] = self.command_latency.mode.value
        return value

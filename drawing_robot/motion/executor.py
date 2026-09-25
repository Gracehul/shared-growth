"""Translate an offline joint plan into scheduled RobotService commands."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable, Sequence

from .. import config
from ..kinematics import check_joint_limits
from ..robot.service import RobotService


@dataclass(frozen=True)
class CommandedWaypoint:
    index: int
    planned_s: float
    issued_s: float
    returned_s: float
    target_angles_deg: tuple[float, ...]


@dataclass(frozen=True)
class ExecutionReport:
    started_s: float
    finished_s: float
    planned_waypoints: int
    commanded: tuple[CommandedWaypoint, ...]


class MotionExecutor:
    """Own timing policy while RobotService owns transport serialization."""

    def __init__(self, service: RobotService, *, command_rate_hz: float = config.COMMAND_RATE_HZ):
        if command_rate_hz <= 0:
            raise ValueError("command_rate_hz must be positive")
        self.service = service
        self.command_rate_hz = float(command_rate_hz)

    def execute_joint_plan(
        self,
        waypoints_deg: Iterable[Sequence[float]],
        timestamps_s: Iterable[float],
        *,
        firmware_speed: int,
        auto_disarm: bool = True,
    ) -> ExecutionReport:
        waypoints = [tuple(float(v) for v in q) for q in waypoints_deg]
        timestamps = [float(v) for v in timestamps_s]
        self._validate(waypoints, timestamps, firmware_speed)
        started = time.monotonic()
        commanded: list[CommandedWaypoint] = []
        try:
            for index, (target, planned_s) in enumerate(zip(waypoints, timestamps, strict=True)):
                deadline = started + planned_s
                remaining = deadline - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                issued = time.monotonic()
                self.service.send_angles(list(target), firmware_speed, _async=True)
                returned = time.monotonic()
                commanded.append(CommandedWaypoint(index, planned_s, issued, returned, target))
            return ExecutionReport(started, time.monotonic(), len(waypoints), tuple(commanded))
        except Exception:
            self.service.stop_motion()
            raise
        finally:
            if auto_disarm:
                self.service.disarm_motion()

    def _validate(
        self,
        waypoints: list[tuple[float, ...]],
        timestamps: list[float],
        firmware_speed: int,
    ) -> None:
        if not waypoints or len(waypoints) != len(timestamps):
            raise ValueError("waypoints and timestamps must be non-empty and equally sized")
        if timestamps[0] < 0 or any(b <= a for a, b in zip(timestamps, timestamps[1:])):
            raise ValueError("timestamps must be strictly increasing")
        minimum_step = 1.0 / self.command_rate_hz
        if any(b - a < minimum_step - 1e-9 for a, b in zip(timestamps, timestamps[1:])):
            raise ValueError("plan exceeds configured command rate")
        if not config.FIRMWARE_SPEED_MIN <= firmware_speed <= config.FIRMWARE_SPEED_MAX:
            raise ValueError("firmware_speed outside configured range")
        for index, target in enumerate(waypoints):
            problems = check_joint_limits(target)
            if problems:
                raise ValueError(f"waypoint {index} violates joint limits: {'; '.join(problems)}")

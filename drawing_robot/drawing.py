"""Bounded drawing primitives built on the current :class:`Arm` API.

All public positions are centimetres in the robot base frame. A caller must
provide explicit XY bounds and calibrated drawing/safe heights; this module
does not treat the placeholder values in ``config.py`` as hardware approval.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, Sequence

from .arm import Arm, SUCCESS

Point = tuple[float, float]
Segment = tuple[Point, Point]


class DrawingError(RuntimeError):
    """A drawing request was invalid or a robot command failed."""


def _finite_pair(point: Sequence[float]) -> Point:
    if len(point) != 2:
        raise ValueError(f"expected an (x, y) point, got {point!r}")
    x, y = float(point[0]), float(point[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError(f"point must contain finite values, got {point!r}")
    return x, y


@dataclass(frozen=True)
class DrawingWorkspace:
    """Calibrated rectangular drawing area in robot-frame centimetres."""

    x_min_cm: float
    x_max_cm: float
    y_min_cm: float
    y_max_cm: float
    drawing_z_cm: float
    safe_z_cm: float

    def __post_init__(self) -> None:
        values = (
            self.x_min_cm,
            self.x_max_cm,
            self.y_min_cm,
            self.y_max_cm,
            self.drawing_z_cm,
            self.safe_z_cm,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("workspace values must be finite")
        if self.x_min_cm >= self.x_max_cm:
            raise ValueError("x_min_cm must be smaller than x_max_cm")
        if self.y_min_cm >= self.y_max_cm:
            raise ValueError("y_min_cm must be smaller than y_max_cm")
        if self.safe_z_cm <= self.drawing_z_cm:
            raise ValueError("safe_z_cm must be above drawing_z_cm")

    def validate(self, points: Iterable[Sequence[float]]) -> list[Point]:
        checked: list[Point] = []
        for point in points:
            x, y = _finite_pair(point)
            if not self.x_min_cm <= x <= self.x_max_cm:
                raise DrawingError(
                    f"x={x:g} cm is outside [{self.x_min_cm:g}, {self.x_max_cm:g}]"
                )
            if not self.y_min_cm <= y <= self.y_max_cm:
                raise DrawingError(
                    f"y={y:g} cm is outside [{self.y_min_cm:g}, {self.y_max_cm:g}]"
                )
            checked.append((x, y))
        if not checked:
            raise DrawingError("at least one point is required")
        return checked


def branch_points(
    origin: Sequence[float],
    direction: Sequence[float],
    length_cm: float,
    *,
    split_fraction: float = 0.6,
    arm_length_cm: float | None = None,
    arm_angle_deg: float = 30.0,
) -> tuple[Point, Point, Point, Point]:
    """Return origin, split, left end, and right end for a Y-branch."""

    ox, oy = _finite_pair(origin)
    dx, dy = _finite_pair(direction)
    magnitude = math.hypot(dx, dy)
    if magnitude == 0:
        raise ValueError("direction must be nonzero")
    length_cm = float(length_cm)
    if not math.isfinite(length_cm) or length_cm <= 0:
        raise ValueError("length_cm must be positive and finite")
    if not 0 < split_fraction < 1:
        raise ValueError("split_fraction must be between 0 and 1")
    if not 0 < arm_angle_deg < 180:
        raise ValueError("arm_angle_deg must be between 0 and 180")

    ux, uy = dx / magnitude, dy / magnitude
    split = (
        ox + ux * length_cm * split_fraction,
        oy + uy * length_cm * split_fraction,
    )
    if arm_length_cm is None:
        arm_length_cm = length_cm * (1 - split_fraction) * 1.2
    arm_length_cm = float(arm_length_cm)
    if not math.isfinite(arm_length_cm) or arm_length_cm <= 0:
        raise ValueError("arm_length_cm must be positive and finite")

    angle = math.radians(arm_angle_deg)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    left = (ux * cos_a - uy * sin_a, ux * sin_a + uy * cos_a)
    right = (ux * cos_a + uy * sin_a, -ux * sin_a + uy * cos_a)
    left_end = (
        split[0] + left[0] * arm_length_cm,
        split[1] + left[1] * arm_length_cm,
    )
    right_end = (
        split[0] + right[0] * arm_length_cm,
        split[1] + right[1] * arm_length_cm,
    )
    return (ox, oy), split, left_end, right_end


def branch_segments(*args, **kwargs) -> tuple[Segment, Segment, Segment]:
    """Return the stem and two arms as separate pen strokes."""

    origin, split, left_end, right_end = branch_points(*args, **kwargs)
    return (origin, split), (split, left_end), (split, right_end)


class DrawingController:
    """Execute validated drawing strokes through an :class:`Arm` instance."""

    def __init__(
        self,
        arm: Arm,
        workspace: DrawingWorkspace,
        *,
        speed_cm_s: float = 1.0,
    ) -> None:
        if not math.isfinite(speed_cm_s) or speed_cm_s <= 0:
            raise ValueError("speed_cm_s must be positive and finite")
        self.arm = arm
        self.workspace = workspace
        self.speed_cm_s = float(speed_cm_s)

    def _require(self, result: int, action: str) -> None:
        if result == SUCCESS:
            return
        self.arm.stop()
        detail = self.arm.last_error or "unknown robot-control failure"
        raise DrawingError(f"{action} failed: {detail}")

    def pen_up(self) -> None:
        self._require(
            self.arm.send_coords(z=self.workspace.safe_z_cm, speed=self.speed_cm_s),
            "pen-up move",
        )

    def pen_down(self) -> None:
        self._require(
            self.arm.send_coords(z=self.workspace.drawing_z_cm, speed=self.speed_cm_s),
            "pen-down move",
        )

    def draw_polyline(self, points: Iterable[Sequence[float]]) -> None:
        """Draw one continuous stroke and lift the pen after success.

        The complete XY input is checked against the workspace before the first
        motion. If any robot command fails, execution stops and raises instead
        of attempting an automatic recovery move.
        """

        checked = self.workspace.validate(points)
        first_x, first_y = checked[0]
        self._require(
            self.arm.send_coords(
                x=first_x,
                y=first_y,
                z=self.workspace.safe_z_cm,
                speed=self.speed_cm_s,
            ),
            "safe approach",
        )
        self.pen_down()

        if len(checked) > 1:
            self._require(
                self.arm.send_path(
                    x=[point[0] for point in checked[1:]],
                    y=[point[1] for point in checked[1:]],
                    z=self.workspace.drawing_z_cm,
                    speed=self.speed_cm_s,
                ),
                "stroke",
            )
        self.pen_up()

    def draw_point(self, point: Sequence[float], *, dwell_s: float = 0.2) -> None:
        if not math.isfinite(dwell_s) or dwell_s < 0:
            raise ValueError("dwell_s must be non-negative and finite")
        checked = self.workspace.validate([point])
        x, y = checked[0]
        self._require(
            self.arm.send_coords(
                x=x,
                y=y,
                z=self.workspace.safe_z_cm,
                speed=self.speed_cm_s,
            ),
            "safe approach",
        )
        self.pen_down()
        if dwell_s:
            time.sleep(dwell_s)
        self.pen_up()

    def draw_line(self, start: Sequence[float], end: Sequence[float]) -> None:
        self.draw_polyline([start, end])

    def draw_curve(self, points: Iterable[Sequence[float]]) -> None:
        checked = list(points)
        if len(checked) < 2:
            raise DrawingError("draw_curve requires at least two points")
        self.draw_polyline(checked)

    def draw_branch(self, *args, **kwargs) -> None:
        segments = branch_segments(*args, **kwargs)
        self.workspace.validate(point for segment in segments for point in segment)
        for start, end in segments:
            self.draw_line(start, end)

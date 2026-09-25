"""Pure geometry mapping from one human stroke to a robot branch."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..drawing import DrawingWorkspace, Point, Segment, branch_segments


@dataclass(frozen=True)
class HumanStroke:
    start_cm: Point
    end_cm: Point
    started_s: float
    ended_s: float

    def __post_init__(self) -> None:
        values = (*self.start_cm, *self.end_cm, self.started_s, self.ended_s)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("stroke values must be finite")
        if self.ended_s <= self.started_s:
            raise ValueError("stroke end must follow stroke start")
        if self.length_cm <= 0:
            raise ValueError("stroke length must be positive")

    @property
    def duration_s(self) -> float:
        return self.ended_s - self.started_s

    @property
    def length_cm(self) -> float:
        return math.dist(self.start_cm, self.end_cm)

    @property
    def direction(self) -> Point:
        length = self.length_cm
        return (
            (self.end_cm[0] - self.start_cm[0]) / length,
            (self.end_cm[1] - self.start_cm[1]) / length,
        )

    @property
    def mean_speed_cm_s(self) -> float:
        return self.length_cm / self.duration_s


@dataclass(frozen=True)
class GrowthResponse:
    source: HumanStroke
    segments_cm: tuple[Segment, Segment, Segment]
    response_length_cm: float
    rule_version: str = "shared-growth-rule/v1"


class GrowthRule:
    def __init__(
        self,
        *,
        length_ratio: float = 0.75,
        split_fraction: float = 0.55,
        arm_angle_deg: float = 30.0,
    ):
        if not 0 < length_ratio <= 1:
            raise ValueError("length_ratio must be within (0, 1]")
        self.length_ratio = float(length_ratio)
        self.split_fraction = float(split_fraction)
        self.arm_angle_deg = float(arm_angle_deg)

    def generate(
        self,
        stroke: HumanStroke,
        *,
        workspace: DrawingWorkspace | None = None,
    ) -> GrowthResponse:
        response_length = stroke.length_cm * self.length_ratio
        segments = branch_segments(
            stroke.end_cm,
            stroke.direction,
            response_length,
            split_fraction=self.split_fraction,
            arm_angle_deg=self.arm_angle_deg,
        )
        if workspace is not None:
            workspace.validate(point for segment in segments for point in segment)
        return GrowthResponse(stroke, segments, response_length)

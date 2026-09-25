"""Reproducible timing manipulation for Case A."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from enum import Enum


class TimingCondition(str, Enum):
    BASELINE = "baseline"
    FIXED_DELAY = "fixed_delay"
    JITTER = "jitter"


@dataclass(frozen=True)
class TimingDecision:
    condition: TimingCondition
    intended_delay_s: float
    seed: int | None
    sample_index: int


class TimingPolicy:
    """Sample intended delays without sleeping or touching hardware."""

    def __init__(
        self,
        condition: TimingCondition,
        *,
        fixed_delay_s: float | None = None,
        jitter_min_s: float | None = None,
        jitter_max_s: float | None = None,
        seed: int | None = None,
    ):
        self.condition = TimingCondition(condition)
        self.fixed_delay_s = fixed_delay_s
        self.jitter_min_s = jitter_min_s
        self.jitter_max_s = jitter_max_s
        self.seed = seed
        self._rng = random.Random(seed)
        self._sample_index = 0
        self._validate()

    def _validate(self) -> None:
        values = [
            value for value in (self.fixed_delay_s, self.jitter_min_s, self.jitter_max_s)
            if value is not None
        ]
        if any(not math.isfinite(value) or value < 0 for value in values):
            raise ValueError("timing values must be finite and non-negative")
        if self.condition is TimingCondition.FIXED_DELAY and self.fixed_delay_s is None:
            raise ValueError("fixed_delay requires fixed_delay_s")
        if self.condition is TimingCondition.JITTER:
            if self.jitter_min_s is None or self.jitter_max_s is None:
                raise ValueError("jitter requires jitter_min_s and jitter_max_s")
            if self.jitter_min_s > self.jitter_max_s:
                raise ValueError("jitter_min_s must not exceed jitter_max_s")

    def sample(self) -> TimingDecision:
        if self.condition is TimingCondition.BASELINE:
            delay = 0.0
        elif self.condition is TimingCondition.FIXED_DELAY:
            delay = float(self.fixed_delay_s)
        else:
            delay = self._rng.uniform(float(self.jitter_min_s), float(self.jitter_max_s))
        decision = TimingDecision(self.condition, delay, self.seed, self._sample_index)
        self._sample_index += 1
        return decision

"""Explicit deterministic clock used instead of wall-clock time."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Callable
from enum import IntEnum


class SimulationPhase(IntEnum):
    FAULTS = 10
    ACTIVATE_COMMANDS = 20
    UPDATE_STATE = 30
    PUBLISH_STATE = 40


ClockCallback = Callable[[float, float], None]


class SimulationClock:
    def __init__(self, start_time_s: float = 0.0) -> None:
        self._time_s = float(start_time_s)
        self._callbacks: dict[SimulationPhase, list[ClockCallback]] = defaultdict(list)

    def now(self) -> float:
        return self._time_s

    def register(self, phase: SimulationPhase, callback: ClockCallback) -> None:
        self._callbacks[phase].append(callback)

    def advance(self, dt_s: float) -> float:
        dt_s = float(dt_s)
        if dt_s <= 0:
            raise ValueError("clock advance must be positive")
        self._time_s += dt_s
        for phase in SimulationPhase:
            for callback in tuple(self._callbacks[phase]):
                callback(dt_s, self._time_s)
        return self._time_s

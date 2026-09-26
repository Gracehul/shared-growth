"""Explicit deterministic and monotonic real-time executor clocks."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Callable
from enum import IntEnum
import time


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


class WallClock:
    """Monotonic real-time clock for hardware execution.

    ``advance`` sleeps instead of synthesizing time, preserving the Stage-3
    executor contract without putting wall-clock access into MotionExecutor.
    """

    def now(self) -> float:
        return time.monotonic()

    def advance(self, dt_s: float) -> float:
        dt_s = float(dt_s)
        if dt_s <= 0:
            raise ValueError("clock advance must be positive")
        time.sleep(dt_s)
        return self.now()

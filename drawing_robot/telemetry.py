"""Threaded, timestamped telemetry for supervised myCobot motion tests."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any, Callable


READ_RETRIES = 4
READ_RETRY_DELAY_S = 0.03


class TelemetryFailure(RuntimeError):
    """A telemetry read or development gate failed."""


def _vector(value: object, name: str) -> list[float]:
    if not (
        isinstance(value, (list, tuple))
        and len(value) == 6
        and all(isinstance(item, (int, float)) for item in value)
    ):
        raise TelemetryFailure(f"invalid {name}: {value!r}")
    return [float(item) for item in value]


def read_vector(call: Callable[[], object], name: str) -> tuple[list[float], int]:
    last: object = None
    for attempt in range(1, READ_RETRIES + 1):
        last = call()
        try:
            return _vector(last, name), attempt
        except TelemetryFailure:
            time.sleep(READ_RETRY_DELAY_S)
    raise TelemetryFailure(f"invalid {name} after {READ_RETRIES} attempts: {last!r}")


def read_error(call: Callable[[], object]) -> tuple[int, int]:
    last: object = None
    for attempt in range(1, READ_RETRIES + 1):
        last = call()
        if isinstance(last, int) and last >= 0:
            return last, attempt
        time.sleep(READ_RETRY_DELAY_S)
    raise TelemetryFailure(f"invalid controller error after {READ_RETRIES} attempts: {last!r}")


def maximum_joint_error(actual: list[float], target: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(actual, target, strict=True))


@dataclass(frozen=True)
class CommandEvent:
    label: str
    target_angles_deg: list[float]
    speed: int
    issued_s: float
    api_returned_s: float


class TelemetryRecorder:
    """Sample one thread-safe pymycobot connection while commands are issued."""

    def __init__(
        self,
        robot: Any,
        *,
        interval_s: float = 0.05,
        health_interval_s: float = 0.5,
        maximum_temperature_c: float = 60.0,
    ):
        self.robot = robot
        self.interval_s = interval_s
        self.health_interval_s = health_interval_s
        self.maximum_temperature_c = maximum_temperature_c
        self.samples: list[dict[str, object]] = []
        self.commands: list[CommandEvent] = []
        self.failure: str | None = None
        self._latest: dict[str, object] | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._origin = 0.0

    def elapsed(self) -> float:
        return time.monotonic() - self._origin

    def start(self) -> None:
        self._origin = time.monotonic()
        self._thread = threading.Thread(target=self._run, name="motion-telemetry", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout=5.0):
            raise TelemetryFailure("telemetry did not produce an initial sample")
        self.raise_if_failed()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def latest(self) -> dict[str, object]:
        with self._lock:
            if self._latest is None:
                raise TelemetryFailure("no telemetry sample available")
            return dict(self._latest)

    def raise_if_failed(self) -> None:
        if self.failure is not None:
            raise TelemetryFailure(self.failure)

    def command(self, label: str, target: list[float], speed: int) -> CommandEvent:
        self.raise_if_failed()
        issued = self.elapsed()
        self.robot.send_angles([round(value, 2) for value in target], speed, _async=True)
        returned = self.elapsed()
        event = CommandEvent(label, list(target), speed, issued, returned)
        self.commands.append(event)
        return event

    def wait_for_target(
        self,
        target: list[float],
        *,
        initial_angles: list[float] | None = None,
        timeout_s: float = 45.0,
        tolerance_deg: float = 2.0,
        settle_samples: int = 3,
    ) -> tuple[float, float]:
        started = self.elapsed()
        first_motion: float | None = None
        initial = list(initial_angles or self.latest()["angles_deg"])
        stable = 0
        seen = len(self.samples)
        while self.elapsed() - started < timeout_s:
            self.raise_if_failed()
            sample = self.latest()
            angles = list(sample["angles_deg"])
            if len(self.samples) > seen:
                if first_motion is None and maximum_joint_error(angles, initial) >= 0.20:
                    first_motion = float(sample["elapsed_s"])
                stable = stable + 1 if maximum_joint_error(angles, target) <= tolerance_deg else 0
                seen = len(self.samples)
                if stable >= settle_samples:
                    return first_motion or started, float(sample["elapsed_s"])
            time.sleep(0.02)
        raise TelemetryFailure(f"target did not settle within {timeout_s:.1f} s")

    def hold(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.raise_if_failed()
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    def _run(self) -> None:
        health: dict[str, object] = {}
        next_health = 0.0
        try:
            while not self._stop.is_set():
                loop_started = time.monotonic()
                angles, angle_attempts = read_vector(self.robot.get_angles, "joint angles")
                error, error_attempts = read_error(self.robot.get_error_information)
                speeds, speed_attempts = read_vector(self.robot.get_servo_speeds, "servo speeds")
                now = self.elapsed()
                if now >= next_health or not health:
                    temperatures, temperature_attempts = read_vector(
                        self.robot.get_servo_temps, "servo temperatures"
                    )
                    voltages, voltage_attempts = read_vector(
                        self.robot.get_servo_voltages, "servo voltages"
                    )
                    coords, coordinate_attempts = read_vector(
                        self.robot.get_coords, "firmware pose"
                    )
                    health = {
                        "temperatures_c": temperatures,
                        "servo_voltages_v": voltages,
                        "firmware_flange_pose_mm_deg": coords,
                        "temperature_read_attempts": temperature_attempts,
                        "voltage_read_attempts": voltage_attempts,
                        "coordinate_read_attempts": coordinate_attempts,
                    }
                    next_health = now + self.health_interval_s
                sample: dict[str, object] = {
                    "elapsed_s": round(now, 6),
                    "angles_deg": angles,
                    "servo_speeds": speeds,
                    "controller_error": error,
                    "angle_read_attempts": angle_attempts,
                    "error_read_attempts": error_attempts,
                    "speed_read_attempts": speed_attempts,
                    **health,
                }
                with self._lock:
                    self.samples.append(sample)
                    self._latest = sample
                self._ready.set()
                if error != 0:
                    raise TelemetryFailure(f"controller error during motion: {error}")
                hottest = max(float(value) for value in sample["temperatures_c"])
                if hottest >= self.maximum_temperature_c:
                    raise TelemetryFailure(
                        f"project temperature gate {self.maximum_temperature_c:.1f} C reached"
                    )
                remaining = self.interval_s - (time.monotonic() - loop_started)
                self._stop.wait(max(0.0, remaining))
        except Exception as exc:
            self.failure = f"{type(exc).__name__}: {exc}"
            self._ready.set()
            try:
                self.robot.stop()
            except Exception:
                pass

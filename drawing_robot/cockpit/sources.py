"""Small read-only adapters over existing Shared Growth data sources."""

from __future__ import annotations

import glob
import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ..execution import ExecutionLog
from ..robot import RobotService
from ..robot.state import RobotState
from ..stage2 import CartesianTrajectory, JointTrajectory, ValidationResult


@dataclass(frozen=True)
class CameraSnapshot:
    connected: bool = False
    device: str = "off"
    resolution: tuple[int, int] | None = None
    effective_fps: float | None = None
    frame_monotonic_s: float | None = None
    failed_frames: int = 0
    estimated_dropped_frames: int = 0
    frame_bgr: np.ndarray | None = None
    detail: str | None = None

    def frame_age_s(self, now_s: float | None = None) -> float | None:
        if self.frame_monotonic_s is None:
            return None
        return max(0.0, (time.monotonic() if now_s is None else now_s) - self.frame_monotonic_s)


class CameraSource:
    """Observe one camera in a worker thread without touching robot control."""

    def __init__(
        self,
        device: str = "off",
        width: int = 640,
        height: int = 480,
        requested_fps: float = 30.0,
        *,
        mock: bool = False,
    ) -> None:
        self.device = device
        self.width = int(width)
        self.height = int(height)
        self.requested_fps = float(requested_fps)
        self.mock = bool(mock)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._snapshot = CameraSnapshot(device="mock" if mock else device)

    def start(self) -> None:
        if self._thread is not None:
            return
        if not self.mock and self.device == "off":
            self._snapshot = CameraSnapshot(device="off", detail="camera disabled")
            return
        self._thread = threading.Thread(target=self._run, name="cockpit-camera", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def latest(self) -> CameraSnapshot:
        with self._lock:
            value = self._snapshot
            frame = None if value.frame_bgr is None else value.frame_bgr.copy()
            return CameraSnapshot(
                value.connected,
                value.device,
                value.resolution,
                value.effective_fps,
                value.frame_monotonic_s,
                value.failed_frames,
                value.estimated_dropped_frames,
                frame,
                value.detail,
            )

    def _publish(self, value: CameraSnapshot) -> None:
        with self._lock:
            self._snapshot = value

    def _run(self) -> None:
        if self.mock:
            self._run_mock()
            return
        capture = None
        try:
            try:
                import cv2
            except ImportError as exc:
                raise RuntimeError("OpenCV is not installed; install the cockpit extra") from exc
            device = self.device
            if device == "auto":
                candidates = sorted(glob.glob("/dev/video*"))
                if not candidates:
                    raise RuntimeError("no /dev/video* camera found")
                device = candidates[0]
            capture = cv2.VideoCapture(device, getattr(cv2, "CAP_V4L2", 0))
            if not capture.isOpened():
                raise RuntimeError(f"camera could not be opened: {device}")
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            capture.set(cv2.CAP_PROP_FPS, self.requested_fps)
            if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
                capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            timestamps: deque[float] = deque(maxlen=90)
            failed = 0
            dropped = 0
            while not self._stop.is_set():
                ok, frame = capture.read()
                captured_at = time.monotonic()
                if not ok or frame is None:
                    failed += 1
                    continue
                if timestamps:
                    interval = captured_at - timestamps[-1]
                    expected = 1.0 / self.requested_fps
                    dropped += max(0, int(round(interval / expected)) - 1)
                timestamps.append(captured_at)
                duration = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0.0
                fps = (len(timestamps) - 1) / duration if duration > 0 else None
                self._publish(CameraSnapshot(
                    True,
                    str(device),
                    (int(frame.shape[1]), int(frame.shape[0])),
                    fps,
                    captured_at,
                    failed,
                    dropped,
                    frame.copy(),
                    None,
                ))
        except Exception as exc:
            self._publish(CameraSnapshot(device=self.device, detail=f"{type(exc).__name__}: {exc}"))
        finally:
            if capture is not None:
                capture.release()

    def _run_mock(self) -> None:
        period = 1.0 / min(max(self.requested_fps, 1.0), 30.0)
        started = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            phase = now - started
            x = np.linspace(0, 1, self.width, dtype=float)[None, :]
            y = np.linspace(0, 1, self.height, dtype=float)[:, None]
            frame = np.empty((self.height, self.width, 3), dtype=np.uint8)
            frame[:, :, 0] = np.clip(35 + 50 * x, 0, 255)
            frame[:, :, 1] = np.clip(35 + 80 * y, 0, 255)
            frame[:, :, 2] = np.clip(45 + 35 * (1 + math.sin(phase)) + 35 * x, 0, 255)
            self._publish(CameraSnapshot(
                True, "mock", (self.width, self.height), 1.0 / period,
                now, 0, 0, frame, "synthetic preview",
            ))
            self._stop.wait(period)


class RobotStateSource:
    """Expose the existing RobotService state; no replacement state model exists."""

    def __init__(self, service: RobotService | None = None, *, owns_service: bool = False) -> None:
        self.service = service
        self.owns_service = owns_service
        self._lock = threading.Lock()
        self._state: RobotState | None = None
        self._unsubscribe = None
        if service is not None:
            self._state = service.latest_state()
            self._unsubscribe = service.subscribe(self._consume)

    @classmethod
    def mock(cls) -> "RobotStateSource":
        service = RobotService(mock=True, read_only=True, telemetry=True)
        return cls(service, owns_service=True)

    @classmethod
    def read_only_hardware(cls, port: str, baudrate: int) -> "RobotStateSource":
        service = RobotService(
            port, baudrate, read_only=True, telemetry=True, runtime_profile=True,
            fast_hz=5.0, slow_hz=1.0,
        )
        return cls(service, owns_service=True)

    def _consume(self, state: RobotState) -> None:
        with self._lock:
            self._state = state

    def latest(self) -> RobotState | None:
        with self._lock:
            return self._state

    def close(self) -> None:
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None
        if self.owns_service and self.service is not None:
            self.service.close()
        self.service = None
        self.owns_service = False


@dataclass(frozen=True)
class ExecutionSnapshot:
    status: str = "NO RUN"
    target_angles_deg: tuple[float, ...] = ()
    result: str = "not loaded"
    fault: str | None = None
    trajectory_id: str | None = None
    events: tuple[tuple[float, str], ...] = ()


class ExecutionLogSource:
    """Read optional recorded targets/results; logs are not claimed as live state."""

    def __init__(self, path: str | Path | None = None) -> None:
        self.path = None if path is None else Path(path)
        self._mtime_ns: int | None = None
        self._snapshot = ExecutionSnapshot()

    def latest(self) -> ExecutionSnapshot:
        if self.path is None:
            return self._snapshot
        try:
            mtime = self.path.stat().st_mtime_ns
            if mtime != self._mtime_ns:
                log = ExecutionLog.read(self.path)
                target = tuple(log.commands[-1]["target_angles_deg"]) if log.commands else ()
                result = str(log.result.get("status", "unknown"))
                fault = log.result.get("reason")
                if result.lower() in {"failed", "error", "aborted"}:
                    latest_status = "FAILED"
                elif result.lower() in {"completed", "passed", "success"}:
                    latest_status = "COMPLETED"
                else:
                    latest_status = (
                        str(log.states[-1].get("status", "UNKNOWN"))
                        if log.states else result.upper()
                    )
                self._snapshot = ExecutionSnapshot(
                    latest_status,
                    target,
                    result,
                    fault,
                    log.trajectory_id,
                    tuple((event.timestamp_s, event.kind) for event in log.events[-12:]),
                )
                self._mtime_ns = mtime
        except Exception as exc:
            self._snapshot = ExecutionSnapshot("FAILED", result="log read failed", fault=str(exc))
        return self._snapshot


class TrajectorySource:
    """Own one immutable Stage-2B bundle for display only."""

    def __init__(self, path: str | Path | None = None) -> None:
        self.path = None if path is None else Path(path)
        self.cartesian: CartesianTrajectory | None = None
        self.joint: JointTrajectory | None = None
        self.validation: ValidationResult | None = None
        self.error: str | None = None
        if self.path is not None:
            self.load()

    def load(self) -> None:
        assert self.path is not None
        try:
            from ..stage2.visualization import load_visualization_bundle
            self.cartesian, self.joint, self.validation = load_visualization_bundle(self.path)
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"

    @property
    def sample_count(self) -> int:
        return 0 if self.joint is None else len(self.joint.samples)

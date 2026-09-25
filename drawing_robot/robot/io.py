"""The only module allowed to construct MyCobot280 and touch its UART."""

from __future__ import annotations

import os
import re
import threading
from pathlib import Path
from typing import Any

from .. import config


class RobotIOError(RuntimeError):
    pass


CONTROL_METHODS = {
    "focus_all_servos", "power_off", "power_on", "release_all_servos",
    "send_angle", "send_angles", "set_fresh_mode", "set_gripper_value", "stop",
}


class RobotIO:
    """Own one backend and enforce exclusive access to the physical device."""

    def __init__(
        self,
        port: str = config.DEFAULT_PORT,
        baudrate: int = config.DEFAULT_BAUDRATE,
        *,
        mock: bool = False,
        read_only: bool = False,
        backend: Any | None = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.mock = mock
        self.read_only = read_only
        self._lock = threading.RLock()
        self._lock_file = None
        if backend is None and not mock:
            self._claim_device()
        try:
            self._backend = backend if backend is not None else self._create_backend(mock)
        except Exception:
            self._release_device()
            raise

    def _create_backend(self, mock: bool):
        if mock:
            from ..mock import MockMyCobot
            return MockMyCobot()
        try:
            from pymycobot import MyCobot280
        except ImportError as exc:
            raise RobotIOError(
                "pymycobot is not installed; install the hardware extra or use mock=True"
            ) from exc
        return MyCobot280(self.port, self.baudrate)

    def _claim_device(self) -> None:
        if os.name != "posix":
            return
        import fcntl

        safe_name = re.sub(r"[^A-Za-z0-9_.-]", "_", self.port)
        path = Path("/tmp") / f"shared-growth-{safe_name}.lock"
        handle = path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            handle.close()
            raise RobotIOError(f"serial device already owned by another process: {self.port}") from exc
        handle.seek(0)
        handle.truncate()
        handle.write(str(os.getpid()))
        handle.flush()
        self._lock_file = handle

    def _release_device(self) -> None:
        if self._lock_file is None:
            return
        try:
            import fcntl
            fcntl.flock(self._lock_file.fileno(), fcntl.LOCK_UN)
        finally:
            self._lock_file.close()
            self._lock_file = None

    @property
    def backend_class_name(self) -> str:
        return type(self._backend).__name__

    def call(self, method: str, *args, **kwargs):
        if self.read_only and method in CONTROL_METHODS:
            raise RobotIOError(f"read-only robot I/O refuses {method}")
        target = getattr(self._backend, method, None)
        if target is None:
            raise RobotIOError(f"backend does not implement {method}")
        with self._lock:
            return target(*args, **kwargs)

    def close(self) -> None:
        serial_port = getattr(self._backend, "_serial_port", None)
        if serial_port is not None and hasattr(serial_port, "close"):
            try:
                serial_port.close()
            except Exception:
                pass
        self._release_device()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
        return False

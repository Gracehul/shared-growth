"""Single-threaded command and telemetry scheduler for one robot UART."""

from __future__ import annotations

import itertools
import queue
import threading
import time
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from typing import Any, Callable

from .. import config
from .io import RobotIO
from .safety import MotionClass, evaluate_state, motion_permission
from .state import RobotMode, RobotState


PRIORITY_STOP = 0
PRIORITY_MOTION = 10
PRIORITY_ESSENTIAL_TELEMETRY = 20
PRIORITY_DIAGNOSTIC_TELEMETRY = 30

READ_METHODS = {
    "get_angles", "get_basic_version", "get_coords", "get_error_information",
    "get_fresh_mode", "get_joint_max_angle", "get_joint_min_angle",
    "get_servo_speeds", "get_servo_status", "get_servo_temps",
    "get_servo_voltages", "get_system_version", "is_all_servo_enable",
    "is_power_on",
}
MOTION_METHODS = {"send_angle", "send_angles", "set_gripper_value"}
CONTROL_METHODS = {
    "focus_all_servos", "power_off", "power_on", "release_all_servos",
    "set_fresh_mode",
}


@dataclass(order=True)
class _Request:
    priority: int
    sequence: int
    method: str = field(compare=False)
    args: tuple[Any, ...] = field(compare=False, default=())
    kwargs: dict[str, Any] = field(compare=False, default_factory=dict)
    completed: threading.Event = field(compare=False, default_factory=threading.Event)
    result: Any = field(compare=False, default=None)
    error: BaseException | None = field(compare=False, default=None)
    queued_at_s: float = field(compare=False, default_factory=time.monotonic)


def _vector(value: object, name: str) -> tuple[float, ...]:
    if not (
        isinstance(value, (list, tuple))
        and len(value) == config.DOF
        and all(isinstance(item, (int, float)) for item in value)
    ):
        raise RuntimeError(f"invalid {name}: {value!r}")
    return tuple(float(item) for item in value)


class RobotService:
    """Own RobotIO and make every UART operation pass through one queue.

    The worker is the only thread that calls RobotIO. STOP requests outrank
    motion, which outranks essential and diagnostic telemetry.
    """

    def __init__(
        self,
        port: str = config.DEFAULT_PORT,
        baudrate: int = config.DEFAULT_BAUDRATE,
        *,
        mock: bool = False,
        read_only: bool = False,
        telemetry: bool = True,
        runtime_profile: bool = False,
        fast_hz: float | None = None,
        slow_hz: float | None = None,
        io: RobotIO | None = None,
    ):
        self.io = io or RobotIO(port, baudrate, mock=mock, read_only=read_only)
        self.port = self.io.port
        self.baudrate = self.io.baudrate
        self.read_only = self.io.read_only
        self.telemetry_enabled = telemetry
        self.runtime_profile = runtime_profile
        self._fast_hz = config.TELEMETRY_FAST_HZ if fast_hz is None else fast_hz
        self._slow_hz = config.TELEMETRY_SLOW_HZ if slow_hz is None else slow_hz
        if self._fast_hz <= 0 or self._slow_hz <= 0:
            raise ValueError("telemetry rates must be positive")
        self._requests: queue.PriorityQueue[_Request] = queue.PriorityQueue()
        self._counter = itertools.count()
        self._stop_event = threading.Event()
        self._state_lock = threading.Lock()
        self._state = RobotState(mode=RobotMode.INITIALIZING)
        self._subscribers: list[Callable[[RobotState], None]] = []
        self._motion_armed = False
        self._thread = threading.Thread(target=self._run, name="robot-service", daemon=True)
        self._thread.start()

    @property
    def backend_class_name(self) -> str:
        return self.io.backend_class_name

    def latest_state(self) -> RobotState:
        with self._state_lock:
            return self._state

    def subscribe(self, callback: Callable[[RobotState], None]) -> Callable[[], None]:
        with self._state_lock:
            self._subscribers.append(callback)

        def unsubscribe() -> None:
            with self._state_lock:
                if callback in self._subscribers:
                    self._subscribers.remove(callback)

        return unsubscribe

    def call(
        self,
        method: str,
        *args,
        priority: int | None = None,
        timeout_s: float = 15.0,
        **kwargs,
    ):
        if self._stop_event.is_set():
            raise RuntimeError("robot service is closed")
        if priority is None:
            priority = self._priority_for(method)
        request = _Request(priority, next(self._counter), method, args, kwargs)
        self._requests.put(request)
        if not request.completed.wait(timeout_s):
            raise TimeoutError(f"robot service request timed out: {method}")
        if request.error is not None:
            raise request.error
        return request.result

    def _priority_for(self, method: str) -> int:
        if method == "stop":
            return PRIORITY_STOP
        if method in MOTION_METHODS or method in CONTROL_METHODS:
            return PRIORITY_MOTION
        if method in {"get_angles", "get_error_information", "get_servo_speeds"}:
            return PRIORITY_ESSENTIAL_TELEMETRY
        return PRIORITY_DIAGNOSTIC_TELEMETRY

    def __getattr__(self, name: str):
        if name in READ_METHODS | MOTION_METHODS | CONTROL_METHODS | {"stop"}:
            return lambda *args, **kwargs: self.call(name, *args, **kwargs)
        raise AttributeError(name)

    def stop_motion(self):
        return self.call("stop", priority=PRIORITY_STOP)

    def refresh_state(self, timeout_s: float = 15.0) -> RobotState:
        """Queue one complete read-only telemetry refresh on the UART owner."""
        return self.call(
            "__refresh_all",
            priority=PRIORITY_ESSENTIAL_TELEMETRY,
            timeout_s=timeout_s,
        )

    def refresh_critical_state(self, timeout_s: float = 15.0) -> RobotState:
        """Refresh only joint angles and controller error state."""
        return self.call(
            "__refresh_critical",
            priority=PRIORITY_ESSENTIAL_TELEMETRY,
            timeout_s=timeout_s,
        )

    def arm_motion(
        self,
        *,
        locally_confirmed: bool,
        motion_class: MotionClass = MotionClass.NORMAL,
    ) -> RobotState:
        return self.call(
            "__arm_motion",
            motion_class,
            locally_confirmed,
            priority=PRIORITY_MOTION,
        )

    def disarm_motion(self) -> None:
        self._motion_armed = False

    def disable_torque(self, *, locally_supported: bool = False):
        if not locally_supported:
            raise RuntimeError("disable_torque requires confirmed mechanical support")
        return self.call("release_all_servos", priority=PRIORITY_STOP)

    def close(self) -> None:
        self._stop_event.set()
        self._motion_armed = False
        self._thread.join(timeout=15.0)
        if self._thread.is_alive():
            raise RuntimeError(
                "robot service worker did not stop; UART remains owned to avoid a close/read race"
            )
        while True:
            try:
                request = self._requests.get_nowait()
            except queue.Empty:
                break
            request.error = RuntimeError("robot service closed before request completed")
            request.completed.set()
        self.io.close()
        self._publish(replace(self.latest_state(), mode=RobotMode.DISCONNECTED, connected=False))

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
        return False

    def _run(self) -> None:
        fast_period = 1.0 / self._fast_hz
        slow_period = 1.0 / self._slow_hz
        next_fast = time.monotonic()
        next_slow = time.monotonic()
        self._publish(replace(self.latest_state(), mode=RobotMode.READY, connected=True))
        while not self._stop_event.is_set():
            now = time.monotonic()
            if self.telemetry_enabled and now >= next_fast:
                self._enqueue_internal("__poll_fast", PRIORITY_ESSENTIAL_TELEMETRY)
                next_fast = now + fast_period
            if self.telemetry_enabled and now >= next_slow:
                self._enqueue_internal("__poll_slow", PRIORITY_DIAGNOSTIC_TELEMETRY)
                next_slow = now + slow_period
            try:
                request = self._requests.get(timeout=0.02)
            except queue.Empty:
                continue
            try:
                if request.method == "__poll_fast":
                    request.result = self._poll_fast(request.queued_at_s)
                elif request.method == "__poll_slow":
                    request.result = (
                        self._poll_temperature(request.queued_at_s)
                        if self.runtime_profile
                        else self._poll_slow(request.queued_at_s)
                    )
                elif request.method == "__arm_motion":
                    request.result = self._arm_motion(
                        *request.args, queued_at_s=request.queued_at_s
                    )
                elif request.method == "__refresh_all":
                    self._poll_fast(request.queued_at_s)
                    request.result = self._poll_slow(request.queued_at_s)
                elif request.method == "__refresh_critical":
                    request.result = self._poll_fast(request.queued_at_s)
                else:
                    request.result = self._execute(request)
            except BaseException as exc:
                request.error = exc
                self._publish(replace(
                    self.latest_state(), mode=RobotMode.FAULT, fault=f"{type(exc).__name__}: {exc}"
                ))
            finally:
                request.completed.set()

    def _enqueue_internal(self, method: str, priority: int) -> None:
        # At most one pending poll of each kind; stale telemetry should not
        # create a backlog behind motion commands.
        with self._requests.mutex:
            if any(item.method == method for item in self._requests.queue):
                return
        self._requests.put(_Request(priority, next(self._counter), method))

    def _execute(self, request: _Request):
        state = self.latest_state()
        if request.method in MOTION_METHODS:
            if not self._motion_armed:
                raise RuntimeError("motion is DISARMED; call arm_motion after local preflight")
            self._publish(replace(state, mode=RobotMode.EXECUTING))
        elif request.method == "stop":
            self._publish(replace(state, mode=RobotMode.STOPPING))
        result = self.io.execute_transaction(
            request.method,
            *request.args,
            queued_at_s=request.queued_at_s,
            priority=request.priority,
            **request.kwargs,
        )
        if request.method in MOTION_METHODS | {"stop"}:
            self._publish(replace(self.latest_state(), mode=RobotMode.READY))
        return result

    def _arm_motion(
        self,
        motion_class: MotionClass,
        locally_confirmed: bool,
        *,
        queued_at_s: float | None = None,
    ) -> RobotState:
        self._poll_fast(queued_at_s)
        state = self._poll_slow(queued_at_s)
        decision = motion_permission(
            state, motion_class=motion_class, locally_confirmed=locally_confirmed
        )
        if not decision.allowed:
            self._motion_armed = False
            raise RuntimeError("motion preflight failed: " + "; ".join(decision.reasons))
        self._motion_armed = True
        return state

    def _poll_fast(self, queued_at_s: float | None = None) -> RobotState:
        current = self.latest_state()
        angles = self._read_vector("get_angles", "joint angles", PRIORITY_ESSENTIAL_TELEMETRY, queued_at_s)
        error = self._io_poll("get_error_information", PRIORITY_ESSENTIAL_TELEMETRY, queued_at_s)
        if not isinstance(error, int) or error < 0:
            raise RuntimeError(f"invalid controller error: {error!r}")
        changes = {
            "angles_deg": angles,
            "controller_error": error,
            "critical_monotonic_s": time.monotonic(),
        }
        if not self.runtime_profile:
            changes["speeds"] = self._read_vector(
                "get_servo_speeds", "servo speeds", PRIORITY_ESSENTIAL_TELEMETRY, queued_at_s
            )
        return self._publish_state(current, **changes)

    def _poll_temperature(self, queued_at_s: float | None = None) -> RobotState:
        return self._publish_state(
            self.latest_state(),
            temperatures_c=_vector(
                self._io_poll("get_servo_temps", PRIORITY_DIAGNOSTIC_TELEMETRY, queued_at_s),
                "temperatures",
            ),
        )

    def _poll_slow(self, queued_at_s: float | None = None) -> RobotState:
        current = self._poll_temperature(queued_at_s)
        return self._publish_state(
            current,
            voltages_v=_vector(self._io_poll("get_servo_voltages", PRIORITY_DIAGNOSTIC_TELEMETRY, queued_at_s), "voltages"),
            servo_status=tuple(int(v) for v in _vector(self._io_poll("get_servo_status", PRIORITY_DIAGNOSTIC_TELEMETRY, queued_at_s), "servo status")),
            firmware_pose_mm_deg=_vector(self._io_poll("get_coords", PRIORITY_DIAGNOSTIC_TELEMETRY, queued_at_s), "firmware pose"),
            powered=bool(self._io_poll("is_power_on", PRIORITY_DIAGNOSTIC_TELEMETRY, queued_at_s)),
            servos_enabled=bool(self._io_poll("is_all_servo_enable", PRIORITY_DIAGNOSTIC_TELEMETRY, queued_at_s)),
        )

    def _io_poll(self, method: str, priority: int, queued_at_s: float | None = None):
        queued = time.monotonic() if queued_at_s is None else queued_at_s
        return self.io.execute_transaction(
            method, queued_at_s=queued, priority=priority
        )

    def _read_vector(
        self,
        method: str,
        name: str,
        priority: int,
        queued_at_s: float | None = None,
    ) -> tuple[float, ...]:
        """Retry transient malformed passive reads; every attempt remains logged."""
        last_error: RuntimeError | None = None
        for attempt in range(config.TELEMETRY_READ_RETRIES):
            try:
                return _vector(self._io_poll(method, priority, queued_at_s), name)
            except RuntimeError as exc:
                last_error = exc
                if attempt + 1 < config.TELEMETRY_READ_RETRIES:
                    time.sleep(config.TELEMETRY_RETRY_DELAY_S)
        assert last_error is not None
        raise last_error

    def _publish_state(self, current: RobotState, **changes) -> RobotState:
        candidate = replace(
            current,
            timestamp_utc=datetime.now(timezone.utc).isoformat(),
            monotonic_s=time.monotonic(),
            sequence=current.sequence + 1,
            connected=True,
            fault=None,
            **changes,
        )
        decision = evaluate_state(candidate)
        if not decision.allowed:
            mode = RobotMode.FAULT
        elif candidate.mode is RobotMode.FAULT:
            mode = RobotMode.READY
        else:
            mode = candidate.mode
        if not decision.allowed:
            self._motion_armed = False
        return self._publish(replace(candidate, mode=mode, alerts=decision.alerts))

    def _publish(self, state: RobotState) -> RobotState:
        with self._state_lock:
            self._state = state
            subscribers = list(self._subscribers)
        for callback in subscribers:
            try:
                callback(state)
            except Exception:
                pass
        return state

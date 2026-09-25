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
        io: RobotIO | None = None,
    ):
        self.io = io or RobotIO(port, baudrate, mock=mock, read_only=read_only)
        self.port = self.io.port
        self.baudrate = self.io.baudrate
        self.read_only = self.io.read_only
        self.telemetry_enabled = telemetry
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
        fast_period = 1.0 / config.TELEMETRY_FAST_HZ
        slow_period = 1.0 / config.TELEMETRY_SLOW_HZ
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
                    request.result = self._poll_fast()
                elif request.method == "__poll_slow":
                    request.result = self._poll_slow()
                elif request.method == "__arm_motion":
                    request.result = self._arm_motion(*request.args)
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
        result = self.io.call(request.method, *request.args, **request.kwargs)
        if request.method in MOTION_METHODS | {"stop"}:
            self._publish(replace(self.latest_state(), mode=RobotMode.READY))
        return result

    def _arm_motion(self, motion_class: MotionClass, locally_confirmed: bool) -> RobotState:
        self._poll_fast()
        state = self._poll_slow()
        decision = motion_permission(
            state, motion_class=motion_class, locally_confirmed=locally_confirmed
        )
        if not decision.allowed:
            self._motion_armed = False
            raise RuntimeError("motion preflight failed: " + "; ".join(decision.reasons))
        self._motion_armed = True
        return state

    def _poll_fast(self) -> RobotState:
        current = self.latest_state()
        angles = _vector(self.io.call("get_angles"), "joint angles")
        error = self.io.call("get_error_information")
        speeds = _vector(self.io.call("get_servo_speeds"), "servo speeds")
        if not isinstance(error, int) or error < 0:
            raise RuntimeError(f"invalid controller error: {error!r}")
        return self._publish_state(current, angles_deg=angles, speeds=speeds, controller_error=error)

    def _poll_slow(self) -> RobotState:
        current = self.latest_state()
        return self._publish_state(
            current,
            temperatures_c=_vector(self.io.call("get_servo_temps"), "temperatures"),
            voltages_v=_vector(self.io.call("get_servo_voltages"), "voltages"),
            servo_status=tuple(int(v) for v in _vector(self.io.call("get_servo_status"), "servo status")),
            firmware_pose_mm_deg=_vector(self.io.call("get_coords"), "firmware pose"),
            powered=bool(self.io.call("is_power_on")),
            servos_enabled=bool(self.io.call("is_all_servo_enable")),
        )

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

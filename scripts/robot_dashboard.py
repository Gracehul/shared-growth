#!/usr/bin/env python3
"""Serve the local read-only Shared Growth robot telemetry dashboard."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import threading
import time
from collections import deque
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drawing_robot import config


PROJECT_TEMPERATURE_GATE_C = 60.0
HISTORY_SAMPLES = 180

ERROR_LABELS = {
    0: "No controller error",
    1: "J1 limit exceeded",
    2: "J2 limit exceeded",
    3: "J3 limit exceeded",
    4: "J4 limit exceeded",
    5: "J5 limit exceeded",
    6: "J6 limit exceeded",
    16: "Collision protection",
    17: "Collision protection",
    18: "Collision protection",
    19: "Collision protection",
    32: "No inverse-kinematics solution",
    33: "No adjacent linear solution",
    34: "No adjacent linear solution",
}


def _valid_vector(value: object, length: int = 6) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == length
        and all(isinstance(item, (int, float)) for item in value)
    )


def _read_vector(callable_, name: str, retries: int = 4) -> list[float]:
    last: object = None
    for _ in range(retries):
        last = callable_()
        if _valid_vector(last):
            return [float(item) for item in last]
        time.sleep(0.04)
    raise RuntimeError(f"invalid {name} response: {last!r}")


class TelemetrySampler:
    def __init__(self, port: str, baud: int, interval_s: float, mock: bool = False):
        self.port = port
        self.baud = baud
        self.interval_s = interval_s
        self.mock = mock
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._robot = None
        self._history: deque[dict[str, Any]] = deque(maxlen=HISTORY_SAMPLES)
        self._current = self._offline("Waiting for the first telemetry sample.")

    def _offline(self, detail: str) -> dict[str, Any]:
        return {
            "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "connected": False,
            "read_only": True,
            "motion_enabled": False,
            "severity": "danger",
            "state_label": "Robot offline",
            "state_detail": detail,
            "host": socket.gethostname(),
            "port": self.port,
            "controller_error": None,
            "controller_error_label": "Unavailable",
            "sample_latency_ms": None,
            "firmware_pose_mm_deg": [],
            "joints": [],
            "alerts": [{"severity": "danger", "title": "No robot telemetry", "detail": detail}],
        }

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="robot-telemetry", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._disconnect()

    def payload(self) -> dict[str, Any]:
        with self._lock:
            return {"current": self._current, "history": list(self._history)}

    def _disconnect(self) -> None:
        serial_port = getattr(self._robot, "_serial_port", None)
        if serial_port is not None:
            try:
                serial_port.close()
            except Exception:
                pass
        self._robot = None

    def _connect(self):
        if self._robot is None:
            from pymycobot import MyCobot280

            self._robot = MyCobot280(self.port, self.baud)
        return self._robot

    def _mock_sample(self) -> dict[str, Any]:
        phase = time.monotonic() / 8.0
        angles = [0.6, 111.5, -146.5, -37.4, -6.8, 11.8 + math.sin(phase) * 1.5]
        temperatures = [44, 39, 41, 52 + math.sin(phase / 2) * 1.5, 41, 40]
        return self._build_sample(
            angles=angles,
            temperatures=temperatures,
            voltages=[12.1, 12.1, 12.1, 7.3, 7.5, 7.4],
            statuses=[0, 0, 0, 0, 0, 0],
            speeds=[0, 0, 0, 0, 0, 0],
            firmware_pose=[38.2, -68.3, 157.0, -161.4, -2.77, -102.66],
            error=0,
            powered=True,
            servos_enabled=True,
            latency_ms=18.0,
        )

    def _hardware_sample(self) -> dict[str, Any]:
        robot = self._connect()
        started = time.monotonic()
        angles = _read_vector(robot.get_angles, "angles")
        temperatures = _read_vector(robot.get_servo_temps, "temperatures")
        voltages = _read_vector(robot.get_servo_voltages, "voltages")
        statuses = _read_vector(robot.get_servo_status, "servo status")
        speeds = _read_vector(robot.get_servo_speeds, "servo speeds")
        firmware_pose = _read_vector(robot.get_coords, "firmware pose")
        error = robot.get_error_information()
        if not isinstance(error, int) or error < 0:
            raise RuntimeError(f"invalid controller error response: {error!r}")
        return self._build_sample(
            angles=angles,
            temperatures=temperatures,
            voltages=voltages,
            statuses=statuses,
            speeds=speeds,
            firmware_pose=firmware_pose,
            error=error,
            powered=bool(robot.is_power_on()),
            servos_enabled=bool(robot.is_all_servo_enable()),
            latency_ms=(time.monotonic() - started) * 1000.0,
        )

    def _build_sample(
        self,
        *,
        angles: list[float],
        temperatures: list[float],
        voltages: list[float],
        statuses: list[float],
        speeds: list[float],
        firmware_pose: list[float],
        error: int,
        powered: bool,
        servos_enabled: bool,
        latency_ms: float,
    ) -> dict[str, Any]:
        alerts: list[dict[str, str]] = []
        joints = []
        for index, angle in enumerate(angles):
            low, high = config.JOINT_LIMITS_DEG[index]
            margin = min(angle - low, high - angle)
            joint = {
                "id": index + 1,
                "angle_deg": angle,
                "minimum_deg": low,
                "maximum_deg": high,
                "limit_margin_deg": margin,
                "temperature_c": temperatures[index],
                "voltage_v": voltages[index],
                "status": int(statuses[index]),
                "speed_steps_s": speeds[index],
            }
            joints.append(joint)
            if temperatures[index] >= PROJECT_TEMPERATURE_GATE_C:
                alerts.append({
                    "severity": "danger",
                    "title": f"J{index + 1} temperature gate",
                    "detail": f"{temperatures[index]:.0f} °C is at or above the project pause gate of {PROJECT_TEMPERATURE_GATE_C:.0f} °C.",
                })
            elif temperatures[index] >= PROJECT_TEMPERATURE_GATE_C - 5:
                alerts.append({
                    "severity": "warning",
                    "title": f"J{index + 1} temperature rising",
                    "detail": f"{temperatures[index]:.0f} °C is within 5 °C of the project pause gate.",
                })
            if margin < config.JOINT_LIMIT_MARGIN_DEG:
                alerts.append({
                    "severity": "danger",
                    "title": f"J{index + 1} limit margin",
                    "detail": f"Only {margin:.2f}° remains to the nearest configured limit.",
                })
            if int(statuses[index]) != 0:
                alerts.append({
                    "severity": "danger",
                    "title": f"J{index + 1} servo status",
                    "detail": "The servo reports a non-zero status flag.",
                })

        if error != 0:
            alerts.append({
                "severity": "danger",
                "title": "Controller error",
                "detail": ERROR_LABELS.get(error, f"Undocumented controller code {error}."),
            })
        if not powered or not servos_enabled:
            alerts.append({
                "severity": "warning",
                "title": "Drive state",
                "detail": f"Power: {'on' if powered else 'off'} · servos: {'enabled' if servos_enabled else 'disabled'}.",
            })

        severity = "danger" if any(alert["severity"] == "danger" for alert in alerts) else "warning" if alerts else "ok"
        if severity == "ok":
            label = "Telemetry nominal"
            detail = "All read-only development gates are currently clear."
        elif severity == "warning":
            label = "Review advised"
            detail = "Telemetry is available, with one or more values approaching a development gate."
        else:
            label = "Motion paused"
            detail = "At least one development gate is active. This dashboard cannot clear it or move the robot."
        timestamp = datetime.now(timezone.utc).isoformat()
        return {
            "timestamp_utc": timestamp,
            "connected": True,
            "read_only": True,
            "motion_enabled": False,
            "severity": severity,
            "state_label": label,
            "state_detail": detail,
            "host": socket.gethostname(),
            "port": self.port,
            "powered": powered,
            "servos_enabled": servos_enabled,
            "controller_error": error,
            "controller_error_label": ERROR_LABELS.get(error, f"Undocumented code {error}"),
            "sample_latency_ms": latency_ms,
            "firmware_pose_mm_deg": firmware_pose,
            "joints": joints,
            "alerts": alerts,
        }

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                sample = self._mock_sample() if self.mock else self._hardware_sample()
                history = {
                    "timestamp_utc": sample["timestamp_utc"],
                    "temperatures_c": [joint["temperature_c"] for joint in sample["joints"]],
                    "angles_deg": [joint["angle_deg"] for joint in sample["joints"]],
                }
                with self._lock:
                    self._current = sample
                    self._history.append(history)
            except Exception as exc:
                self._disconnect()
                with self._lock:
                    self._current = self._offline(f"{type(exc).__name__}: {exc}")
            self._stop.wait(self.interval_s)


class DashboardHandler(SimpleHTTPRequestHandler):
    sampler: TelemetrySampler

    def __init__(self, *args, directory: str, **kwargs):
        super().__init__(*args, directory=directory, **kwargs)

    def end_headers(self) -> None:
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:; script-src 'self'; connect-src 'self'")
        super().end_headers()

    def do_GET(self) -> None:
        if self.path.split("?", 1)[0] == "/api/status":
            body = json.dumps(self.sampler.payload()).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.split("?", 1)[0] == "/healthz":
            body = b"ok\n"
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def log_message(self, format: str, *args) -> None:
        print(f"dashboard: {format % args}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--web-port", type=int, default=8765)
    parser.add_argument("--robot-port", default=config.DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=config.DEFAULT_BAUDRATE)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--mock", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.interval < 0.25:
        raise SystemExit("--interval must be at least 0.25 seconds")
    static_directory = Path(__file__).resolve().parent.parent / "dashboard"
    sampler = TelemetrySampler(args.robot_port, args.baud, args.interval, mock=args.mock)
    sampler.start()

    def handler(*handler_args, **handler_kwargs):
        return DashboardHandler(*handler_args, directory=str(static_directory), **handler_kwargs)

    DashboardHandler.sampler = sampler
    server = ThreadingHTTPServer((args.host, args.web_port), handler)
    mode = "mock telemetry" if args.mock else f"read-only robot telemetry on {args.robot_port}"
    print(f"Shared Growth Control Room: http://{args.host}:{args.web_port}")
    print(f"Mode: {mode}. No motion endpoint exists.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        sampler.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

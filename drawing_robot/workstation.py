"""HTTP-facing adapter for the Robot Interaction Workstation.

The module deliberately exposes only the established read-side snapshot and
high-level operation contracts.  It has no robot, UART or trajectory-planning
knowledge.
"""

from __future__ import annotations

import json
import mimetypes
import threading
import uuid
from dataclasses import asdict, dataclass
from enum import Enum
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Mapping, Sequence

from .hardware import SystemSnapshotAdapter
from .operations import (
    InteractionMode,
    Operation,
    OperationController,
    OperationReceipt,
    OperationRequest,
)


API_SCHEMA_VERSION = "shared-growth/workstation-api/v1"


@dataclass(frozen=True)
class WorkstationOperation:
    """One prepared high-level operation advertised to the browser."""

    operation_id: str
    label: str
    operation: Operation
    trajectory_id: str | None = None
    description: str = ""

    def to_dict(self) -> dict[str, object]:
        return {
            "operation_id": self.operation_id,
            "label": self.label,
            "operation": self.operation.value,
            "trajectory_id": self.trajectory_id,
            "description": self.description,
        }


def _json_value(value):
    if isinstance(value, Enum):
        return value.value
    if hasattr(value, "to_dict"):
        return _json_value(value.to_dict())
    if hasattr(value, "__dataclass_fields__"):
        return _json_value(asdict(value))
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    return value


def receipt_to_dict(receipt: OperationReceipt | None) -> dict[str, object] | None:
    """Serialize the authoritative controller receipt without reinterpretation."""

    return None if receipt is None else _json_value(receipt)


class WorkstationApplication:
    """Small adapter over SystemSnapshotAdapter and OperationController."""

    _REQUEST_FIELDS = {
        "request_id",
        "operation_id",
        "session_id",
        "perception_id",
        "intent_id",
    }

    def __init__(
        self,
        snapshot_adapter: SystemSnapshotAdapter,
        operation_controller: OperationController,
        operations: Sequence[WorkstationOperation],
        *,
        backend_label: str = "core",
    ) -> None:
        self._snapshot = snapshot_adapter
        self._controller = operation_controller
        self._operations = {item.operation_id: item for item in operations}
        self._backend_label = backend_label
        self._state_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._active_request: dict[str, object] | None = None

    @property
    def mode(self) -> InteractionMode:
        return self._controller.mode

    def set_mode(self, value: str) -> dict[str, object]:
        mode = InteractionMode(value)
        with self._state_lock:
            if self._active_request is not None:
                raise RuntimeError("MODE_CHANGE_BLOCKED: operation is active")
            self._controller.set_mode(mode)
        return {"mode": mode.value}

    def status(self) -> dict[str, object]:
        snapshot = self._snapshot.get_system_snapshot()
        # Keep one read authoritative for this response instead of sampling
        # state and gates independently at slightly different instants.
        gates = snapshot.gate_results
        event = snapshot.latest_execution_event
        receipt = self._controller.latest_receipt()
        if event is None and receipt is not None:
            event = receipt.latest_execution_event
        with self._state_lock:
            active = None if self._active_request is None else dict(self._active_request)
        return {
            "schema_version": API_SCHEMA_VERSION,
            "backend": self._backend_label,
            "mode": self.mode.value,
            "snapshot": _json_value(snapshot.to_dict()),
            "gate_results": [_json_value(item) for item in gates],
            "thresholds": self._snapshot.get_thresholds(),
            "latest_execution_event": _json_value(event),
            "latest_receipt": receipt_to_dict(receipt),
            "active_request": active,
            "operations": [item.to_dict() for item in self._operations.values()],
        }

    def request_operation(self, payload: Mapping[str, object]) -> dict[str, object]:
        unknown = set(payload) - self._REQUEST_FIELDS
        if unknown:
            raise ValueError(
                "unsupported operation fields: " + ", ".join(sorted(unknown))
            )
        operation_id = str(payload.get("operation_id", ""))
        prepared = self._operations.get(operation_id)
        if prepared is None:
            raise ValueError(f"unknown prepared operation: {operation_id or '<empty>'}")
        request_id = str(payload.get("request_id") or uuid.uuid4())
        request = OperationRequest(
            request_id=request_id,
            mode=self.mode,
            operation=prepared.operation,
            trajectory_id=prepared.trajectory_id,
            session_id=_optional_string(payload.get("session_id")),
            perception_id=_optional_string(payload.get("perception_id")),
            intent_id=_optional_string(payload.get("intent_id")),
        )
        if not self._request_lock.acquire(blocking=False):
            raise RuntimeError("OPERATION_IN_PROGRESS")
        try:
            with self._state_lock:
                self._active_request = {
                    "request_id": request.request_id,
                    "operation_id": operation_id,
                    "operation": prepared.operation.value,
                }
            return receipt_to_dict(self._controller.request(request))
        finally:
            with self._state_lock:
                self._active_request = None
            self._request_lock.release()

    def stop(self, payload: Mapping[str, object] | None = None) -> dict[str, object]:
        payload = payload or {}
        unknown = set(payload) - {"request_id", "session_id"}
        if unknown:
            raise ValueError("unsupported stop fields: " + ", ".join(sorted(unknown)))
        receipt = self._controller.stop(
            request_id=str(payload.get("request_id") or f"stop-{uuid.uuid4()}"),
            mode=self.mode,
            session_id=_optional_string(payload.get("session_id")),
        )
        return receipt_to_dict(receipt)


def _optional_string(value: object) -> str | None:
    if value is None:
        return None
    text = str(value)
    return text or None


def create_workstation_server(
    application: WorkstationApplication,
    static_directory: str | Path,
    *,
    host: str = "127.0.0.1",
    port: int = 8766,
) -> ThreadingHTTPServer:
    """Create a local server; serving it does not open any hardware session."""

    static_root = Path(static_directory).resolve()

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(static_root), **kwargs)

        def log_message(self, format, *args):  # noqa: A002 - stdlib signature
            return

        def do_GET(self) -> None:  # noqa: N802 - stdlib interface
            route = self.path.split("?", 1)[0]
            if route == "/api/workstation/status":
                self._send_json(HTTPStatus.OK, application.status())
                return
            if route == "/":
                self.path = "/index.html"
            super().do_GET()

        def do_POST(self) -> None:  # noqa: N802 - stdlib interface
            route = self.path.split("?", 1)[0]
            try:
                payload = self._read_json()
                if route == "/api/workstation/mode":
                    result = application.set_mode(str(payload.get("mode", "")))
                elif route == "/api/workstation/operations":
                    result = application.request_operation(payload)
                elif route == "/api/workstation/stop":
                    result = application.stop(payload)
                else:
                    self._send_json(HTTPStatus.NOT_FOUND, {"error": "NOT_FOUND"})
                    return
            except (KeyError, TypeError, ValueError) as exc:
                self._send_json(
                    HTTPStatus.BAD_REQUEST,
                    {"error": type(exc).__name__, "detail": str(exc)},
                )
                return
            except RuntimeError as exc:
                self._send_json(
                    HTTPStatus.CONFLICT,
                    {"error": "CONFLICT", "detail": str(exc)},
                )
                return
            self._send_json(HTTPStatus.OK, result)

        def _read_json(self) -> dict[str, object]:
            length = int(self.headers.get("Content-Length", "0"))
            if length > 64 * 1024:
                raise ValueError("request body is too large")
            if length == 0:
                return {}
            value = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(value, dict):
                raise TypeError("JSON body must be an object")
            return value

        def _send_json(self, status: HTTPStatus, value: object) -> None:
            body = json.dumps(_json_value(value), allow_nan=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def guess_type(self, path):
            return mimetypes.guess_type(path)[0] or "application/octet-stream"

    return ThreadingHTTPServer((host, int(port)), Handler)

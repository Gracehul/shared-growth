"""Shared, versioned event envelope for hardware and experiment runs."""

from __future__ import annotations

import json
import platform
import time
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class TimelineEvent:
    kind: str
    elapsed_s: float
    timestamp_utc: str
    data: dict[str, Any] = field(default_factory=dict)


class RunLog:
    EVENT_KINDS = {
        "COMMAND_SENT", "STATE_SAMPLE", "TEMP_SAMPLE", "FAULT",
        "STOP_REQUESTED", "MOTION_DETECTED", "MOTION_SETTLED",
    }

    def __init__(
        self,
        schema: str,
        *,
        hardware: dict[str, Any] | None = None,
        software: dict[str, Any] | None = None,
        config: dict[str, Any] | None = None,
        run_id: str | None = None,
    ):
        if not schema.startswith("shared-growth/"):
            raise ValueError("schema must start with shared-growth/")
        self.schema = schema
        self.run_id = run_id or str(uuid.uuid4())
        self.hardware = dict(hardware or {})
        self.software = {"python": platform.python_version(), **dict(software or {})}
        self.config = dict(config or {})
        self.timeline: list[TimelineEvent] = []
        self.result: dict[str, Any] = {"status": "running"}
        self.started_utc = datetime.now(timezone.utc).isoformat()
        self._origin = time.monotonic()

    def add(self, kind: str, **data: Any) -> TimelineEvent:
        if kind not in self.EVENT_KINDS:
            raise ValueError(f"unknown event kind: {kind}")
        event = TimelineEvent(
            kind=kind,
            elapsed_s=round(time.monotonic() - self._origin, 6),
            timestamp_utc=datetime.now(timezone.utc).isoformat(),
            data=data,
        )
        self.timeline.append(event)
        return event

    def finish(self, status: str, **details: Any) -> None:
        if status not in {"completed", "aborted", "failed"}:
            raise ValueError("invalid result status")
        self.result = {"status": status, **details}

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": self.schema,
            "run_id": self.run_id,
            "started_utc": self.started_utc,
            "hardware": self.hardware,
            "software": self.software,
            "config": self.config,
            "timeline": [asdict(event) for event in self.timeline],
            "result": self.result,
        }

    def write(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8")

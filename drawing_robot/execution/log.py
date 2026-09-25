"""Versioned Stage-3 execution log and recorded-state replay."""

from __future__ import annotations

import json
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable

from .commands import JointCommand
from .state import RobotState


SCHEMA_VERSION = "shared-growth/execution/v1"


@dataclass(frozen=True)
class ExecutionEvent:
    sequence: int
    timestamp_s: float
    kind: str
    data: dict[str, Any]


@dataclass(frozen=True)
class ReplayFrame:
    timestamp_s: float
    state: dict[str, Any]
    events: tuple[dict[str, Any], ...]


class ExecutionLog:
    EVENT_KINDS = {
        "COMMAND_SENT",
        "COMMAND_ACTIVATED",
        "COMMAND_DROPPED",
        "COMMAND_REJECTED",
        "STATE_PUBLISHED",
        "STATE_STALE",
        "STOP_REQUESTED",
        "STOP_APPLIED",
        "FAULT",
        "TIMEOUT",
        "RUN_COMPLETED",
        "RUN_FAILED",
    }

    def __init__(
        self,
        trajectory_id: str,
        simulation_config: dict[str, Any],
        random_seed: int,
        initial_state: RobotState,
        run_id: str | None = None,
    ) -> None:
        self.schema_version = SCHEMA_VERSION
        self.run_id = run_id or str(uuid.uuid4())
        self.trajectory_id = trajectory_id
        self.simulation_config = simulation_config
        self.random_seed = int(random_seed)
        self.initial_state = self._state_dict(initial_state)
        self.commands: list[dict[str, Any]] = []
        self.states: list[dict[str, Any]] = []
        self.events: list[ExecutionEvent] = []
        self.result: dict[str, Any] = {"status": "running"}
        self._commands_by_id: dict[str, dict[str, Any]] = {}
        self._sequence = 0

    @staticmethod
    def _state_dict(state: RobotState) -> dict[str, Any]:
        value = asdict(state)
        value["status"] = state.status.value
        return value

    @staticmethod
    def _json_value(value: Any) -> Any:
        if isinstance(value, dict):
            return {str(key): ExecutionLog._json_value(item) for key, item in value.items()}
        if isinstance(value, (tuple, list)):
            return [ExecutionLog._json_value(item) for item in value]
        return value

    def add_event(self, kind: str, timestamp_s: float, **data: Any) -> ExecutionEvent:
        if kind not in self.EVENT_KINDS:
            raise ValueError(f"unknown execution event: {kind}")
        event = ExecutionEvent(self._sequence, float(timestamp_s), kind, dict(data))
        self._sequence += 1
        self.events.append(event)
        return event

    def command_sent(self, command: JointCommand, send_time_s: float) -> None:
        record = {
            "command_id": command.command_id,
            "planned_time_s": command.scheduled_time_s,
            "target_angles_deg": list(command.target_angles_deg),
            "send_time_s": float(send_time_s),
            "activation_time_s": None,
            "effective_delay_s": None,
            "dropped": False,
        }
        self.commands.append(record)
        self._commands_by_id[command.command_id] = record
        self.add_event(
            "COMMAND_SENT",
            send_time_s,
            command_id=command.command_id,
            planned_time_s=command.scheduled_time_s,
            target_angles_deg=list(command.target_angles_deg),
        )

    def command_activated(self, command_id: str, activation_time_s: float) -> None:
        record = self._commands_by_id[command_id]
        record["activation_time_s"] = float(activation_time_s)
        record["effective_delay_s"] = float(
            activation_time_s - record["send_time_s"]
        )
        self.add_event(
            "COMMAND_ACTIVATED",
            activation_time_s,
            command_id=command_id,
            effective_delay_s=record["effective_delay_s"],
        )

    def command_dropped(self, command_id: str, timestamp_s: float) -> None:
        self._commands_by_id[command_id]["dropped"] = True
        self.add_event("COMMAND_DROPPED", timestamp_s, command_id=command_id)

    def record_state(self, state: RobotState) -> None:
        value = self._state_dict(state)
        index = len(self.states)
        self.states.append(value)
        self.add_event("STATE_PUBLISHED", state.timestamp_s, state_index=index)

    def finish(self, status: str, timestamp_s: float, **details: Any) -> None:
        self.result = {"status": status, "timestamp_s": float(timestamp_s), **details}

    def to_dict(self) -> dict[str, Any]:
        return self._json_value({
            "schema_version": self.schema_version,
            "run_id": self.run_id,
            "trajectory_id": self.trajectory_id,
            "simulation_config": self.simulation_config,
            "random_seed": self.random_seed,
            "initial_state": self.initial_state,
            "commands": self.commands,
            "states": self.states,
            "events": [asdict(event) for event in self.events],
            "result": self.result,
        })

    def write(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")
        return destination

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "ExecutionLog":
        if value.get("schema_version") != SCHEMA_VERSION:
            raise ValueError("unsupported execution log schema")
        initial = value["initial_state"]
        from .state import RobotStatus

        log = cls(
            trajectory_id=value["trajectory_id"],
            simulation_config=dict(value["simulation_config"]),
            random_seed=value["random_seed"],
            initial_state=RobotState(
                initial["timestamp_s"],
                initial["commanded_angles_deg"],
                initial["actual_angles_deg"],
                RobotStatus(initial["status"]),
                initial.get("last_command_id"),
                initial.get("fault"),
            ),
            run_id=value["run_id"],
        )
        log.commands = list(value["commands"])
        log._commands_by_id = {item["command_id"]: item for item in log.commands}
        log.states = list(value["states"])
        log.events = tuple_to_events(value["events"])
        log._sequence = max((event.sequence for event in log.events), default=-1) + 1
        log.result = dict(value["result"])
        return log

    @classmethod
    def read(cls, path: str | Path) -> "ExecutionLog":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def tuple_to_events(values: Iterable[dict[str, Any]]) -> list[ExecutionEvent]:
    return [
        ExecutionEvent(
            int(value["sequence"]),
            float(value["timestamp_s"]),
            str(value["kind"]),
            dict(value.get("data", {})),
        )
        for value in values
    ]


def replay_log(log: ExecutionLog) -> tuple[ReplayFrame, ...]:
    """Replay recorded evidence; no simulator or trajectory code is run."""
    events_by_time: dict[float, list[dict[str, Any]]] = {}
    for event in log.events:
        events_by_time.setdefault(event.timestamp_s, []).append(asdict(event))
    states_by_time = {float(state["timestamp_s"]): dict(state) for state in log.states}
    timestamps = sorted(set(events_by_time) | set(states_by_time))
    frames: list[ReplayFrame] = []
    current_state = dict(log.initial_state)
    for timestamp in timestamps:
        if timestamp in states_by_time:
            current_state = states_by_time[timestamp]
        frames.append(
            ReplayFrame(
                timestamp,
                dict(current_state),
                tuple(events_by_time.get(timestamp, ())),
            )
        )
    return tuple(frames)

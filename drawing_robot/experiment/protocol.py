"""Explicit, hardware-independent state machine for one Shared Growth turn."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class TrialState(str, Enum):
    IDLE = "IDLE"
    READY = "READY"
    HUMAN_DRAWING = "HUMAN_DRAWING"
    PROCESSING = "PROCESSING"
    WAITING = "WAITING"
    ROBOT_DRAWING = "ROBOT_DRAWING"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"
    PARKED = "PARKED"


class ProtocolEvent(str, Enum):
    INITIALIZE = "INITIALIZE"
    PROMPT_HUMAN = "PROMPT_HUMAN"
    STROKE_COMPLETE = "STROKE_COMPLETE"
    RESPONSE_READY = "RESPONSE_READY"
    DELAY_ELAPSED = "DELAY_ELAPSED"
    ROBOT_COMPLETE = "ROBOT_COMPLETE"
    NEXT_TURN = "NEXT_TURN"
    FAULT = "FAULT"
    PARK = "PARK"


TRANSITIONS = {
    (TrialState.IDLE, ProtocolEvent.INITIALIZE): TrialState.READY,
    (TrialState.READY, ProtocolEvent.PROMPT_HUMAN): TrialState.HUMAN_DRAWING,
    (TrialState.HUMAN_DRAWING, ProtocolEvent.STROKE_COMPLETE): TrialState.PROCESSING,
    (TrialState.PROCESSING, ProtocolEvent.RESPONSE_READY): TrialState.WAITING,
    (TrialState.WAITING, ProtocolEvent.DELAY_ELAPSED): TrialState.ROBOT_DRAWING,
    (TrialState.ROBOT_DRAWING, ProtocolEvent.ROBOT_COMPLETE): TrialState.COMPLETE,
    (TrialState.COMPLETE, ProtocolEvent.NEXT_TURN): TrialState.READY,
    (TrialState.FAULT, ProtocolEvent.PARK): TrialState.PARKED,
}


@dataclass
class TrialMachine:
    state: TrialState = TrialState.IDLE

    def apply(self, event: ProtocolEvent) -> TrialState:
        event = ProtocolEvent(event)
        if event is ProtocolEvent.FAULT and self.state is not TrialState.PARKED:
            self.state = TrialState.FAULT
            return self.state
        try:
            self.state = TRANSITIONS[(self.state, event)]
        except KeyError as exc:
            raise ValueError(f"invalid protocol transition: {self.state.value} + {event.value}") from exc
        return self.state

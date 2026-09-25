"""Backend contract consumed by the Stage-3 executor."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from .commands import CommandAcknowledgement, JointCommand
from .state import RobotState


@runtime_checkable
class RobotInterface(Protocol):
    def send_joint_command(self, command: JointCommand) -> CommandAcknowledgement:
        ...

    def get_state(self) -> RobotState:
        ...

    def stop_motion(self) -> None:
        ...

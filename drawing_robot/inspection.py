"""Read-only robot inspection and forward-kinematics comparison."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone

import numpy as np

from .connection import ArmConnection
from .kinematics import flange_pose_coords, pose_coords, wrap180


@dataclass(frozen=True)
class RobotSnapshot:
    timestamp_utc: str
    expected_model: str
    backend_class: str
    port: str
    baudrate: int
    powered: bool
    joint_angles_deg: list[float]
    firmware_flange_pose_mm_deg: list[float]
    python_flange_pose_mm_deg: list[float]
    python_tcp_pose_mm_deg: list[float]

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class FKComparison:
    position_error_xyz_mm: list[float]
    position_error_norm_mm: float
    orientation_error_rpy_deg: list[float]
    orientation_error_max_deg: float
    position_tolerance_mm: float
    orientation_tolerance_deg: float
    passed: bool

    def to_dict(self) -> dict:
        return asdict(self)


def inspect_robot(
    connection: ArmConnection,
    *,
    expected_model: str = "myCobot 280 JN",
) -> RobotSnapshot:
    """Read current state without issuing configuration or motion commands."""

    if not connection.read_only:
        raise ValueError("inspection requires ArmConnection(read_only=True)")

    angles = connection.get_angles()
    firmware_flange = connection.get_coords_firmware()
    python_flange = flange_pose_coords(angles)
    python_tcp = pose_coords(angles)
    return RobotSnapshot(
        timestamp_utc=datetime.now(timezone.utc).isoformat(),
        expected_model=expected_model,
        backend_class=connection.backend_class_name,
        port=connection.port,
        baudrate=connection.baudrate,
        powered=connection.is_power_on(),
        joint_angles_deg=[float(value) for value in angles],
        firmware_flange_pose_mm_deg=[float(value) for value in firmware_flange],
        python_flange_pose_mm_deg=[float(value) for value in python_flange],
        python_tcp_pose_mm_deg=[float(value) for value in python_tcp],
    )


def compare_fk(
    snapshot: RobotSnapshot,
    *,
    position_tolerance_mm: float = 2.0,
    orientation_tolerance_deg: float = 2.0,
) -> FKComparison:
    """Compare firmware and Python FK at the bare flange, never the TCP."""

    if position_tolerance_mm <= 0 or orientation_tolerance_deg <= 0:
        raise ValueError("FK tolerances must be positive")

    firmware = np.asarray(snapshot.firmware_flange_pose_mm_deg, dtype=float)
    python = np.asarray(snapshot.python_flange_pose_mm_deg, dtype=float)
    position_error = python[:3] - firmware[:3]
    orientation_error = wrap180(python[3:] - firmware[3:])
    position_norm = float(np.linalg.norm(position_error))
    orientation_max = float(np.max(np.abs(orientation_error)))
    return FKComparison(
        position_error_xyz_mm=[float(value) for value in position_error],
        position_error_norm_mm=position_norm,
        orientation_error_rpy_deg=[float(value) for value in orientation_error],
        orientation_error_max_deg=orientation_max,
        position_tolerance_mm=float(position_tolerance_mm),
        orientation_tolerance_deg=float(orientation_tolerance_deg),
        passed=(
            position_norm <= position_tolerance_mm
            and orientation_max <= orientation_tolerance_deg
        ),
    )

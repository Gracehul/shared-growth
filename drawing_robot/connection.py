"""Validated high-level connection built on the repository's sole RobotIO."""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

import numpy as np

from . import config
from .kinematics import check_joint_limits
from .robot.service import RobotService

log = logging.getLogger(__name__)


class ArmError(RuntimeError):
    pass


@dataclass
class SpeedConversion:
    requested_dps: float
    firmware_speed: int
    actual_dps: float

    @property
    def error_dps(self) -> float:
        return self.actual_dps - self.requested_dps


def dps_to_firmware_speed(deg_per_s: float) -> SpeedConversion:
    if deg_per_s <= 0:
        raise ValueError("angular velocity must be positive")
    raw = 100.0 * deg_per_s / config.DEG_PER_S_AT_SPEED_100
    speed = int(round(np.clip(raw, config.FIRMWARE_SPEED_MIN, config.FIRMWARE_SPEED_MAX)))
    actual = speed / 100.0 * config.DEG_PER_S_AT_SPEED_100
    return SpeedConversion(deg_per_s, speed, actual)


class ArmConnection:
    """Validated synchronous API; RobotIO remains the only backend owner."""

    def __init__(
        self,
        port: str = config.DEFAULT_PORT,
        baudrate: int = config.DEFAULT_BAUDRATE,
        mock: bool = False,
        read_only: bool = False,
    ):
        self.mock = mock
        self.read_only = read_only
        self.port = port
        self.baudrate = baudrate
        try:
            self._service = RobotService(
                port, baudrate, mock=mock, read_only=read_only, telemetry=False
            )
        except RuntimeError as exc:
            raise ArmError(str(exc)) from exc
        if mock:
            log.warning("ArmConnection is in MOCK mode -- no hardware will move.")
        else:
            time.sleep(0.1)
        if not read_only:
            try:
                self.set_fresh_mode(1)
            except Exception as exc:
                log.warning("could not set fresh_mode: %s", exc)
        if mock and not read_only:
            self.arm_motion(locally_confirmed=True)

    @property
    def backend_class_name(self) -> str:
        return self._service.backend_class_name

    @property
    def mock_backend(self):
        """Expose only the fake backend for offline assertions."""
        if not self.mock:
            raise ArmError("the real robot backend is intentionally not exposed")
        return self._service.io._backend

    def set_mock_angles(self, angles_deg) -> None:
        if not self.mock:
            raise ArmError("set_mock_angles is available only in mock mode")
        self._service.call("set_angles_directly", angles_deg)

    def arm_motion(self, *, locally_confirmed: bool) -> None:
        self._require_writable("arm_motion")
        self._service.arm_motion(locally_confirmed=locally_confirmed)

    def disarm_motion(self) -> None:
        self._service.disarm_motion()

    def _require_writable(self, action: str) -> None:
        if self.read_only:
            raise ArmError(f"read-only connection refuses {action}")

    def is_power_on(self) -> bool:
        try:
            return bool(self._service.call("is_power_on"))
        except Exception as exc:
            log.debug("is_power_on failed: %s", exc)
            return False

    def power_on(self):
        self._require_writable("power_on")
        return self._service.call("power_on")

    def set_fresh_mode(self, mode: int) -> int:
        self._require_writable("set_fresh_mode")
        return self._service.call("set_fresh_mode", mode)

    def get_fresh_mode(self) -> int:
        return self._service.call("get_fresh_mode")

    def get_angles(self, retries: int = config.TELEMETRY_READ_RETRIES) -> list[float]:
        for attempt in range(retries):
            try:
                angles = self._service.call("get_angles")
            except Exception as exc:
                log.debug("get_angles raised (attempt %d): %s", attempt, exc)
                angles = None
            if (
                isinstance(angles, (list, tuple))
                and len(angles) == config.DOF
                and all(isinstance(value, (int, float)) for value in angles)
            ):
                return [float(value) for value in angles]
            time.sleep(config.TELEMETRY_RETRY_DELAY_S)
        raise ArmError(f"could not read joint angles after {retries} attempts")

    def get_coords_firmware(self) -> list[float]:
        for _ in range(config.TELEMETRY_READ_RETRIES):
            try:
                coords = self._service.call("get_coords")
            except Exception:
                coords = None
            if isinstance(coords, (list, tuple)) and len(coords) == config.DOF:
                return [float(value) for value in coords]
            time.sleep(config.TELEMETRY_RETRY_DELAY_S)
        raise ArmError("could not read coords from firmware")

    def send_angles(self, angles_deg, deg_per_s: float) -> SpeedConversion:
        self._require_writable("send_angles")
        angles = np.asarray(angles_deg, dtype=float)
        if angles.shape != (config.DOF,):
            raise ValueError(f"expected {config.DOF} angles, got {angles.shape}")
        problems = check_joint_limits(angles)
        if problems:
            raise ArmError("refusing out-of-limit angles: " + "; ".join(problems))
        conversion = dps_to_firmware_speed(deg_per_s)
        payload = [round(float(value), 2) for value in angles]
        self._service.call("send_angles", payload, conversion.firmware_speed)
        return conversion

    def send_angle(self, joint_id: int, angle_deg: float, deg_per_s: float) -> SpeedConversion:
        self._require_writable("send_angle")
        if not 1 <= joint_id <= config.DOF:
            raise ValueError(f"joint_id must be 1..{config.DOF}, got {joint_id}")
        low, high = config.joint_limits_array()[joint_id - 1]
        if not low <= angle_deg <= high:
            raise ArmError(f"J{joint_id}={angle_deg:.2f} outside soft limits [{low:.1f}, {high:.1f}]")
        conversion = dps_to_firmware_speed(deg_per_s)
        self._service.call("send_angle", joint_id, round(float(angle_deg), 2), conversion.firmware_speed)
        return conversion

    def stop(self):
        self._require_writable("stop")
        try:
            return self._service.stop_motion()
        except Exception as exc:
            log.error("stop() failed: %s", exc)
            return 0

    def release_all_servos(self):
        self._require_writable("release_all_servos")
        raise ArmError("use disable_torque(locally_supported=True) explicitly")

    def disable_torque(self, *, locally_supported: bool = False):
        self._require_writable("disable_torque")
        return self._service.disable_torque(locally_supported=locally_supported)

    def focus_all_servos(self):
        self._require_writable("focus_all_servos")
        return self._service.call("focus_all_servos")

    def set_gripper_value(
        self,
        value: int,
        speed: int = config.GRIPPER_DEFAULT_SPEED,
        gripper_type: int | None = None,
    ):
        self._require_writable("set_gripper_value")
        value = int(value)
        speed = int(speed)
        if not 0 <= value <= 100:
            raise ValueError(f"gripper value must be 0-100, got {value}")
        if not config.FIRMWARE_SPEED_MIN <= speed <= config.FIRMWARE_SPEED_MAX:
            raise ValueError(f"gripper speed must be 1-100, got {speed}")
        if gripper_type is None:
            return self._service.call("set_gripper_value", value, speed)
        return self._service.call("set_gripper_value", value, speed, gripper_type)

    def is_gripper_moving(self) -> bool:
        try:
            return bool(self._service.call("is_gripper_moving"))
        except Exception as exc:
            log.debug("is_gripper_moving failed: %s", exc)
            return False

    def get_gripper_value(self, gripper_type: int | None = None) -> int | None:
        try:
            args = () if gripper_type is None else (gripper_type,)
            value = self._service.call("get_gripper_value", *args)
        except Exception as exc:
            log.debug("get_gripper_value failed: %s", exc)
            return None
        return int(value) if value is not None else None

    def close(self):
        try:
            if not self.read_only:
                self.stop()
        finally:
            self._service.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
        return False

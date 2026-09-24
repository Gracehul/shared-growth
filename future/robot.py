"""
Robot -- thin, from-scratch wrapper directly over pymycobot's MyCobot280.

Implements exactly the "Useful software interfaces" from Objectives.md
(home, get_pose, grab_pen, release_pen, pen_up, pen_down, move_to), plus
stop()/recover() for the "safe stop + recovery" task. Nothing here depends
on the thesis `armik` library.

Cartesian motion (move_to/pen_up/pen_down) never asks the robot's own
firmware to solve inverse kinematics (pymycobot's send_coords()) -- that
was found unreliable in testing. Instead every Cartesian target is solved
to joint angles by this project's own kinematics.py/ik.py, and only
send_angles() is ever used to actually move the arm.
"""

from __future__ import annotations

import time

from pymycobot.mycobot280 import MyCobot280

from . import config, ik, kinematics


class Robot:
    def __init__(self, port: str = config.DEFAULT_PORT,
                 baud: int = config.DEFAULT_BAUD) -> None:
        self.mc = MyCobot280(port, baud)
        if self.mc.is_power_on() != 1:
            self.mc.power_on()
            time.sleep(1.5)

    def init_gripper(self) -> None:
        """Explicit, one-time gripper calibration. NOT called from
        __init__ -- pymycobot's init_gripper() can cycle the gripper,
        which would drop an already-grabbed pen if it ran on every
        connect. Call this once per session (pen_setup.py does) before
        the first grab_pen(), never after a pen is already gripped."""
        self.mc.init_gripper()

    # --- pose / homing ---
    def home(self) -> None:
        """Move to the documented safe fold (config.HOME_ANGLES)."""
        self._sync_send_angles(config.HOME_ANGLES, config.DEFAULT_SPEED)

    def get_pose(self) -> list[float]:
        """[x, y, z, rx, ry, rz] in mm/degrees, robot frame -- computed by
        our own forward kinematics over get_angles(), not firmware
        get_coords()."""
        return kinematics.forward_kinematics(self.get_angles())

    def get_angles(self) -> list[float]:
        """[J1..J6] in degrees."""
        return self.mc.get_angles()

    # --- pen handling (manual placement, M1) ---
    def grab_pen(self) -> None:
        """Close the gripper on a manually-placed pen, at the arm's
        current position -- no travel to a pickup location."""
        self._set_gripper(config.GRIPPER_CLOSED)

    def release_pen(self) -> None:
        self._set_gripper(config.GRIPPER_OPEN)

    def _set_gripper(self, value: int) -> None:
        # The firmware can silently drop a single gripper packet if it
        # isn't followed by a short quiet window; sending it twice with a
        # brief gap is a proven workaround on this hardware.
        self.mc.set_gripper_value(value, config.GRIPPER_SPEED)
        time.sleep(0.06)
        self.mc.set_gripper_value(value, config.GRIPPER_SPEED)
        deadline = time.time() + 3.0
        while self.mc.is_gripper_moving() == 1 and time.time() < deadline:
            time.sleep(0.1)
        time.sleep(0.2)  # settle before considering the grip done

    # --- pen up/down: move Z only, keep current X/Y and orientation ---
    def pen_up(self) -> None:
        x, y, _z, rx, ry, rz = self.get_pose()
        self._move_cartesian([x, y, config.SAFE_Z, rx, ry, rz], config.DEFAULT_SPEED)

    def pen_down(self) -> None:
        x, y, _z, rx, ry, rz = self.get_pose()
        self._move_cartesian([x, y, config.DRAWING_PLANE, rx, ry, rz], config.DEFAULT_SPEED)

    # --- motion ---
    def move_to(self, x: float, y: float, z: float,
                speed: int = config.DEFAULT_SPEED) -> None:
        """x, y are ORIGIN-relative drawing-plane mm offsets; z is
        absolute (e.g. config.DRAWING_PLANE/SAFE_Z). This is the only
        place the ORIGIN_X/Y offset is applied."""
        coords = [config.ORIGIN_X + x, config.ORIGIN_Y + y, z,
                  config.PEN_RX, config.PEN_RY, config.PEN_RZ]
        self._move_cartesian(coords, speed)

    def _move_cartesian(self, target_pose: list[float], speed: int) -> None:
        """Solve target_pose to joint angles with our own IK, seeded from
        the arm's current angles (keeps the solver on the same IK branch,
        e.g. elbow up/down, instead of jumping to a distant alternate
        solution), then execute purely in joint space -- never asks
        firmware to solve IK."""
        q_seed = self.get_angles()
        q_target = ik.inverse_kinematics(target_pose, q_seed)
        self._sync_send_angles(q_target, speed)

    def _sync_send_angles(self, angles: list[float], speed: int) -> None:
        self.mc.send_angles(angles, speed)
        deadline = time.time() + config.MOVE_TIMEOUT
        while time.time() < deadline:
            if self.mc.is_in_position(angles, 0) == 1:
                return
            time.sleep(0.1)
        raise RuntimeError(
            f"arm did not reach angles {angles} within {config.MOVE_TIMEOUT}s"
        )

    # --- safe stop / recovery ---
    def stop(self) -> None:
        self.mc.stop()

    def recover(self) -> None:
        """After an interrupted/aborted run: re-power and re-home."""
        if self.mc.is_power_on() != 1:
            self.mc.power_on()
            time.sleep(1.5)
        self.home()

    def close(self) -> None:
        self.mc.close()

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *exc) -> bool:
        self.stop()
        self.close()
        return False

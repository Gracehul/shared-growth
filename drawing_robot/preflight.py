"""Environment preflight that never opens a robot connection.

Run with ``python -m drawing_robot.preflight`` after installation. The optional
``--hardware`` flag checks whether the hardware dependency and serial device are
present, but still does not connect to the robot or issue motion commands.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import platform
import sys
from pathlib import Path


def _result(label: str, ok: bool, detail: str) -> bool:
    marker = "PASS" if ok else "FAIL"
    print(f"[{marker}] {label}: {detail}")
    return ok


def _mock_check() -> tuple[bool, str]:
    from . import Arm, SUCCESS

    with Arm(mock=True, control_rate_hz=100.0) as arm:
        coords = arm.get_coords()
        if len(coords) != 6 or not all(isinstance(v, float) for v in coords):
            return False, f"unexpected pose: {coords!r}"

        current = arm.get_angles()
        if arm.move_joints(current, duration=0.02) != SUCCESS:
            return False, arm.last_error or "mock joint move failed"

        command_count = len(arm.conn.raw.commands)
        if command_count < 2:
            return False, "mock backend did not receive the planned setpoints"

    return True, f"pose has 6 values; mock executed {command_count} setpoints"


def _hardware_checks(port: str) -> list[bool]:
    checks: list[bool] = []
    has_pymycobot = importlib.util.find_spec("pymycobot") is not None
    checks.append(_result(
        "pymycobot",
        has_pymycobot,
        "installed" if has_pymycobot else "missing; install with pip install -e '.[hardware]'",
    ))

    if os.name == "posix":
        exists = Path(port).exists()
        checks.append(_result(
            "serial device",
            exists,
            f"{port} exists" if exists else f"{port} not found",
        ))
    else:
        checks.append(_result(
            "serial device",
            True,
            f"existence check skipped on {platform.system()}; requested port is {port}",
        ))
    return checks


def build_parser() -> argparse.ArgumentParser:
    from . import config

    parser = argparse.ArgumentParser(
        description="Check the Shared Growth Python and mock setup without moving hardware."
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="also check pymycobot and the serial-device path; never connects",
    )
    parser.add_argument("--port", default=config.DEFAULT_PORT)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    checks: list[bool] = []

    checks.append(_result(
        "Python",
        sys.version_info >= (3, 10),
        f"{platform.python_version()} at {sys.executable}",
    ))

    try:
        import numpy as np
    except ImportError as exc:
        checks.append(_result("NumPy", False, str(exc)))
    else:
        checks.append(_result("NumPy", True, np.__version__))

    try:
        ok, detail = _mock_check()
    except Exception as exc:  # report setup failures in one readable place
        checks.append(_result("mock pipeline", False, f"{type(exc).__name__}: {exc}"))
    else:
        checks.append(_result("mock pipeline", ok, detail))

    if args.hardware:
        checks.extend(_hardware_checks(args.port))

    passed = sum(checks)
    print(f"\nPreflight: {passed}/{len(checks)} checks passed.")
    if not all(checks):
        return 1
    print("No robot connection was opened and no physical motion was requested.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

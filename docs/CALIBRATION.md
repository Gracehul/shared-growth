# Calibration workflow

The repository contains `config/calibration.example.json` as the canonical
field list. It deliberately contains `null` measurements and
`approved_for_hardware: false`.

## Create a local profile

Copy the example to `config/calibration.local.json`. Local profiles are ignored
by Git so incomplete or machine-specific values are not published by accident.
Once a profile has been measured, reviewed, and intentionally selected for the
shared setup, add a sanitized version through a pull request.

## Measurement order

1. Confirm robot model, serial device, baud rate, firmware, and software commit.
2. Run the read-only inspection and save its output outside the repository's
   tracked files.
3. Run FK verification at the current static pose. Do not move the robot merely
   to collect more poses at this stage.
4. Measure the flange-to-pen-tip tool offset.
5. Confirm the pen orientation relative to the flange.
6. Establish the paper plane and measure drawing Z.
7. Establish Safe Z with clearance from paper, holder, and fixtures.
8. Define a conservative rectangular XY workspace inside the reachable area.
9. Choose a low development speed before any study speed.
10. Review evidence before changing `approved_for_hardware` to `true`.

## Read-only commands

Mock verification:

```bash
python scripts/inspect_robot.py --mock
python scripts/verify_fk.py --mock
```

On the Jetson, after the non-connecting environment preflight and safety review:

```bash
python scripts/inspect_robot.py --port /dev/ttyTHS1
python scripts/verify_fk.py --port /dev/ttyTHS1
```

These scripts create an `ArmConnection(read_only=True)`. That mode refuses
power, fresh-mode, joint, gripper, stop, and servo-control commands. Closing the
connection closes the serial port without sending `stop()`. The pymycobot
backend is never exposed; all access is routed through `RobotIO`.

## Interpreting FK verification

Firmware coordinates describe the bare flange, while the drawing API describes
the configured TCP at the pen. The verification therefore compares firmware FK
with Python's **bare-flange** FK; including the pen offset would create a false
error. Default tolerances are 2 mm for the position-error norm and 2 degrees for
the largest wrapped orientation error.

A pass at one static pose confirms only that one observation. It does not prove
the DH model over the entire workspace. Multi-pose validation belongs to the
later supervised, low-speed test phase.

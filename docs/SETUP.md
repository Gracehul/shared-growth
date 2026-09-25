# Development and robot setup

The setup is intentionally split into a hardware-free development environment
and a robot/Jetson environment. Installing or running the local setup does not
connect to the myCobot.

## Requirements

- Git
- Python 3.10 or newer
- the repository checkout
- for real hardware only: myCobot 280 JN, its Jetson environment, and access to
  the configured serial device

## Local development and mock mode

### Windows PowerShell

```powershell
.\scripts\setup.ps1
.\.venv\Scripts\Activate.ps1
python -m drawing_robot.preflight
python scripts\example.py --mock
```

If Python is installed but not on `PATH`, pass its executable explicitly:

```powershell
.\scripts\setup.ps1 -PythonExecutable "C:\path\to\python.exe"
```

### Linux or macOS

```bash
bash scripts/setup.sh
source .venv/bin/activate
python -m drawing_robot.preflight
python scripts/example.py --mock
```

The local installation includes NumPy and the drawing package. `pymycobot` is
not required for mock development because the hardware import is lazy.

## Jetson installation

Run on the Jetson from the repository checkout:

```bash
bash scripts/setup.sh --hardware
source .venv/bin/activate
python -m drawing_robot.preflight --hardware --port /dev/ttyTHS1
```

If the installation uses the USB bridge instead:

```bash
python -m drawing_robot.preflight --hardware --port /tmp/ttyMyCobot
```

The hardware preflight checks that `pymycobot` and the device path exist. It
does **not** open the serial port, power the robot, or send a motion command.

## Current configuration inherited from the initial robotics work

These values are useful starting points, not proof that the present physical
setup is calibrated:

| Setting | Current value | Status before physical drawing |
| --- | --- | --- |
| Robot | myCobot 280 JN | Confirm model and firmware |
| Default port | `/dev/ttyTHS1` | Confirm on Jetson |
| Baud rate | `1,000,000` | Confirm connection |
| Tool offset | `(0, 0, 90)` mm | Measure pen-tip offset |
| Tool orientation | `(0, 0, -45)` deg | Confirm holder orientation |
| Drawing-plane Z | `3.0` cm | Placeholder; measure |
| Safe Z | `10.0` cm | Placeholder; verify clearance |
| Control rate | `25 Hz` | Confirm serial stability |
| Speed scale | `120 deg/s` at firmware 100 | Calibrate conservatively |

Copy [SETUP_RECORD.md](SETUP_RECORD.md) for each physical configuration and
fill measured values there. Do not replace `TBD` values by guesses.

## First hardware contact gate

Do not start with a drawing script. The first hardware session should stop after
these checks:

1. complete the safety checklist;
2. identify and record the serial device;
3. run the non-connecting hardware preflight;
4. inspect mount, gripper, pen holder, workspace, and stop access;
5. review every configured pose and limit;
6. connect and read state under supervision, without requesting motion.

Motion testing begins in a separate step with no pen, reduced speed, and a
pen-up trajectory.

## Known boundaries before hardware use

- `future/` contains useful earlier geometry and drawing ideas, but targets an
  older API and references configuration names that no longer exist. Do not run
  those scripts until they are deliberately ported.
- `scripts/draw_square.py` and `scripts/draw_circle.py` contain fixed placeholder
  coordinates. They are mock examples, not approved trajectories for the
  current physical workspace.
- `drawing_robot/config.py` refers to `scripts/verify_fk.py`, but that validation
  script is not present in the repository. FK comparison against the real robot
  therefore remains an explicit setup task.
- The tool offset, tool orientation, drawing-plane height, safe height, and
  speed conversion are inherited starting values. Each requires confirmation
  on the actual robot and pen holder.

## Troubleshooting

- `python` not found: install Python 3.10+ or provide its full path to the
  PowerShell helper.
- `pymycobot` missing locally: expected for mock-only setup; use the hardware
  extra only on the Jetson or a machine that needs robot communication.
- serial device missing: verify whether the system uses `/dev/ttyTHS1`, the
  `/tmp/ttyMyCobot` bridge, or another documented device.
- permission denied on Linux: inspect device ownership and the laboratory's
  serial-access procedure; do not run the whole project as root as a shortcut.

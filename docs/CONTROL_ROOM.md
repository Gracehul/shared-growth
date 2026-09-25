# Shared Growth Control Room

The Control Room is a local, read-only state dashboard for development and
supervised hardware tests. It deliberately exposes no motion endpoint, jog
button, stop button, error reset, or power control. Those actions remain in the
explicit test scripts and require a human operator at the robot.

## Signals in version 1

| Layer | Available values | Interpretation |
| --- | --- | --- |
| Joint space | encoder angle, reported speed, configured limits | actual joint state and proximity to a configured limit |
| Cartesian / flange | firmware `x, y, z, rx, ry, rz` | reported end-effector flange pose; not a calibrated pen-tip pose |
| Drive health | temperature, voltage, servo status | development diagnostics, not certified safety measurements |
| Controller | power, servo-enable state, controller error | whether the controller is reporting a usable state |
| Connection | sample duration, timestamp and data age | distinguishes stale UI data from fresh robot telemetry |

Motor current, joint torque, TCP force/moment and vibration are not displayed
because this setup has no verified source for those signals. The interface must
not estimate or invent them. A force-torque sensor or IMU can be added later as
a separate, explicitly labelled data source.

The 60 °C line is a conservative **project pause gate**, not a manufacturer
rating or a certified safety limit.

## Run without hardware

From the repository root:

```bash
python3 scripts/robot_dashboard.py --mock
```

Open `http://127.0.0.1:8765`. Mock mode is suitable for UI development and
contains no robot connection.

## Run on the Nano

First ensure no other program is using `/dev/ttyTHS1`, then run:

```bash
cd /home/er/shared-growth
source .venv/bin/activate
python scripts/robot_dashboard.py --robot-port /dev/ttyTHS1
```

The dashboard binds to the Nano's loopback interface by default. It is not
published to the Wi-Fi network. From the Windows computer, create an SSH tunnel:

```powershell
ssh -i C:\Users\baubu\.ssh\shared_growth_nano_v3_ed25519 -L 8765:127.0.0.1:8765 er@NANO_IP
```

Keep that terminal open and browse to `http://127.0.0.1:8765` on Windows.

## Operational rules

- `RobotService` owns the serial port and publishes state to the dashboard.
  The dashboard itself contains no pymycobot initialization or polling logic.
- An operating-system device lock prevents any second process from opening the
  same serial device. Stop the dashboard service before launching a standalone
  characterization process.
- A green dashboard means only that the implemented development gates are
  currently clear. It does not certify that motion is safe.
- If data becomes stale or the robot disconnects, treat the displayed values as
  historical and verify the physical robot before continuing.
- The dashboard is an observer. It cannot clear errors or authorize motion.

## Control boundary

Active control stays inside `RobotService` and requires explicit local
enablement, calibrated workspace limits, previewed trajectories and an
independent physical stop path. Joint targets, Cartesian targets, waypoints,
blend radii and motion profiles never belong in this read-only screen.

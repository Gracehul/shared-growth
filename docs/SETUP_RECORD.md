# Physical setup record

Create one copy per materially different robot, tool, surface, or camera setup.
Do not include participant data in this file.

## Identification

| Field | Value |
| --- | --- |
| Date/time (UTC) | 2026-09-25 |
| Operator GitHub username | Gracehul |
| Git commit | `33be6e6` at last hardware session |
| Robot model | myCobot 280 JN (2023) |
| Robot serial/asset pseudonym | TBD |
| Firmware version | System firmware `7.3`; basic version unavailable (`-1`) |
| Jetson OS/version | Jetson Nano; Ubuntu 20.04.6 LTS; exact module/RAM/JetPack TBD |
| Python version | Python 3.10 via pyenv environment |
| `pymycobot` version | `4.0.7` |
| Serial device | `/dev/ttyTHS1` |
| Baud rate | `1,000,000` |

## Physical configuration

| Field | Value |
| --- | --- |
| Robot mounting method | Table-edge clamp; final rigidity verification pending |
| Pen holder | Adaptive gripper present; pen/tool calibration pending |
| Pen type/color | TBD |
| Drawing surface size | TBD |
| Drawing surface fixation | TBD |
| Camera model/mount | Not required for Milestone 1 / TBD |
| Emergency stop or power control | TBD |
| Participant exclusion zone | TBD |

## Calibration

| Field | Value |
| --- | --- |
| Home joint angles | Old folded REST rejected as a thermal default; READY/PARK TBD |
| Tool offset `(x, y, z)` mm | TBD |
| Tool orientation `(rx, ry, rz)` deg | TBD |
| Drawing-plane Z cm | TBD |
| Safe Z cm | TBD |
| Drawing workspace X range cm | TBD |
| Drawing workspace Y range cm | TBD |
| Development speed limit | TBD |
| Study speed limit | TBD |
| Calibration method/version | TBD |

## Preflight evidence

- [x] Local mock preflight passed
- [x] Hardware dependency/device preflight passed
- [ ] Safety checklist completed
- [x] Stop behavior tested without participant
- [ ] Pen-up dry run passed
- [ ] Values above independently measured or confirmed

## Notes and anomalies

J4 reached 65 C after an aborted extended-pose Cartesian test. Cartesian
hardware execution remains disabled until READY and PARK are characterized.

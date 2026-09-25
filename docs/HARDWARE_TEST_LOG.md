# Hardware test log

This file records supervised hardware milestones. It is an engineering log,
not evidence of certification or compliance with an industrial safety standard.
Each entry should identify the software revision, robot state, commanded action,
measured result, operator observation, and the gate for the next test.

## Session 001 — Connection, inspection, and first controlled motion

**Date:** 2026-09-25

**Robot:** myCobot 280 JN (2023), adaptive gripper attached

**Controller:** Jetson Nano, Ubuntu 20.04.6 LTS

**Repository revision:** `eb39f96`

**Python / pymycobot:** Python 3.10.10 / pymycobot 4.0.7

**Connection:** `/dev/ttyTHS1`, 1,000,000 baud

**Atom/system firmware:** 7.3

### Read-only baseline

- Power and all servos reported enabled.
- Stable initial joint state (degrees):
  `[0.61, 111.53, -154.95, -36.56, -6.41, 12.30]`.
- Controller-reported J3 range: `[-150, 150]` degrees.
- Initial controller error: `17` (collision protection category).
- The robot was visibly parked in a tightly folded configuration.
- Firmware and Python forward kinematics agreed closely in orientation but
  differed by approximately 11.25 mm in position, mostly on Z. Cartesian
  drawing remains blocked until this discrepancy is characterized across
  multiple poses.

### Supervised recovery movement

The operator cleared the immediate workspace, remained at the robot, and gave
an explicit movement confirmation. A one-shot recovery script then:

1. verified the initial state, active servos, error `17`, and controller limits;
2. issued `stop()`;
3. cleared the stored collision-protection error;
4. commanded only J3 toward `-148.0` degrees at firmware speed `2`;
5. monitored joint state and controller error;
6. issued `stop()` immediately when error `3` appeared.

Measured result:

| Measure | Value |
| --- | --- |
| Initial J3 | `-154.95 deg` |
| Final J3 | `-149.06 deg` |
| Net J3 change | `+5.89 deg` |
| Target | `-148.0 deg` |
| Error during movement | `3` (J3 limit) |
| Error after stopped-state clear | `0` |
| Final state stability | Stable across three samples |
| Final servo state | Enabled |

Although a single-joint command was issued, telemetry also changed by about
`-0.96 deg` on J4 and `-0.70 deg` on J5. This may be settling, backlash,
coupling, or measurement behavior; it must be observed in later repetitions
rather than assumed harmless.

### Operator observation

The movement was physically acceptable with no reported unusual sound or
contact. It appeared sudden, and the end-to-end interaction included a long
delay. Most of that delay belonged to the development path (chat approval,
remote command dispatch, SSH, and process startup), not yet to the robot's
local motion API. Experimental timing must therefore be generated and measured
by a persistent controller running locally on the Nano.

### Outcome and next gate

**Milestone achieved:** authenticated remote development connection plus the
first bounded physical joint movement with automatic stop and post-state
verification.

Before Cartesian or pen-contact motion:

- create at least a 3-degree margin from every configured joint limit;
- measure command-to-first-motion, movement, and settling latency locally;
- repeat small joint-space moves at conservative speeds;
- test normal stop behavior;
- characterize the FK position discrepancy across several safe static poses;
- preserve machine-readable logs for every subsequent hardware test.

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

## Session 002 — J3 safety-margin test

**Date:** 2026-09-25

**Start condition:** J3 at `-149.06 deg`, controller error `0`, all servos
enabled, and the workspace supervised by the operator.

The test commanded only J3 toward `-145.0 deg` at the minimum firmware speed
setting `1`. It continuously sampled all joint angles and the controller error,
then issued `stop()`.

| Measure | Value |
| --- | --- |
| Initial J3 | `-149.06 deg` |
| Final J3 | `-145.98 deg` |
| Net J3 change | `+3.08 deg` |
| Time to first measured change | `< 0.19 s` after local command |
| Time to stable reported position | approximately `0.35 s` |
| Final distance from lower J3 limit | `4.02 deg` |
| Controller error throughout | `0` |
| Post-test state | stable across three read-only samples |

The script reported a timeout because its target tolerance was `0.8 deg` and
the firmware stopped `0.98 deg` short of the requested angle. This was a test
acceptance-threshold issue rather than a controller fault. The result exposes
an important characteristic for later testing: firmware speed `1` still
produced a visibly discrete, sub-second movement and did not converge exactly
to the requested single-joint target.

J5 changed by approximately `+0.70 deg` and J6 by `-0.09 deg` while J3 was
commanded. Cross-joint telemetry changes remain an open characterization item.

**Milestone achieved:** J3 now has more than the configured 3-degree margin
from its lower limit, with controller error `0`. L1 joint-limit recovery is
complete for this starting condition. The next recommended test is local
latency and repeatability measurement using a small, symmetric J6 motion after
checking cable clearance.

## Session 003 — Local J6 latency, repeatability, and stop

**Date:** 2026-09-25

A persistent Nano process alternated J6 by `+2 / -2 deg` around its measured
center. All timestamps used the Nano's monotonic clock; chat, SSH setup, and
process-launch delay were outside every trial measurement.

The main run completed 29 moves before one `get_angles()` call returned the
known sporadic scalar `-1` instead of six angles. The gate stopped the test with
controller error `0`. Bounded telemetry retries were then added and a five-move
Speed-10 supplement completed successfully.

| Speed | Valid moves | Mean first motion | Mean settled | Max absolute final error | Max overshoot | Max other-joint drift |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 2 | 11 | `0.357 s` | `0.611 s` | `0.77 deg` | `0.00 deg` | `0.18 deg` |
| 5 | 11 | `0.456 s` | `0.687 s` | `0.68 deg` | `0.00 deg` | `0.18 deg` |
| 10, initial run | 7 | `0.278 s` | `0.564 s` | `0.68 deg` | `0.00 deg` | `0.18 deg` |
| 10, retry-enabled supplement | 5 | `0.353 s` | `0.585 s` | `0.77 deg` | not observed | within gate |

For these short moves, a larger firmware speed did not produce a monotonic
decrease in measured latency. Serial sampling, command quantization, and the
approximately 0.6–0.8 degree endpoint deadband are material parts of the
observed system.

The application-level stop test commanded a `+4 deg` J6 move at Speed 2 and
sent `stop()` at the first detected movement:

| Measure | Value |
| --- | ---: |
| First movement detected | `0.103 s` after command |
| Stop call duration | `0.044 s` |
| Stable after stop call | `0.273 s` |
| Movement before stop detection | `0.61 deg` |
| Additional movement after detection | `1.94 deg` |
| Final controller error | `0` |

This is an application-level functional measurement, not a safety-rated stop
validation. Development clearance must include the observed post-detection
travel plus sensing uncertainty and an explicit margin.

## Session 004 — Partial multi-pose FK and J4 thermal anomaly

**Date:** 2026-09-25

The five-pose FK sequence sampled three nearby poses before its J4 transition
failed the movement timeout. Across the three valid poses:

- position-error norm stayed within `11.337–11.342 mm`;
- Z error stayed within `-11.193 to -11.183 mm`;
- controller error remained `0`.

This narrow J6-only variation supports a nearly constant offset hypothesis but
is not sufficient to validate it across the workspace.

J4 moved only about `0.08 deg` toward a `2 deg` target at Speed 5 and again
timed out. Read-only diagnostics reported all servos enabled, all servo status
flags `0`, and controller error `0`. J4 temperature rose from `62 C` to `66 C`
while the system was holding its folded configuration; other joints were
`39–54 C`.

No manufacturer temperature limit was found in the public API documentation,
so `60 C` is now a conservative **project development gate**, not a claimed
device rating. Hardware motion is paused pending cooldown and investigation of
J4 load, posture, and command response. All hardware runners must reject motion
when any reported joint temperature is at or above this gate.

## Session 005 — REST / UPRIGHT state cycle

**Date:** 2026-09-25

The measured folded pose was stored as `REST`; the manufacturer's calibrated
zero pose `[0, 0, 0, 0, 0, 0]` was used as `UPRIGHT`. The outward leg settled
with a maximum joint error of `1.31 deg`. A conservative 55 C test gate paused
the return when J4 reached that value. A separately logged recovery used the
established 60 C project gate and returned to the exact recorded REST target.

| Measure | REST to UPRIGHT | Thermal recovery to REST |
| --- | ---: | ---: |
| Settled time | `8.420 s` | `5.876 s` |
| Maximum final joint error | `1.31 deg` | `0.70 deg` |
| Maximum observed temperature | `54 C` | `58 C` |
| Controller errors | `0` | `0` |

The final static Python-versus-firmware flange-position error was `10.36 mm`
at UPRIGHT and `11.32 mm` at REST; maximum static orientation error was at most
`0.01 deg`. Dynamic comparisons are not treated as FK validation because joint
angles and firmware coordinates were read sequentially while the arm moved.

## Session 006 — Threaded telemetry and joint wave

**Date:** 2026-09-25

Telemetry v2 started before the first command and shared pymycobot's
thread-safe serial connection with asynchronous motion commands. One cycle ran
`REST -> UPRIGHT -> 13-frame wave -> UPRIGHT -> REST`. J1 and J6 remained fixed;
the commanded wave amplitudes were J2/J3 `6 deg`, J4 `2 deg`, and J5 `5 deg`.

| Measure | Result |
| --- | ---: |
| Total duration | `22.385 s` |
| Telemetry samples | `380` |
| Mean sample interval | `0.059 s` |
| Motion commands | `14` |
| Command API duration, outward / inward | `0.00038 / 0.00033 s` |
| First observed motion, outward / inward | `0.150 / 0.151 s` |
| Settled time, outward / inward | `5.694 / 5.648 s` |
| Final REST error | `0.70 deg` maximum |
| Maximum temperature | `51 C` |
| Controller errors / retry samples | `0 / 0` |

During the wave window, observed joint ranges were J2 `-4.57..4.21 deg`, J3
`-1.40..5.71 deg`, J4 `-2.46..0.35 deg`, and J5 `-4.74..1.31 deg`. J4 rose only
from `37 C` to `43 C`; J5 was the warmest joint at `49–51 C`. The successful
cycle validates the asynchronous command plus threaded telemetry architecture
for the next pen-up Cartesian motion tests. It does not yet validate drawing
contact, calibrated TCP position, or safety-rated stopping.

## Session 007 — Aborted pen-up Cartesian S-curve

**Date:** 2026-09-25

The first model-based XZ S-curve used a 32 mm by 14 mm path, an 8 Hz joint
waypoint stream, firmware Speed 10, and continuous telemetry. The robot reached
the intermediate center and completed the forward command stream. It did not
settle at the final forward target within the 10-second gate, so the runner
called `stop()` and did not execute the reverse path or the return to REST.

| Measure | Result |
| --- | ---: |
| Center transition: first observed motion | `0.604 s` |
| Center transition: settled | `2.925 s` |
| Telemetry samples | `256` |
| Motion commands | `61` |
| Mean command API duration | `0.136 s` |
| Maximum command API duration | `1.468 s` |
| Commands taking more than 100 ms | `11` |
| Final target error | `4.43 deg` maximum |
| Controller errors | `0` |

The observed command delays show that the 8 Hz command stream and concurrent
telemetry saturated the shared serial connection. The test therefore does not
characterize the intended geometric S-curve or its motion style.

After stopping, the robot remained motionless at approximately
`[1.31, 50.71, -70.75, -21.00, -10.89, 7.73] deg`. J4 then showed delayed
thermal rise from `44 C` during the logged path to `61 C`, then `64 C`, and
finally `65 C` while holding that extended posture. Controller error remained
`0` and all measured joint speeds remained `0`. The 60 C project gate correctly
prevented an automatic return. The operator supported the arm and powered the
robot down; no further motion command was sent.

Before retrying Cartesian motion:

- replace the old folded REST concept with separate low-load READY and
  mechanically supported PARK states;
- characterize each candidate state thermally before a path test;
- coordinate serial reads and commands or lower the stream and telemetry rates;
- require a cool start and retain the 60 C automatic abort gate;
- use a supervised recovery only below the project temperature gate.

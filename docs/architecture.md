# Architecture

## Split of responsibilities

The one rule that shapes everything: **nothing that must happen within 100 ms
is allowed to depend on Wi-Fi.**

| Layer | Runs on | Rate | Owns |
|---|---|---|---|
| Sensing | ESP32 | 100 Hz | BNO085, QMC5883L, GPS NMEA, 4× INA226, e-stop inputs |
| Estimation | ESP32 | 100 Hz | fused heading, fused speed, dead-reckoned odometry |
| Control | ESP32 | 100 Hz | heading PID, speed PI, motor PWM, safety cut-off |
| Planning | Ubuntu / ROS 2 | 5 Hz | which waypoint, what bearing, what speed |
| Supervision | Ubuntu / ROS 2 | 10 Hz | heartbeat, e-stop latch, GPS quality gate |
| Interface | Ubuntu / ROS 2 | 5 Hz | web UI, settings files, waypoint editor, log |

If the network dies mid-drive the robot keeps the last heading for at most
600 ms and then brakes — it never swerves because a packet was late.

## Heading convention

**Compass degrees everywhere**: 0 = north, 90 = east, positive = clockwise.
Same units as GPS course over ground and as the bearing to a waypoint, so the
customer-visible numbers match a phone compass.

Conversions happen only at the edges:

* BNO085 reports yaw counter-clockwise → `IMU_YAW_TO_COMPASS_SIGN = -1`.
* ROS odometry is published in ENU (REP-103, x east, y north, yaw CCW from
  east) → `StateEstimator::headingEnuRad()` and `geo.compass_to_enu_deg()`.

## Estimating speed and wheel speed without encoders

This robot has no wheel encoders (outdoor build, draft_5). Two sources are
fused in `firmware/lib/estimator/state_estimator.h`:

```
speed  = ∫ IMU forward acceleration           (fast, drifts)
       + pull towards GPS ground speed        (slow, drift free, gps_tau_s)
       + pull towards the motor model         (used only when the fix is lost)

wheel_left  = speed − yaw_rate · track / 2
wheel_right = speed + yaw_rate · track / 2     yaw_rate from the BNO085 gyro
```

So the wheel RPM shown on the web UI is *derived*, exactly as requested
("find wheel speed from IMU"). It is good enough for monitoring and for the
speed loop, not for precise odometry — the GPS is the position truth.

## Heading estimation — all sources combined, with fallbacks

The gyro is always the backbone: it is fast, quiet and never lies about
*change*. What it cannot do is tell you where north is, so an absolute
reference is eased onto it. Three independent references exist, and the
estimator uses the best one that is currently healthy:

| Priority | Source | Works when | Weakness |
|---|---|---|---|
| 1 | GPS course over ground | robot is moving faster than `align_min_mps` | useless standing still, noisy at low speed |
| 2 | QMC5883L in the GPS module | any time, including standing still | magnetic interference (rejected by the field check) |
| 3 | BNO085 magnetometer (rotation vector) | any time | sits next to the motors, so it is the noisiest |
| 4 | none — gyro only | always | drifts, heading is relative; flagged on the web UI |

```
heading_source = 0   BNO085 rotation vector only
                 1   gyro + GPS course only
                 2   GPS course only
                 3   gyro + compass when still + GPS when moving
                 4   AUTO: the table above, health checked  ← default
```

Losing a sensor degrades the heading, it never stops the robot:

* **compass dies** → standing-still reference falls back to the BNO085
  magnetometer; once moving, GPS course takes over anyway
* **GPS dies** → compass keeps the heading absolute, navigation blocks (it has
  no position) but manual driving still works
* **IMU dies** → heading comes straight from the GPS course while moving, or
  from the compass while stopped; the PID keeps running on that
* **everything magnetic dies** → gyro-only relative heading, `heading_relative_only`
  is raised and shown on the UI

Same idea for speed: `IMU + GPS` → `GPS only` → `IMU + motor model` →
`motor model only`, reported in `speed_source`.

The "motor model" is not a guess. Each motor has its own INA226, so the
firmware knows the rail voltage and the current going into that one wheel, and
a brushed DC motor obeys `V = I·R + Ke·ω`. Rearranged:

```
wheel_rpm = ((V_bus − V_diode) · duty − I · R) / Ke     Ke = 12 V / 100 rpm = 0.12
```

`V_diode` is the series diode that sits between the sensor and the driver in
schematic draft_5 (`Vf + I·R_slope`, set per motor — they can be different
parts). Without subtracting it the estimate reads several percent high, and
much worse at low duty.

That gives a **per-wheel** speed with no encoder, available the instant the
wheels turn, and it reacts to load: a slope or long grass draws more current
and the estimate drops, exactly like an encoder would show. It also detects
two failures nothing else on this robot can see:

* **stall** — high current, no back-EMF → the wheel is blocked
* **open circuit** — duty commanded but almost no current → broken wire, dead
  driver channel, motor fell off

Accuracy is 10–20 % (R and Ke drift with temperature, the INA226 averages over
~18 ms), so it is a fallback and a diagnostic, not a replacement for GPS.

Why the fast loop still runs on the gyro and not on the compass: a magnetometer
reading is disturbed by the motor current a few centimetres away and updates at
tens of Hz with heavy filtering. Feeding that into a 100 Hz PID makes the robot
weave. The compass corrects the *offset* slowly (`mag_tau_s`, seconds), the gyro
handles the *dynamics*.

## Driving straight

```
bearing to waypoint ──► /cmd_move (heading, speed) ──► ESP32
                                                        │
   heading error = bearing − fused heading              │ 100 Hz
   |error| > 45°  → turn on the spot                    │
   otherwise      → PID(error) = differential PWM       │
   speed error    → PI + feed-forward = common PWM      ▼
                                       left = base + diff, right = base − diff
```

Saturation keeps the ratio: if one side would exceed full scale both sides are
scaled down, so the robot never veers just because it asked for too much power.

Two ramps sit between a command and the wheels:

1. **setpoint slew** `SPEED_SLEW_MPS_S` (0.6 m/s per second) — the speed target
   itself cannot jump
2. **motor acceleration ramp** `MOTOR_ACCEL_RAMP_S` (0.6 s stopped → full PWM,
   0.3 s back down) — applied to the PWM that actually reaches the TB6612, so
   inrush current and gearbox shock are limited whatever the controller asks
   for. Starting from rest the output jumps to `PWM_MIN_MOVE` first (breakaway
   torque) and ramps from there; a direction reversal always ramps through zero.

An emergency stop bypasses both — `motor.disable()` is called directly.

## ROS graph

```
ESP32 ──/telemetry ──┬─► web_server ──► browser (SSE)
      ──/gps/fix ────┼─► waypoint_nav ──► /cmd_move ──► ESP32
      ──/imu ────────┤   gps_odom ──► /odom/gps
      ──/odom ───────┘   safety_watchdog ──► /heartbeat, /estop ──► ESP32
browser ──/api/heartbeat──► web_server ──► /web/heartbeat ──► safety_watchdog
```

Every topic, message layout and HTTP endpoint: [topics.md](topics.md).

## Configuration flow

```
package config/*.yaml   (defaults, read only, shipped with the code)
        │  first run: copied
        ▼
~/.gps_localize/*.yaml  (live settings, written by the web UI)
        │  POST /api/config
        ├─► /gps_localize/config/updated ──► nodes reload themselves
        └─► /gps_localize/config/pid ─────► firmware applies new gains live
```

No rebuild, no reflash, no ROS parameter files to hand-edit. `configstore.py`
merges the defaults underneath the live file, so a software update that adds a
new setting cannot break an existing installation.

## Memory policy

**Firmware**

* no `String`, no `malloc`/`new` after `setup()`
* every ROS message owns a static buffer (`telemetry_buf`, `config_buf`, frame
  id strings); `Float32MultiArray` capacity is fixed at compile time
* the NMEA parser uses one 96-byte line buffer and consumes at most 160 bytes
  per control cycle, so a noisy GPS burst cannot stall the loop
* micro-ROS pools are trimmed in `firmware/colcon.meta` (history 2, 8 pubs/subs)
* free heap and loop time are published in the telemetry array — if the heap
  number on the web footer drifts down over hours, something leaks

**ROS / web**

* `deque(maxlen=…)` for the log, bounded SSE queue per client, capped client
  count (`web.max_clients`), oldest frame dropped for slow clients
* no request-scoped state kept between calls; the snapshot dict is rebuilt,
  not accumulated
* the browser caps the trail (`TRAIL_MAX`) and the log DOM nodes (`LOG_MAX`)

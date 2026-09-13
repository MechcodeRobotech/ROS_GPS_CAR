# Bugs and problems found by Claude

Reviewed 2026-09-12 by Claude Code (Opus 5). **Nothing here has been fixed** —
this is a report, as asked.

## Scope and method

Read, in full: `firmware/src/safety/safety_manager.h`,
`firmware/src/estimator/pose_ekf.h`, `firmware/src/control/drive_controller.h`,
the heading and speed sections of `firmware/src/estimator/state_estimator.h`,
and the control-callback and safety-assembly sections of
`firmware/src/main.cpp`. Spot-checked the host nodes and the web settings
plumbing with targeted searches. A few findings are backed by measurements taken
on the live robot today; the rest are from reading the code, and each says which.

Deliberately **not** covered, so nobody assumes it was: `web/app.js` beyond the
settings table, `qr.py`, `tools/` other than QC, and the firmware sensor drivers
in `firmware/lib/`. Anything already described as fixed in `CLAUDE.md` or
`HANDOFF.md` is excluded.

---

## 1. HIGH — automatic steering inverts its own feedback when driving in reverse

`firmware/src/control/drive_controller.h:174`

```c
float diff = heading_.compute_with_error(heading_error_);
if (target_speed_ < 0.0f) diff = -diff;   // steering flips when reversing
```

For a differential drive the yaw rate is `(v_right - v_left) / track`, and that
does not change sign with the direction of travel. With `left = base + diff` and
`right = base - diff`, a positive `diff` turns the robot clockwise whether
`base` is positive or negative. Flipping `diff` in reverse therefore turns the
heading PID from negative feedback into positive feedback: the heading error
grows instead of shrinking, and the robot turns away from its target until it is
spinning.

**The same file argues the opposite for manual driving**, at lines 316-317: "The
sign works for reverse without a special case: heading responds to the wheel
difference the same way whichever direction the robot is travelling." That is
the correct statement, and `test_manual_hold_works_in_reverse`
(`firmware/test/test_native/test_drive_control.cpp:421`) proves it for the
manual half. Nothing tests the automatic half in reverse.

*Unless* the intent is that a negative speed means the robot's rear becomes its
front, so `target_heading` refers to the direction of travel — but then the
measured heading would need the same 180° flip, and it does not get one.

**How it fails in practice:** the first time a mission commands a negative
speed, the robot turns the wrong way and keeps turning.

**Why it has not bitten yet:** `waypoint_nav` never commands a negative speed. A
search of the node finds only `speed = self.cruise_speed`
(`waypoint_nav.py:484`), so this path is currently unreachable from the mission
code. Exposure is low today; severity is high the moment reverse is used.

**Verified:** read the code, checked the native tests for reverse coverage,
searched `waypoint_nav.py` for negative speed commands.

---

## 2. HIGH — the position EKF is corrected only on a fix good enough for *speed*

`firmware/src/main.cpp:1408` (the gate) and `main.cpp:1288` (what the gate is)

```c
if (gps_ok && gps.fixQuality() > 0) {        // gps_ok is the SPEED gate
    ...
    pose_ekf.updateGps(fix_east, fix_north, gps.hdop(), gps.fixQuality());
}
```

`gps_ok` requires 6 satellites and HDOP ≤ 1.5 (`GPS_MIN_SATS_FOR_SPEED`,
`GPS_MAX_HDOP_FOR_SPEED`). Those thresholds are strict on purpose, because a
marginal fix reports nonsense *velocities* — that is documented at length in
`firmware/config/gps.h`. T198 already split this from the separate,
position-grade `gps_fix_ok` (four satellites, no HDOP term) for the health flag,
but **this call was not changed with it**.

**How it fails in practice:** on a usable but rough fix the EKF receives no
correction at all and runs as pure dead reckoning, while `odom_x`/`odom_y` and
`/odom` keep being published as though they were fixes. Measured on this robot
today: indoors 4 satellites at HDOP 2.73, outdoors earlier 7 satellites at HDOP
1.8 — both real positions, both refused by this gate.

The EKF already scales its trust by HDOP and fix quality
(`pose_ekf.h:217 measurementVariance`), so a rough fix would be absorbed
weakly rather than believed. Refusing it entirely is strictly worse than letting
the filter weigh it.

**Verified:** read both gates; the satellite and HDOP values are from live
telemetry taken today.

---

## 3. MEDIUM — a welded contactor is reported but never stops the robot

`firmware/src/safety/safety_manager.h:169-189`

`allow_` is decided at lines 169-175 from the flag word `f`. The
contactor cross-check runs *after* that, at lines 182-189, and only then ORs
`STOP_CONTACTOR_STUCK` into `f`. Since `f` is rebuilt from zero on every call
(line 92), that bit is never present when `allow_` is computed — on this cycle
or any other.

So `STOP_CONTACTOR_STUCK` appears in `flags()` for display and changes nothing:
`allowMotion()` ignores it, and `runState()` (line 250) does not list it either.

**How it fails in practice:** the contactor has welded shut, motor power cannot
be removed by any means the firmware has — and the robot is still allowed to
drive. That is the one fault where continuing to drive is least acceptable, and
the check's own comment says the point is not to report "a safely stopped
robot".

**Fix shape (not applied):** move the cross-check above the release block so the
bit is part of `f` when `allow_` is decided.

---

## 4. MEDIUM — the update lockout drops the rails without setting a stop flag

`firmware/src/safety/safety_manager.h:241-244`

`beginUpdateLockout()` opens the contactor and holds it open until reboot, but
sets no bit in `f`. `allowMotion()` can therefore still return true while the
rails are deliberately dead, and the control loop keeps computing and applying
PWM into unpowered drivers. It also means the state is invisible in `flags()`,
so the web UI cannot say why the robot has gone quiet.

Real-world harm is small, because flashing blocks the loop anyway. It is
reported because a safety state that the safety flags cannot see is the kind of
gap that matters later, and because it interacts with finding 3: during the
lockout the rails take time to fall, which can assert `STOP_CONTACTOR_STUCK`
spuriously — and that bit, per finding 3, does nothing.

---

## 5. MEDIUM — the UI permanently tells the user to update the host, wrongly

`gps_localize_ws/src/gps_localize/gps_localize/telemetry_split.py:60` against
`firmware/config/version.h:25-27`

```python
HOST_EXPECTS_FIRMWARE = 104          # rendered as "1.4"
```
```c
#define FIRMWARE_VERSION_MAJOR  1
#define FIRMWARE_VERSION_MINOR  61
#define FIRMWARE_VERSION  ((FIRMWARE_VERSION_MAJOR) * 100 + (FIRMWARE_VERSION_MINOR))
```

The board reports 161 and the host expects 104, so `raw > HOST_EXPECTS_FIRMWARE`
is true on every boot and the host logs, and the web UI shows, *"board firmware
1.61 is NEWER than this host expects (1.4) - update the host with git pull"*.
The advice is false: pulling changes nothing, because the constant is simply
stale. Observed live in `journalctl -u gps_localize` today.

A warning that is always on is a warning nobody reads, which is the point of the
project's own rule about documentation that disagrees with the code. Nothing in
`tools/qc.py` ties the two constants together, so nothing catches it.

---

## 6. MEDIUM — `nav.reverse_allowed` is a setting that does nothing

Present in `gps_localize_ws/src/gps_localize/config/nav.yaml:29`, in
`webapp/settings/nav.yaml:9`, and shown on the Settings page as
"ถอยหลังได้ / Reverse allowed" (`web/app.js:3376`). A repository-wide search for
`reverse_allowed` finds those three and **no code that reads it** —
`waypoint_nav.py` does not load it in `_load_config`, and the firmware has no
equivalent.

**How it fails in practice:** the operator toggles it and believes reverse
driving has been enabled or disabled. Neither happens. The live
`~/.gps_localize/nav.yaml` currently has it set to `true` while the shipped
default says `false` and comments "keep false: this robot steers by driving
forward" — so the stored configuration already disagrees with the intent, with
no effect either way.

Related to finding 1: if this setting were ever honoured, it would be the thing
that exposes the reverse steering bug.

---

## 7. MEDIUM (suspicion) — the web server copies a live deque without the lock

`gps_localize_ws/src/gps_localize/gps_localize/web_server.py:723`, reached from
`:760`

`_topic_rate()` does `stamps = list(st['stamps'])`, where `st['stamps']` is a
`deque(maxlen=20)` created at line 780 and appended from the ROS executor
thread. It is called **with** `self._lock` held at line 701 (`api_topics`) and
**without** it at line 760 (`api_topic_echo`).

Copying a deque while another thread appends to it can raise
`RuntimeError: deque mutated during iteration`. That would surface as an
intermittent HTTP 500 on the Topics page, most likely on a fast topic.

**Not reproduced.** Proving it needs a high-rate topic being echoed plus a tight
poll of `/api/topic`; the inconsistent locking between the two call sites is the
evidence that one of them is wrong.

---

## 8. LOW/MEDIUM — the heading reference is a latch, not a statement about now

`firmware/src/estimator/state_estimator.h:283-334`

In the fallback chain, `heading_ref_` is written only inside a branch that
actually applies an absolute reference. Standing still with no compass and no
GPS course, no branch runs, and the previous value persists — so telemetry and
the web UI keep reporting, say, `REF_GPS` while the heading is being carried by
the IMU alone and drifting.

This is the same shape as the host-side `live_fix_source()` latch that was
already fixed for position: the field answers "what was used at some point"
rather than "what is being used now". A freshness timeout on the last applied
reference would make it honest.

---

## 9. LOW — the automatic speed loop lacks the guard the manual one has

`firmware/src/control/drive_controller.h:168` against the same file's
lines 354-367

Manual driving refuses to close the speed loop unless the speed source is
GPS-backed, and the comment records why in measured detail: on the motor-model
and IMU estimate the loop chased noise and PWM surged between 771 and 464 on a
constant command, and the estimate read negative while the robot was plainly
moving forward.

Automatic driving calls `speed_.compute(speed_ramp_, est.speed())` with no such
check. Whenever the GPS speed gate closes — indoors, under trees, in the noise
that finding 2 also concerns — the automatic controller is closing a loop on
exactly the estimate that was measured to be unusable.

---

## 10. LOW — `millis() == 0` is used as "not started" in six timers

`safety_manager.h:128` (`batt_low_since_`), `:170` (`last_clear_ms_`),
`:185` (`relay_open_since_`), `:290` (debounce `change_ms`),
`drive_controller.h:276` (`turn_ok_ms_`), `main.cpp:1417` (`last_cmd_ms == 0`).

Each restarts its timer if the event lands on the one millisecond in 49.7 days
where `millis()` wraps to zero. Negligible in probability, listed only because
one of them is the battery cutoff hold and one is the command timeout, and
because the pattern is repeated six times rather than solved once.

---

## 11. LOW — the position EKF carries a heading variance nothing reads

`firmware/src/estimator/pose_ekf.h:126-127`, `:233`

`p_hh_` is grown by the process noise and capped against the caller's
`heading_var`, and then never used: heading uncertainty enters the position
covariance only through the `heading_var` argument at line 118. Dead state.

In the same update, `p_xx_` and `p_yy_` are floored at `1e-4` (line 189-190),
a 1 cm sigma. That is far tighter than the 0.8 m measurement floor the same file
argues for at line 55, and it is precisely the mistake that made the host-side
fusion ignore GPS until a `min_sigma_m` floor was added — worth a look before it
is trusted outdoors.

---

## 12. LOW — the EKF covariance is silently symmetrised

`firmware/src/estimator/pose_ekf.h:186-191`

`(I - K) P` is computed for three entries and one off-diagonal is stored.
`K` and `P` are each symmetric, but their product is not symmetric in general,
so `P` after the update is not exactly the matrix the arithmetic assumes.
Harmless at this scale and with this gate, but it means the covariance is an
approximation in a way the code does not say.

---

## 13. LOW — the manual heading hold is not cleared when the mode changes

`firmware/src/control/drive_controller.h:64-108`

`setAuto()`, `setTurnTo()` and `stop()` all leave `hold_active_` and
`hold_heading_` untouched. Two consequences:

- `holdingHeading()` — published in telemetry — can report true while the robot
  is driving a mission, which is simply wrong.
- If manual driving resumes without passing through the "stopped" branch at
  lines 320-325, the first straight command re-uses a `hold_heading_` latched
  before the mission, and the robot steers to a heading nobody asked for.

In practice the stopped branch usually clears it first, which is why this is
low rather than medium.

---

## Things checked and found sound

Worth recording so the next reader does not spend the time again:

- The battery cutoff latch, its hold time, and the reasoning for not
  self-clearing (`safety_manager.h:127-140`).
- Input pull direction derived from the active level (`:47-60`).
- The relay being written low before the pin becomes an output (`:65-70`).
- The speed ladder and its creep floor, including why the floor is conditional
  (`state_estimator.h:154-193`).
- The Mahalanobis gate and its ability to give up (`pose_ekf.h:164-173`).
- Saturation handling that preserves the left/right ratio
  (`drive_controller.h:180-185`).
- Breakaway and reversal handling in the PWM ramp (`:243-262`).

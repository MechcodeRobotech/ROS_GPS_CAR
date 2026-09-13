# Repository bug review

Reviewed on September 12, 2026. Locations refer to the working-tree files, including existing uncommitted changes.

I read `CLAUDE.md` and `HANDOFF.md` and excluded the problems they describe as fixed. This review used source inspection, isolated Python execution, and read-only checks. No robot processes were started, no commands were published, and no hardware was accessed. Only this report was written; the instruction to leave other files untouched also applies to `HANDOFF.md`.

The findings below are demonstrated code defects, not claims that these failures were observed on the physical robot. Firmware findings were verified by tracing the actual source and, where stated, evaluating its arithmetic; the firmware was not compiled or executed. Physical outcomes are conditional on the described operating conditions.

## 1. High: Turn-to-heading commands bypass command timeout and have no maximum duration

**Locations:** `firmware/src/main.cpp:1427`, `firmware/src/control/drive_controller.h:83`, `firmware/src/safety/safety_manager.h:107`.

`setTurnTo()` sets both target speed and target yaw rate to zero while enabling `turn_in_place_`. The safety input `motion_requested` checks only those two numerical targets. Consequently, it is false throughout a turn-to-heading operation, and the safety manager never raises `STOP_CMD_TIMEOUT` for that operation. `updateTurnInPlace()` continues producing opposite wheel commands until the heading reaches its tolerance; it has no independent overall timeout.

On the robot, a turn can continue after its command producer disappears while the independent browser/watchdog heartbeat remains healthy. If the heading is frozen, unavailable, or never reaches the target, the turn can continue indefinitely. The one-shot `/api/turn` interface makes this especially relevant: normal turns intentionally outlive a single command, but there is no bounded failure path. Heartbeat loss and emergency inputs still stop the robot; this defect bypasses the separate command supervision layer. Telemetry also labels the turn as idle because run-state selection uses the same false motion flag.

**Verification:** I traced `cmd_move_cb()` at `main.cpp:604` through `setTurnTo()`, the safety input construction, and `SafetyManager::update()`. With `turn_in_place_=true`, both targets zero, and a command age greater than the configured 500 ms timeout, the timeout condition evaluates to false. The turn-output branch remains active independently of those targets.

## 2. High: A temporary I2C outage can permanently remove battery protection

**Locations:** `firmware/lib/ina226/INA226.h:60`, `firmware/lib/ina226/INA226.h:79`, `firmware/src/power/power_monitor.h:79`, `firmware/src/safety/safety_manager.h:127`.

After six consecutive read failures, `INA226::fail()` sets `present_` to false. Every subsequent `read()` immediately returns false without attempting another transaction. `PowerMonitor` has no recovery path, and `power.begin()` is called only during setup. When both motor-rail monitors reach this state, `pack_valid_` remains false for the rest of the boot, even after the bus recovers.

The safety manager treats invalid battery data as a reason to skip the hard cutoff and clear the soft-stop condition. It does not distinguish an intentionally unpowered rail from failed measurement while the relay is closed. With otherwise healthy safety inputs and no already-latched battery cutoff, driving remains permitted. A shared-bus interruption can therefore leave the robot driving until its battery is over-discharged, without restoring protection when communication recovers. The contactor cross-check also loses its measurement.

**Verification:** I traced the sixth-error transition, the early return in `read()`, both motor-validity branches, and both battery gates. A repository search found the sole `power.begin()` call in setup at `main.cpp:2193`; normal operation only calls `power.update()` at line 1291. This finding concerns sensor-failure recovery and fail-open protection, not the already-fixed INA address mapping or use of the processor rail.

## 3. High: Reverse travel is fed into the estimator as positive forward speed and forward-facing GPS course

**Locations:** `firmware/src/main.cpp:1370`, `firmware/src/estimator/state_estimator.h:160`, `firmware/src/estimator/state_estimator.h:283`.

The NMEA parser supplies ground-speed magnitude. `main.cpp` passes that value directly to `gps_speed`, and the estimator pulls signed body speed toward it without determining travel direction. The heading alignment similarly treats course over ground as the direction the nose faces. During reverse travel, course is approximately 180 degrees away from nose heading.

With usable GPS, a reversing robot therefore reports positive body speed. The enabled manual speed controller compares a negative command against this positive estimate and requests excessive reverse correction. Odometry initially integrates displacement in the wrong direction. Above the GPS alignment threshold, the heading reference also starts rotating toward the reverse course, corrupting heading hold. This affects ordinary outdoor reverse commands, not just malformed inputs.

**Verification:** I traced the magnitude conversions in `gps_nmea.h:230` and `gps_nmea.h:241`, the unchanged assignment in `main.cpp`, and the speed and heading corrections. An isolated arithmetic reproduction of the speed update, starting at -0.30 m/s with a +0.30 m/s GPS magnitude, zero acceleration, a 10 ms step, and a 1.5 s time constant, produced +0.299972 m/s after 15 seconds. The incorrect positive equilibrium does not depend on that particular time constant. `DriveController::updateManual()` closes its speed loop on this GPS-backed estimate; `MANUAL_SPEED_PID` is enabled in `firmware/config/control.h:95`.

## 4. High: Heading discovery drives despite a rejected or stale GPS fix

**Locations:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:497`, `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:509`.

`needs_discovery()` requires only that a position has existed and the heading is relative. `tick()` dispatches the discovery state before its normal `fix_usable()` gate. `tick_discover()` checks emergency stop and its discovery limits, but never checks position freshness, satellite count, HDOP, or fix quality.

An operator can start a mission after GPS has gone stale, or GPS can fail during discovery, and the robot still receives forward commands. With defaults, this can continue for approximately 12 seconds at a requested 0.15 m/s while the position needed to learn heading is unavailable. Because the node continues refreshing commands, the firmware command watchdog cannot detect this failure.

**Verification:** I extracted and executed the actual `WaypointNav` class in memory with inert message publishers and a fake clock. With a fix 100 seconds old, `fix_usable()` returned `(False, 'GPS fix is 100.0 s old')`. Calling `tick()` in `DISCOVER` nevertheless published a command with `linear.x = 0.15`. No ROS runtime was started.

## 5. Medium: Finishing alignment at an intermediate waypoint crashes navigation on the next tick

**Locations:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:620`, `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:631`, `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:601`.

`advance()` clears `align_target`, then increments the waypoint index and calls `start_leg()` when another waypoint exists. It never changes `state` from `ALIGN` back to `RUNNING`. The loop-waypoints branch has the same omission. On the next tick, `tick_align()` subtracts the current heading from `None` through `geo.angle_error_deg()`.

A multi-waypoint mission therefore stops functioning after an intermediate waypoint with a facing requirement, whether alignment succeeds or times out. The exception is not caught by the normal spin loop, so navigation exits. Its `finally` block attempts to publish a stop, but mission continuation is lost; guaranteed physical stopping should not depend on delivery of that shutdown publication, especially with finding 1.

**Verification:** Executing the actual class methods in memory with two waypoints and a successfully reached alignment produced `state='ALIGN', index=1, align_target=None`. The next `tick()` raised `TypeError: unsupported operand type(s) for -: 'NoneType' and 'float'`. This reproduction used the repository's actual `geo.angle_error_deg()`.

## 6. Medium: Automatic reverse heading correction steers away from the target

**Location:** `firmware/src/control/drive_controller.h:174`.

Automatic heading control negates its differential correction whenever target speed is negative. For differential drive, compass yaw depends on the left-minus-right wheel velocity, regardless of the sign of their common velocity. Negating the correction reverses the feedback direction. The manual controller correctly avoids this reversal, as its own comment at line 316 explains.

A reverse `/cmd_move` command with a small heading error turns away from the requested heading. Error can grow until the controller switches to its spin-first branch, making reverse motion unstable even when heading and speed measurements are otherwise correct. This defect is independent of the GPS reverse-estimation error in finding 3.

**Verification:** I evaluated the wheel-mixing equations with a common output of -300 and a positive clockwise correction of 40. The implemented negation produces left=-340 and right=-260, so left-minus-right is -80: counterclockwise, opposite the requested correction. The same error requires a positive wheel difference in either travel direction.

## 7. Medium: Firmware updates are accepted while the robot rotates in place

**Location:** `firmware/src/main.cpp:2019`.

`otaIsSafe()` checks only `fabsf(estimator.speed()) < 0.05`. It does not inspect yaw rate, either wheel output, or a pending turn. A differential-drive robot rotating in place has approximately zero center-of-body translational speed while both wheels are moving.

Both push OTA and pull updates can therefore pass their stationary guard during a turn. `otaOnBegin()` then abruptly stops the motors and opens the contactor. This violates the implemented update policy of refusing an update while moving and can interrupt an active maneuver. The update lockout does remove power before flashing; the defect is acceptance during rotational motion, not absence of that lockout.

**Verification:** I traced the shared predicate into `startPullUpdate()` at line 2050 and `ota.begin()` at line 2297. For a stationary-center turn with speed=0 and nonzero yaw rate, the exact predicate returns true because yaw and wheel motion never enter the expression.

## 8. Medium: Non-finite manual input becomes a full-scale movement request

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:1038`.

The manual endpoint converts inputs with `float()` and then clamps them using `max(-1.0, min(1.0, value))`, without checking finiteness. `float('nan')` succeeds, and the clamp turns NaN into +1.0. A valid JSON request containing `{"forward":"nan","turn":0}` therefore becomes maximum configured forward input rather than a rejected request. The turn input has the same problem.

When motion is otherwise allowed, a malformed client value can unexpectedly command full configured forward speed or clockwise turn rate. This is the configured manual limit, not necessarily the robot's absolute maximum speed. Normal emergency and timeout protections still apply.

**Verification:** I extracted and executed the actual `api_manual()` method against an inert publisher, with the motion blocker returning clear and manual limits of 0.18 m/s and 45 deg/s. A NaN forward input returned HTTP status 200 and `{'ok': True, 'speed': 0.18, 'turn': 0.0}`, and published that nonzero command. A JSON string `"nan"` follows the same `float()` conversion, so the trigger does not require nonstandard JSON numeric syntax.

## Checks and review limits

- I executed the existing read-only `check_telemetry()` and `check_config_selectors()` functions from `tools/qc.py` in memory. All 64 declared telemetry fields are assigned by the firmware, and all host configuration selectors exist in the firmware. These selected checks reported zero failures.
- `node --check gps_localize_ws/src/gps_localize/web/app.js` exited successfully.
- Host reproductions executed repository methods with fake clocks and publishers. Firmware arithmetic reproductions illustrate source-level failures and are not hardware measurements.
- I did not run the four full project gates, native compilation, firmware builds, live probes, or hardware tests. Those were unnecessary for this report-only review and could create files or interact with the running system. No claim is made that the full test suite passes or that this review exhausts all possible defects.

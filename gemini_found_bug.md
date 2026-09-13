# Robot Codebase Bug Audit Report

Deliverable bug audit report. Evaluated strictly against the live working tree without applying fixes.

---

## 1. `firmware/src/control/drive_controller.h`

### Finding 1.1: Automatic reverse heading correction inverts feedback sign
- **Severity:** High
- **Location:** `firmware/src/control/drive_controller.h:174`
- **What is wrong:**
  ```cpp
  float diff = heading_.compute_with_error(heading_error_);
  if (target_speed_ < 0.0f) diff = -diff;   // steering flips when reversing
  ```
  In differential drive, body yaw rate is proportional to $(v_{\text{right}} - v_{\text{left}}) / \text{track}$. A positive heading error (target clockwise of current heading) requires clockwise torque, which always demands that the left wheel drive faster forward (or less backward) than the right wheel ($v_{\text{left}} > v_{\text{right}}$, i.e., positive `diff`), regardless of whether the common base speed is positive or negative. Inverting `diff` when `target_speed_ < 0` produces $v_{\text{left}} < v_{\text{right}}$, turning the robot counter-clockwise and converting the negative feedback loop into positive feedback. (Line 316 in the same file for manual mode explicitly and correctly states: *"heading responds to the wheel difference the same way whichever direction the robot is travelling"*).
- **How it fails on the real robot:**
  When executing reverse motion in AUTO mode, any small heading error causes the controller to steer in the wrong direction. The heading error diverges exponentially rather than converging, causing the robot to veer violently off course until error exceeds the spin threshold or the robot collides with an obstacle.

### Finding 1.2: Zero-speed auto command fails to halt active spin
- **Severity:** High
- **Location:** `firmware/src/control/drive_controller.h:146-164` (and line 64 `setAuto()`)
- **What is wrong:**
  When a large heading error triggers in-place spin (`spinning_ = true`), `spinning_` can only be cleared when `fabsf(heading_error_) < HEADING_SPIN_EXIT_DEG` (line 150) or by calling `stop()`.
  Calling `setAuto(0.0f, heading)` (which occurs when `/cmd_move` sends linear velocity `0.0` in AUTO mode, `main.cpp:612`) does not clear `spinning_`.
  In `computeTargets()`, the check `if (spinning_)` at line 155 is evaluated *before* checking `if (fabsf(target_speed_) < 1e-3f)` at line 164.
- **How it fails on the real robot:**
  If the robot is spinning to align with a heading setpoint and a stop command or zero-velocity auto command is issued, the robot ignores the zero-speed setpoint and continues driving both motors at full spin power (`±HEADING_SPIN_PWM`) until the target heading is reached.

### Finding 1.3: `DriveController::reset()` fails to clear turn-in-place state
- **Severity:** Medium
- **Location:** `firmware/src/control/drive_controller.h:41-48`
- **What is wrong:**
  `DriveController::reset()` clears `heading_`, `speed_`, `speed_ramp_`, `spinning_`, and outputs, but omits clearing `turn_in_place_`, `turn_done_`, and `turn_ok_ms_` (which are properly cleared in `stop()` at line 103).
- **How it fails on the real robot:**
  When a reset command is received over ROS (`/cmd_reset`, handled by `main.cpp:837`) while the robot is in a turn-to-heading maneuver, `turn_in_place_` remains `true`. On the next control cycle, `computeTargets()` re-enters `updateTurnInPlace()` and resumes driving the motors to complete the previous turn, violating the expectation that a reset halts all ongoing navigation actions.

---

## 2. `firmware/src/safety/safety_manager.h`

### Finding 2.1: Welded contactor fault (`STOP_CONTACTOR_STUCK`) never disables motion and reports healthy LED/state
- **Severity:** High
- **Location:** `firmware/src/safety/safety_manager.h:169-189`, lines 214, 250-256, 298-311
- **What is wrong:**
  In `SafetyManager::update()`:
  1. `f` is initialized to 0 at line 92.
  2. `allow_` is evaluated at lines 169-175:
     ```cpp
     if (f == 0) {
       if (last_clear_ms_ == 0) last_clear_ms_ = now;
       allow_ = (now - last_clear_ms_) >= ESTOP_RELEASE_HOLD_MS;
     } else {
       last_clear_ms_ = 0;
       allow_ = false;
     }
     ```
  3. The contactor cross-check runs *after* line 175, at lines 186-188:
     ```cpp
     if (in.rails_live && (now - relay_open_since_) >= CONTACTOR_SETTLE_MS) {
       f |= STOP_CONTACTOR_STUCK;
     }
     ```
  Because `f` is wiped to 0 at the start of every tick, `STOP_CONTACTOR_STUCK` is NEVER set when lines 169-175 evaluate `f == 0`. Therefore, `allow_` remains `true`, and `allowMotion()` returns `true`.
  Furthermore:
  - `runState()` (lines 250-256) checks `emergencyLatched()`, `batt_latched_`, and `linkLost()`, none of which include `STOP_CONTACTOR_STUCK`. It returns `RUN_IDLE` (or `RUN_AUTO`/`RUN_MANUAL`), not `RUN_ESTOP`.
  - `serviceLed()` (lines 298-311) does not check `STOP_CONTACTOR_STUCK`, and blinks the LED in the "healthy blip" pattern.
- **How it fails on the real robot:**
  If the main power relay contacts weld closed (e.g. from inductive arcing or contact bounce under motor load), the firmware commands the contactor open, detects that high-current motor rails remain live, and flags `STOP_CONTACTOR_STUCK`. However, `allowMotion()` continues returning `true`, so `main.cpp:1437-1440` continues driving PWM to the motor driver. The red LED blinks healthy and telemetry reports normal operating state, concealing a welded contactor hazard where emergency hardware cutoff has failed.

### Finding 2.2: Sensor failure permanently disables battery low-voltage cutoff while running
- **Severity:** High
- **Location:** `firmware/src/safety/safety_manager.h:127-133`
- **What is wrong:**
  ```cpp
  if (in.battery_valid && in.battery_v < cutoff_v_) {
    if (batt_low_since_ == 0) batt_low_since_ = now;
    if (now - batt_low_since_ > BATT_CUTOFF_HOLD_MS) batt_latched_ = true;
  } else {
    batt_low_since_ = 0;
  }
  if (batt_latched_) f |= STOP_BATTERY_LOW;
  ```
  The code assumes `battery_valid == false` only occurs when the relay or mushroom switch is intentionally open. However, if an I2C communication glitch or bus hang occurs on the INA226 power monitors while the relay is closed, `power.packValid()` returns `false` (`in.battery_valid = false`).
  In this state, `batt_low_since_` is reset to 0, neither `STOP_BATTERY_LOW` nor `STOP_BATTERY_SOFT` is asserted, and the safety manager fails open without verifying whether `relay_closed_` is true.
- **How it fails on the real robot:**
  If I2C noise or a transient bus stall causes the INA226 sensors to fail (which permanently marks them unpresent after 6 retries in `INA226.h`), the robot continues to operate with the contactor closed and motors driving. With battery validity lost, the cutoff never latches, allowing the robot to drive the LiPo battery down to complete depletion (< 2.5V/cell), permanently destroying the battery pack and creating a fire hazard.

### Finding 2.3: `beginUpdateLockout()` does not set a safety stop flag, allowing motion
- **Severity:** Medium
- **Location:** `firmware/src/safety/safety_manager.h:164-175, 241-244`
- **What is wrong:**
  `beginUpdateLockout()` sets `update_lockout_ = true` and `setRelay(false)`. However, `update_lockout_` does not assert any stop bit in `f`.
  In `SafetyManager::update()`, if other inputs are clear (agent ok, heartbeat ok, no e-stop), `f` evaluates to `0`. Consequently, `allow_` becomes `true` after `ESTOP_RELEASE_HOLD_MS`, and `allowMotion()` returns `true`.
- **How it fails on the real robot:**
  During wireless OTA firmware update, `main.cpp:1432` queries `safety.allowMotion()`. Because `allowMotion()` returns `true`, the control loop continues updating motor setpoints and calling `motor_left.spin()` and `motor_right.spin()`. If the contactor is welded or rail capacitance is discharging, the motors can unexpectedly jerk or drive during flashing.

### Finding 2.4: `safe_to_power` checks stale previous-cycle `flags_` for emergency status
- **Severity:** Medium
- **Location:** `firmware/src/safety/safety_manager.h:164`
- **What is wrong:**
  Line 164 computes:
  ```cpp
  const bool safe_to_power = !(f & STOP_STARTUP) && !emergencyLatched()
                             && !batt_latched_ && !update_lockout_;
  setRelay(safe_to_power);
  ```
  `emergencyLatched()` reads the member variable `flags_`:
  ```cpp
  bool emergencyLatched() const {
    return (flags_ & (STOP_HW_EMERGENCY | STOP_ESP_BUTTON | STOP_SW_ESTOP)) != 0;
  }
  ```
  However, `flags_` is only updated from `f` at line 191 (`flags_ = f;`). Thus, `safe_to_power` checks the flags from the *previous* 100 Hz control tick rather than the current cycle's freshly debounced hardware and software inputs in `f`.
- **How it fails on the real robot:**
  When an emergency stop button or software e-stop is triggered, the command to de-energize the relay (`setRelay(false)`) is delayed by a full control loop cycle (10 ms) rather than executing immediately on the cycle where the emergency was detected.

### Finding 2.5: `STOP_BATTERY_SOFT` lacks voltage hysteresis or recovery delay
- **Severity:** Low
- **Location:** `firmware/src/safety/safety_manager.h:139-140`
- **What is wrong:**
  `batt_soft_stop_ = in.battery_valid && in.battery_v < soft_stop_v_;`
  When pack voltage drops below `soft_stop_v_`, `STOP_BATTERY_SOFT` is asserted, setting `allow_ = false` and disabling drive power. When motor current ceases, battery internal resistance causes the terminal voltage to immediately rebound above `soft_stop_v_`. Without hysteresis or a minimum recovery hold time, `batt_soft_stop_` clears immediately on the next measurement, allowing motion to resume 400 ms later.
- **How it fails on the real robot:**
  When the battery reaches near-depletion, heavy motor acceleration causes voltage sag, tripping soft stop. As soon as motors halt, voltage springs back and the robot resumes driving, creating rapid start-stop oscillations (chatter) that repeatedly stress motor gearboxes and the battery.

---

## 3. `firmware/src/estimator/state_estimator.h`

### Finding 3.1: Unsigned GPS speed and course over ground invert body speed and heading in reverse
- **Severity:** High
- **Location:** `firmware/src/estimator/state_estimator.h:160-163, 283-286, 340-343`
- **What is wrong:**
  1. GPS ground speed reported by NMEA (VTG/RMC) is an unsigned scalar magnitude ($\ge 0$). In `update()`:
     ```cpp
     if (in.gps_ok) {
       float k = dt / ((in.imu_ok ? cfg_.gps_tau_s : cfg_.gps_tau_s * 0.3f) + dt);
       v += k * (in.gps_speed - v);
       speed_source_ = in.imu_ok ? SPEED_IMU_GPS : SPEED_GPS;
     }
     ```
     `v` represents signed body velocity. The estimator treats `in.gps_speed` as a signed forward velocity, pulling `v` towards positive values even when driving in reverse.
  2. In `update()`, when moving (`in.gps_course_ok && in.gps_ok && fabsf(speed_) > cfg_.align_min_mps`):
     ```cpp
     pullOffset(angErrDeg(in.gps_course, in.imu_yaw), cfg_.align_tau_s, dt);
     ```
     During reverse motion, course over ground is the direction of translation, which is $180^\circ$ opposite the orientation of the robot's nose. `pullOffset` forces `heading_deg_` to align directly with `in.gps_course`, falsely rotating the estimated nose heading by $180^\circ$.
  3. Odometry updates at lines 341-342 integrate `speed_ * cosf(h_enu) * dt`. With both speed sign and heading angle corrupted by reverse travel, displacement accumulates in reverse of reality.
- **How it fails on the real robot:**
  When driving in reverse with GPS lock:
  - The robot estimates positive forward speed. The manual closed-loop speed controller (`MANUAL_SPEED_PID`) calculates an excessive negative error (`target - estimated`), slamming the motors into maximum reverse saturation.
  - The heading filter twists the robot's heading by $180^\circ$, destabilizing heading hold and waypoint navigation.
  - Odometry integrates in the wrong direction, producing diverging pose errors.

### Finding 3.2: Heading reference and health flags fail to clear when all sensors fail
- **Severity:** Medium
- **Location:** `firmware/src/estimator/state_estimator.h:328-336`
- **What is wrong:**
  In `update()` lines 320-334:
  ```cpp
  if (in.imu_ok) {
    heading_deg_ = wrap360f(in.imu_yaw + align_offset_);
  } else if (in.gps_course_ok && in.gps_ok && fabsf(speed_) > cfg_.align_min_mps) {
    heading_deg_ = wrap360f(in.gps_course);
    heading_ref_ = REF_GPS;
  } else if (allow_mag && in.mag_ok) {
    heading_deg_ = wrap360f(in.mag_heading);
    heading_ref_ = REF_MAG;
  } else {
    heading_deg_ = wrap360f(heading_deg_ + compass_rate_dps * dt);
  }
  ```
  In the final `else` branch (where IMU, GPS course, and compass have all failed simultaneously), the code integrates `compass_rate_dps` (derived from `in.yaw_rate` from the dead IMU) but omits setting `heading_ref_ = REF_NONE`.
  At line 336:
  ```cpp
  if (heading_ref_ == REF_NONE) health_ |= HEALTH_NO_ABS_HEADING;
  ```
  Because `heading_ref_` retains whatever previous reference it held (`REF_MAG` or `REF_GPS`), `HEALTH_NO_ABS_HEADING` is never set.
- **How it fails on the real robot:**
  If the IMU, compass, and GPS disconnect or fail concurrently during operation, the estimator continues reporting that its heading is referenced to an absolute sensor (`REF_GPS` or `REF_MAG`) and omits `HEALTH_NO_ABS_HEADING`. Upstream planners (such as `waypoint_nav`) believe the robot maintains an absolute reference and will not trigger heading recovery or safety aborts.

---

## 4. `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py`

### Finding 4.1: Intermediate waypoint alignment crashes node with `TypeError` on completion
- **Severity:** High
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:601-640`
- **What is wrong:**
  When arriving at a waypoint with a target heading (`heading_deg`), `reached()` sets `self.state = ALIGN` and assigns `self.align_target = heading`.
  During alignment, `tick_align()` monitors angle error. When alignment completes (`abs(error) <= self.heading_tolerance`) or times out (`elapsed > self.align_timeout`), it calls `self.advance(...)`.
  Inside `advance()`:
  ```python
  def advance(self, why):
      self.align_target = None
      ...
      if self.index + 1 < len(self.waypoints):
          self.index += 1
          self.start_leg()
          return
      if self.loop_waypoints:
          self.index = 0
          self.start_leg()
          return
  ```
  `advance()` resets `self.align_target = None`, increments the index, and calls `start_leg()`. It **never resets `self.state` from `ALIGN` back to `RUNNING`**.
  On the very next control cycle, `tick()` observes `self.state == ALIGN` and calls `tick_align()`.
  Line 601 executes:
  ```python
  error = geo.angle_error_deg(self.align_target, self.heading_deg)
  ```
  In `geo.py:54`, `(target - current + 540.0) % 360.0 - 180.0` attempts subtraction with `target = None`, raising:
  `TypeError: unsupported operand type(s) for -: 'NoneType' and 'float'`.
- **How it fails on the real robot:**
  On any multi-waypoint mission containing an intermediate waypoint with a heading specified, as soon as the robot finishes rotating to face the heading at that waypoint, the node crashes on the next tick. Mission execution terminates abruptly.

### Finding 4.2: Heading discovery drives robot blind during stale or invalid GPS fixes
- **Severity:** High
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:406-408, 497-501, 509-547`
- **What is wrong:**
  In `tick()`, the `DISCOVER` state is handled at lines 406-408:
  ```python
  if self.state == DISCOVER:
      self.tick_discover()
      return
  ```
  This check occurs **before** `self.fix_usable()` is evaluated at line 437.
  Furthermore, `needs_discovery()` only verifies:
  ```python
  return (self.discover_enabled
          and self.heading_ref == 0
          and self.fix is not None)
  ```
  It does not check whether `self.fix` is fresh, or whether satellites, HDOP, and fix quality satisfy safety requirements.
  Inside `tick_discover()`, only `self.estop` and elapsed time/distance are checked; there are no checks on fix age or fix quality.
- **How it fails on the real robot:**
  If an operator starts a mission when GPS has lost lock or is stale (e.g. `self.fix` contains coordinates from 10 minutes prior), the robot transitions into `DISCOVER` and starts driving forward at 0.15 m/s. Because the fix is stale or unavailable, GPS course is never acquired, and `travelled` remains 0. The robot drives blind without position tracking for the entire 12-second duration (`discover_max_s`), unable to stop for GPS loss.

### Finding 4.3: Raw GPS fixes asynchronously clobber filtered EKF position estimate
- **Severity:** Medium
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:241-250, 272-281`
- **What is wrong:**
  `on_pose_estimate()` listens to `/pose/estimate` from `gps_odom` to navigate on the smoothed, EKF-fused position. It updates `self.fix = (pose['lat'], pose['lon'])`.
  However, `on_fix()` listens directly to the raw sensor topic `/gps/fix` and unconditionally executes:
  ```python
  self.fix = (msg.latitude, msg.longitude)
  ```
  Because both callbacks execute asynchronously on arriving ROS messages, incoming raw GPS fixes constantly overwrite `self.fix`.
- **How it fails on the real robot:**
  Raw GPS fixes suffer from multi-path and noise (jumping up to several meters while stationary). Because `on_fix` overwrites `self.fix`, `waypoint_nav` randomly alternates between the smooth EKF estimate and the erratic raw measurement, causing noisy steering jerks and unstable waypoint approach.

---

## 5. `gps_localize_ws/src/gps_localize/gps_localize/web_server.py`

### Finding 5.1: Non-finite manual API input (`NaN`) evaluates to full-scale forward speed and rotation
- **Severity:** High
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:1033-1045`
- **What is wrong:**
  In `api_manual()`:
  ```python
  forward = float(body.get('forward', 0.0))
  turn = float(body.get('turn', 0.0))
  forward = max(-1.0, min(1.0, forward))
  turn = max(-1.0, min(1.0, turn))
  ```
  In Python, `float('nan')` converts successfully without throwing `TypeError` or `ValueError`.
  Because standard IEEE 754 comparisons with `NaN` evaluate to `False`, `min(1.0, float('nan'))` returns `1.0`, and `max(-1.0, 1.0)` evaluates to `1.0`.
  Consequently, `forward` becomes `1.0`, and `turn` becomes `1.0`.
  Lines 1042-1043 then calculate:
  ```python
  cmd.linear.x = forward * self.manual_speed   # 1.0 * 0.18 = 0.18 m/s
  cmd.angular.z = turn * self.manual_turn       # 1.0 * 45.0 = 45.0 deg/s
  ```
- **How it fails on the real robot:**
  If a web client or API caller transmits a JSON payload containing `{"forward": "nan"}` or `{"turn": "nan"}`, instead of rejecting the input with HTTP 400, the server accepts it and commands the robot to drive forward at maximum manual speed (`0.18 m/s`) and full spin rate (`45 deg/s`).

### Finding 5.2: In-place turn API (`api_turn`) completely bypasses motion safety blocker
- **Severity:** High
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:1050-1094`
- **What is wrong:**
  `api_manual()` and `api_nav()` call `self.motion_blocker()` to reject motion commands if safety conditions are violated (such as hardware E-stop, software E-stop, low battery cutoff, or startup lockout).
  However, `api_turn()` omits checking `self.motion_blocker()` entirely. It constructs and publishes a movement Twist command (`linear.y = 1.0`, `angular.z = target`) directly to `/gps_localize/cmd_move`.
- **How it fails on the real robot:**
  An operator or API caller can issue `/api/turn` commands while the robot is in software E-stop or battery cutoff. The web server forwards the motion command to the firmware instead of rejecting it with an HTTP 409 error.

### Finding 5.3: Motion blocker ignores welded contactor (`STOP_CONTACTOR_STUCK`) and soft battery cutoff
- **Severity:** Medium
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:962-993`
- **What is wrong:**
  `motion_blocker()` checks for `'hardware_emergency'`, `'esp_button'`, `'software_estop'`, `'battery_low'`, `'agent_lost'`, and `'startup_lockout'`.
  It completely omits checking for `'contactor_stuck'` (`STOP_CONTACTOR_STUCK`) and `'battery_soft'` (`STOP_BATTERY_SOFT`).
- **How it fails on the real robot:**
  When a contactor welds shut, the firmware flags `STOP_CONTACTOR_STUCK`. The web server fails to recognize this as a motion blocker, returning `None` from `motion_blocker()` and permitting operators to continue queueing autonomous missions and jog commands on a dangerous, un-isolated electrical system.

### Finding 5.4: Unread web configuration settings (`web.yaml` and web UI Settings)
- **Severity:** Low
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:74-86`, `config/web.yaml:15,22,25`, `web/app.js:18,1392,3426,3598`
- **What is wrong:**
  `config/web.yaml` specifies configuration parameters:
  - `heartbeat_rate_hz: 4.0`
  - `manual_repeat_ms: 150`
  - `trail_points: 300`
  In `web_server.py` lines 74-86 (`_load_config`), none of these three settings are read or parsed.
  In the browser frontend (`web/app.js`), all three values are hardcoded:
  - `TRAIL_MAX = 300;` (line 18)
  - `setInterval(sendJog, 150);` (line 1392)
  - `setInterval(heartbeat, 250);` (line 3598)
  Even though `web.trail_points` is exposed in the Settings table in `app.js:3426`, it is never bound to runtime logic.
- **How it fails on the real robot:**
  An operator modifying `heartbeat_rate_hz`, `manual_repeat_ms`, or `trail_points` in `web.yaml` or through the web settings interface will find their changes completely ignored by both backend and frontend.

---

## 6. `firmware/src/main.cpp` (Control Callback & Safety Input Assembly)

### Finding 6.1: Safety input assembly omits `turningInPlace()`, disabling command timeout and misreporting run state
- **Severity:** High
- **Location:** `firmware/src/main.cpp:1427-1428, 1443-1444`
- **What is wrong:**
  In the 100 Hz control callback, `main.cpp` prepares the safety inputs:
  ```cpp
  si.motion_requested = fabsf(drive.targetSpeed()) > 1e-3f ||
                        fabsf(drive.targetYawRate()) > 1e-3f;   // pure rotation counts too
  safety.update(si);
  ```
  When the robot executes a turn-to-heading maneuver (`drive.setTurnTo()` or `/cmd_move` with `linear.y == 1.0`), `DriveController` sets `target_speed_ = 0` and `target_yaw_rate_ = 0`, while setting `turn_in_place_ = true` (exposed via `drive.turningInPlace()`).
  Because `turningInPlace()` is not checked, `si.motion_requested` evaluates to `false`.
  Consequently:
  1. In `safety_manager.h:107`:
     ```cpp
     if (in.motion_requested && in.cmd_age_ms > CMD_TIMEOUT_MS) f |= STOP_CMD_TIMEOUT;
     ```
     `STOP_CMD_TIMEOUT` is disabled. The command watchdog never fires during a turn-to-heading operation.
  2. At lines 1443-1444:
     ```cpp
     const bool moving = si.motion_requested;
     const RunState state = safety.runState(!drive.manual() && moving, drive.manual() && moving);
     ```
     Telemetry reports `RUN_IDLE` instead of `RUN_AUTO` while the robot is actively rotating in place.
- **How it fails on the real robot:**
  If a one-shot turn command is sent and the command sender crashes or network disconnects, or if the turn never finishes (e.g. frozen IMU or wheel stalled), the turn continues indefinitely with both motors powered. The command watchdog cannot intervene. Additionally, the web interface reports the robot as IDLE while it is actively spinning.

### Finding 6.2: Position EKF update is gated on speed-grade `gps_ok` instead of position-grade `gps_fix_ok`
- **Severity:** High
- **Location:** `firmware/src/main.cpp:1407-1414` (compared with lines 1324-1359)
- **What is wrong:**
  At lines 1337-1359, the code explicitly defines two separate GPS gates:
  - `gps_ok`: A strict velocity gate requiring $\ge 6$ satellites and $\text{HDOP} \le 1.5$.
  - `gps_fix_ok`: A position gate requiring only a valid 3D fix and $\ge 4$ satellites, noting that HDOP measures uncertainty, not absence.
  However, at line 1407, the position EKF update is gated with:
  ```cpp
  if (gps_ok && gps.fixQuality() > 0) {
    ...
    pose_ekf.updateGps(fix_east, fix_north, gps.hdop(), gps.fixQuality());
  }
  ```
  `gps_ok` is checked instead of `gps_fix_ok`.
- **How it fails on the real robot:**
  In typical outdoor environments where satellite geometry yields an HDOP of 1.6 to 2.5, or satellite counts of 4 to 5, the GPS fix is valid and accurate to within a few meters. However, `gps_ok` evaluates to `false`. The position EKF receives no GPS updates and runs exclusively on dead reckoning, causing fused position to drift unbounded even with an active GPS lock.

### Finding 6.3: Unsigned GPS ground speed magnitude passed directly to estimator
- **Severity:** Medium
- **Location:** `firmware/src/main.cpp:1371-1372`
- **What is wrong:**
  ```cpp
  est_in.gps_speed = (fabsf(gps.speedMps()) < GPS_SPEED_NOISE_FLOOR_MPS)
                          ? 0.0f : gps.speedMps();
  ```
  NMEA ground speed from `gps.speedMps()` is an unsigned scalar magnitude ($\ge 0$). `main.cpp` assigns this directly to `est_in.gps_speed` without accounting for vehicle direction (e.g. forward vs reverse wheel PWM or motor model).
- **How it fails on the real robot:**
  When driving in reverse, positive scalar speed is passed to the estimator, which pulls body velocity positive, corrupting closed-loop reverse speed regulation and reversing the heading reference.

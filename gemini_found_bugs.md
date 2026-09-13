# Gemini Bug Audit — GPS_Localize

Audit date: 2026-09-12.
Scope: Inspect the repository and system for bugs and defects without modifying or fixing them.

---

## 1. Static Verification and Quality Control Defects

### GEM-001 — NameError in QC runner when Node.js is missing
- **Location:** `tools/qc.py:1463-1465`
- **Defect:** 
  ```python
  if not shutil.which('node'):
      note('node not installed, so the web smoke test was skipped')
      return
  ```
  `tools/qc.py` does not define a `note()` function (it defines `NOTES = []`). If `node` is not installed on the host, running `python3 tools/qc.py` crashes with:
  `NameError: name 'note' is not defined`
- **Severity:** High (breaks QC execution on hosts without Node.js).

### GEM-002 — Syntax error in PEP 484 type comment
- **Location:** `tools/qc.py:426`
- **Defect:** 
  ```python
  # type: four sizes as tokens; component rules may use a couple of one-offs
  ```
  Starts with `# type:`, which PEP 484 and Python parsers interpret as a type comment. Tools like `flake8` report:
  `F723 syntax error in type comment 'four sizes as tokens; component rules may use a couple of one-offs'`
- **Severity:** Low (tooling/linter warning).

### GEM-003 — Docker image stale against interface definition
- **Location:** `tools/qc.py:1173-1222`, `firmware/extra_packages/gps_localize_msgs/msg/Telemetry.msg`
- **Defect:** `python3 tools/qc.py` fails:
  ```
  QC FAILED - 2 problem(s):
    - gps_localize:latest is newer than every workspace source (stale against Telemetry.msg)
    - fix: rebuild the image from the docker branch (git worktree add <dir> origin/docker && docker build -t gps_localize:latest <dir>)
  ```
- **Severity:** Medium (container out of sync with current ROS message interface).

### GEM-004 — pytest execution fails under default plugin discovery
- **Location:** `gps_localize_ws/src/gps_localize/test/`
- **Defect:** Running `python3 -m pytest` fails with:
  `ModuleNotFoundError: No module named '_pytest.scope'` inside `/home/mannaja/.local/lib/python3.10/site-packages/anyio/pytest_plugin.py` due to version incompatibility with system pytest (6.2.5). Tests only pass when invoked with `-p no:anyio` or through `colcon test` / `unittest`.
- **Severity:** Low (developer workflow defect).

---

## 2. Runtime Environment & Connectivity

### GEM-005 — Host IP and subnet mismatch breaks micro-ROS agent link
- **Location:** System network configuration vs `HANDOFF.md` / `firmware/config/network.h`
- **Evidence:** `python3 tools/doctor.py` reports:
  `FAIL: board is publishing (no telemetry in 12 s)`.
  The ESP32 firmware connects to Wi-Fi SSID `manny` with expected agent at `10.126.95.220`. The host currently has interface `enp0s3` at `192.168.100.74` without the hotspot interface active. Micro-ROS agent listens on `192.168.100.74:8888`, unreachable by the robot on `10.126.95.80`.
- **Severity:** High (robot communication offline).

---

## 3. Firmware Control & Safety Defects

### GEM-006 — Zero-speed automatic command does not cancel active spin
- **Location:** `firmware/src/control/drive_controller.h:146-164`, line 63 (`setAuto()`)
- **Defect:** If robot enters spin mode (`spinning_ = true`), calling `setAuto(0, heading)` does not clear `spinning_`. Because the `spinning_` branch executes before checking `target_speed_ == 0`, motor outputs remain active (`pwm = ±420`) instead of stopping.
- **Severity:** High / P1 (safety hazard; motors continue turning).

### GEM-007 — Turn-to-heading bypasses command timeout
- **Location:** `firmware/src/main.cpp:1427-1428`, `firmware/src/control/drive_controller.h:81-91`, `firmware/src/safety/safety_manager.h:107-114`
- **Defect:** `setTurnTo()` sets `target_speed_ = 0` and `target_yaw_rate_ = 0`. Safety manager `motion_requested` checks only those two targets, ignoring `turningInPlace()`. Thus `STOP_CMD_TIMEOUT` is never asserted during a turn, allowing runaway turns if heading does not converge.
- **Severity:** High / P1 (safety hazard; unmonitored rotation).

### GEM-008 — Automatic reverse heading correction negates wheel differential
- **Location:** `firmware/src/control/drive_controller.h:174`
- **Defect:** In differential drive, a clockwise yaw correction requires left wheel speed > right wheel speed regardless of forward/reverse direction. Line 174 negates `diff` when `target_speed_ < 0`, causing reverse driving to steer away from the target heading.
- **Severity:** High (destabilizes reverse navigation).

### GEM-009 — INA226 read failure permanently disables battery protection
- **Location:** `firmware/lib/ina226/INA226.h:60,79`, `firmware/src/power/power_monitor.h:79`, `firmware/src/safety/safety_manager.h:127`
- **Defect:** After 6 I2C errors, `INA226::fail()` sets `present_ = false`. There is no re-probe or recovery logic. Safety manager treats missing/invalid battery data as reason to skip hard cutoff, allowing an undervoltage battery to be drained without shutoff.
- **Severity:** High (hardware damage risk).

### GEM-010 — Ground speed magnitude treated as signed forward velocity
- **Location:** `firmware/src/main.cpp:1370`, `firmware/lib/gps/gps_nmea.h:226-239`, `firmware/src/estimator/state_estimator.h:160`
- **Defect:** NMEA ground speed is an unsigned magnitude. Passing it directly to estimator corrupts reverse velocity estimation and pulls heading 180 degrees backwards during reverse motion.
- **Severity:** Medium (corrupts dead reckoning and reverse speed loops).

### GEM-011 — Over-the-air update permitted during stationary spin
- **Location:** `firmware/src/main.cpp:2019`
- **Defect:** `otaIsSafe()` checks `fabsf(estimator.speed()) < 0.05` but does not check yaw rate or wheel PWM. An in-place turn has ~0 center linear velocity, so an OTA update can start mid-spin and abruptly cut motor contactors.
- **Severity:** Medium (inconsistent safety policy).

---

## 4. Navigation & Web API Defects

### GEM-012 — Intermediate waypoint alignment completion crashes navigation
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:620-647`
- **Defect:** Reaching heading in `tick_align()` calls `advance()`, which clears `align_target` and advances index, but does not reset state from `ALIGN` to `RUNNING`. Next tick calls `geo.angle_error_deg(heading, None)`, raising `TypeError: unsupported operand type(s) for -: 'NoneType' and 'float'`.
- **Severity:** High / P1 (navigation node crashes on multi-waypoint missions).

### GEM-013 — Non-finite manual API input produces maximum motor commands
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:1034-1045`
- **Defect:** `float('nan')` clamped by `max(-1.0, min(1.0, value))` yields `1.0` in Python. A JSON payload with `{"forward":"nan"}` commands full configured speed (`0.18 m/s`) rather than rejecting the input.
- **Severity:** High (unexpected movement on malformed input).

### GEM-014 — Heading discovery drives without verifying GPS fix
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:406-410,494-554`
- **Defect:** `DISCOVER` state runs before `fix_usable()` checks. If GPS fix is stale or lost, robot still drives forward for discovery interval without valid positioning.
- **Severity:** High / P1 (blind driving on stale GPS).

### GEM-015 — Setting coordinate datum teleports fused position without rebase
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/gps_odom.py:135-142,325-328`
- **Defect:** Changing datum origin leaves local EN coordinates in filter untouched, causing the global position to jump and subsequent valid GPS updates to be rejected as outlier innovations.
- **Severity:** Medium.

### GEM-016 — Raw GPS fixes overwrite fused pose in waypoint_nav
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:241-249,273-280`
- **Defect:** Both raw GPS and fused odometry write to `self.fix`. Because raw fixes arrive asynchronously, raw outliers bypass the EKF filter right before a navigation tick.
- **Severity:** Medium.

### GEM-017 — Watchdog restart clears software E-stop latch
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/safety_watchdog.py:59-60,85`, `bringup.launch.py`
- **Defect:** Watchdog initializes `estop_latched = False` on startup. If watchdog node crashes or respawns, it unlatches the firmware E-stop without operator confirmation.
- **Severity:** Medium / P1 (safety bypass).

### GEM-018 — Uncaught ValueError on invalid HTTP navigation arguments
- **Location:** `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:995-1018`, `waypoint_nav.py:327-330`
- **Defect:** String or non-numeric arguments to `start` or `goto` are forwarded without conversion checks, triggering uncaught exceptions in ROS callbacks.
- **Severity:** Medium (node crash via API).

---

## 5. Summary & State
- All identified defects have been documented without making any code changes.
- Reports available on disk:
  - `gpt_found_bug.md` (root)
  - `gemini_found_bugs.md` (root)
  - `_patches/gpt_audit_20260912_231621/after/gpt_found_bug.md`


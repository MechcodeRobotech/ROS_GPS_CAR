# Status of the 21 Historical Codex Audit Findings (2026-09-13)

This report provides a complete, systematic pass over all 21 defects originally identified in `gpt_found_bug_original_21_20260912.md` against the live repository state.

## Summary

- **9 Fixed**: GPT-001, GPT-002, GPT-003, GPT-005, GPT-006, GPT-008, GPT-009, GPT-010, GPT-012.
- **12 Open**: GPT-004, GPT-007, GPT-011, GPT-013, GPT-014, GPT-015, GPT-016, GPT-017, GPT-018, GPT-019, GPT-020, GPT-021.

---

## Detailed Matrix

| Finding | Severity | Description | Current Status | Code Location / Verification |
|---|---|---|---|---|
| **GPT-001** | P1 | Zero-speed automatic command does not stop an active spin | **FIXED** | `firmware/src/control/drive_controller.h:171-179`. Zero-speed command (`target_speed_ < 1e-3f`) exits spinning mode and resets heading PID. |
| **GPT-002** | P1 | Turn-to-heading bypasses command timeout and has no maximum duration | **FIXED** | `firmware/src/control/drive_controller.h:324-339`. Bounded by `TURN_IN_PLACE_MAX_MS` (20 s), setting `turn_timed_out_` and zeroing PWM. |
| **GPT-003** | P1 | Completing alignment at an intermediate waypoint crashes navigation | **FIXED** | `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:670-674`. Transitions state back to `RUNNING` on `self.start_leg()`. |
| **GPT-004** | P1 | Dead-reckoning navigation accepts a frozen position indefinitely | **OPEN** | `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:697`. `fix_usable()` unconditionally returns `(True, '')` when `allow_dead_reckoning` and `dead_reckon` is present, without reception freshness check. |
| **GPT-005** | P1 | Heading discovery drives before checking GPS usability | **FIXED** | `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:522`. `needs_discovery()` requires `self.fix_usable()[0]`. |
| **GPT-006** | P1 | Non-finite manual input becomes a full-scale drive command | **FIXED** | `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:1067`. Explicit `math.isfinite()` check rejects non-finite `forward` and `turn`. |
| **GPT-007** | P1 | Configuration API persists values that crash consuming nodes | **OPEN** | `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:875-891`. `api_config_set` validates only top-level dictionary type, persisting invalid nested scalar values that raise unhandled exceptions on node reload. |
| **GPT-008** | P1 | Watchdog restart clears the software emergency-stop latch | **FIXED** | `gps_localize_ws/src/gps_localize/gps_localize/safety_watchdog.py:71-75`. Latched state is persisted to disk in `estop_latched` and survives node respawn. |
| **GPT-009** | P1 | Automatic reverse heading correction has the wrong sign | **FIXED** | `firmware/src/control/drive_controller.h:220-228`. Steering difference is not negated in reverse; heading PID feedback remains correct in both directions. |
| **GPT-010** | P2 | GPS ground-speed magnitude is treated as signed body velocity | **FIXED** | `firmware/src/main.cpp:1347,1390`. Reversing sets `gps_speed_signed = -gps.speedMps()`, and `course_ok` is gated false during reverse travel. |
| **GPT-011** | P2 | Changing the datum moves the fused global position | **OPEN** | `gps_localize_ws/src/gps_localize/gps_localize/gps_odom.py:325-328`. `set_datum` sets new coordinate origin without transforming or resetting existing fused EN state. |
| **GPT-012** | P2 | Raw fixes overwrite fused positions; pose refresh extends GPS freshness | **FIXED** | `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:259-260`. `on_fix` will not overwrite `self.fix` with raw coordinates while fused poses are fresh (`now - self.fused_time <= self.fix_timeout`). |
| **GPT-013** | P2 | Invalid navigation indices crash the ROS callback | **OPEN** | `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:343,384`. `int(command.get('index'))` has no error handling and raises uncaught `ValueError`/`TypeError` on malformed client payload. |
| **GPT-014** | P2 | Disabling fix requirement allows missing position into navigation | **OPEN** | `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:694`. When `require_fix=False` and `self.fix is None`, `fix_usable()` returns True, causing `lat, lon = self.fix` in `tick()` to raise `TypeError`. |
| **GPT-015** | P2 | Paused map-picked destination cannot resume with an empty saved list | **OPEN** | `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:377`. `resume` requires `self.waypoints` to be nonempty, ignoring active `temp_target`. |
| **GPT-016** | P2 | Runtime rate settings reload without changing existing timers | **OPEN** | `waypoint_nav.py:150` & `safety_watchdog.py:81`. Config reload modifies rate properties but does not re-create or alter period of ROS timer objects. |
| **GPT-017** | P2 | Parameter persistence always reports success, including failed NVS writes | **OPEN** | `firmware/src/params/param_registry.h:196-212`. `store()` ignores return values from `putFloat` and `remove`, unconditionally returning `true`. |
| **GPT-018** | P2 | Updater can report success after restart failure or partial stack failure | **OPEN** | `tools/self_update.py:171-182`. If `restart()` fails, update continues and reports success if the existing old web process still responds to `/api/state`. |
| **GPT-019** | P2 | Patch rollback scripts cannot undo newly created files | **OPEN** | `tools/make_patch.py:94`. `rollback.sh` executes `cp "before/<file>" ...` with `set -e`, aborting immediately on new files missing from `before/`. |
| **GPT-020** | P2 | HTTP body reads and connection threads are unbounded | **OPEN** | `gps_localize_ws/src/gps_localize/gps_localize/httpd.py:256,265`. Negative `Content-Length` bypasses max check and reads until EOF; `_Server` lacks socket timeout and request concurrency limit. |
| **GPT-021** | P2 | Local Docker image is stale against workspace sources | **OPEN** | `tools/qc.py:1173`. `gps_localize:latest` is stale against `web_server.py`. Rebuild is deferred pending branch push authorization. |

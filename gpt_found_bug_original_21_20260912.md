> Historical audit recovered on 2026-09-13 from the original patch snapshot. These findings describe the 2026-09-12 pre-fix tree; subsequent fixes have not been re-audited in this recovered copy. The current gpt_found_bug.md is preserved.

# GPT bug audit — GPS_Localize

Audit date: 2026-09-12. Requested scope: inspect the project and record problems without fixing them.

No application, firmware, configuration, or existing test source was changed. No robot commands, firmware uploads, service restarts, dependency installations, commits, or pushes were performed. The audited working tree already contained extensive uncommitted changes; findings refer to that tree, not just HEAD.

This report records 21 findings. It is not a guarantee that every defect has been found. P1 means a safety/control failure or a normal-use crash that should be addressed first; P2 means a functional, robustness, or verification defect. “Reproduced” means an offline execution of the relevant production methods or headers, using fake clocks/messages/publishers. “Code trace” means the execution path was inspected but not exercised on hardware.

## Checks performed

| Check | Result |
| --- | --- |
| `python3 tools/qc.py` | FAIL: stale `gps_localize:latest` Docker image. The script prints two problem lines for this single issue. |
| `python3 tools/port_qc.py --static-only .` | PASS: 11 checks. |
| `cd firmware && pio test -e native` | PASS: 254/254 cases. |
| `cd firmware && pio run -e wifi` | PASS: RAM 108116/327680 bytes; flash 1060737/1966080 bytes. |
| `python3 -m unittest discover -s gps_localize_ws/src/gps_localize/test` | PASS: 46 tests. |
| Offline audit probes | Reproduced the navigation, input-validation, datum, and control defects described below. |

Full outputs and probe sources are in [gpt_found_bug_evidence](gpt_found_bug_evidence/). The Python probe loads production class definitions through AST, with ROS replaced by simple local objects; it never creates a ROS node. The C++ probe links the project's mock Arduino implementation and actual controller/safety headers. Hardware timing, physical stops, actual NVS failures, browser concurrency, and full live ROS integration were not tested. Existing passing tests do not cover the failing paths below. In particular, `test_arrival.py:85` mirrors the dwell branch instead of executing the production navigation state machine.

## P1 findings

### GPT-001 — A zero-speed automatic command does not stop an active spin

**Location:** `firmware/src/control/drive_controller.h:146-164`, `setAuto()` at line 63.

**Evidence: Reproduced.** Start automatic motion with a large heading error, then call `setAuto(0, same_heading)`. The `spinning_` branch returns before the zero-speed branch, and `setAuto()` does not clear `spinning_`. After another second the probe reports `pwm=420,-420 spinning=1`.

**Impact:** A zero-speed `/cmd_move` can keep the wheels rotating. Since `target_speed_` is now zero, the main-loop motion-request test can also stop applying the command timeout. An explicit `drive.stop()` or a different safety condition stops it; merely asking for zero speed is insufficient. This can occur when a navigation stop uses a stale heading that remains outside the spin-exit tolerance.

### GPT-002 — Turn-to-heading bypasses command timeout and reports no requested motion

**Location:** `firmware/src/main.cpp:1427-1428`; `firmware/src/control/drive_controller.h:81-91`; `firmware/src/safety/safety_manager.h:112-114`.

**Evidence: Reproduced.** `setTurnTo()` sets both target speed and target yaw rate to zero. `motion_requested` tests only those values, omitting `turningInPlace()`. With a 10-second-old command and a healthy heartbeat, the probe reports `motion_requested=0 allowMotion=1 cmd_timeout_flag=0`.

**Impact:** An abandoned turn can continue indefinitely if its heading never converges while another browser keeps heartbeats alive. Run-state reporting also classifies this rotation as idle. `/api/turn` sends only one command, so changing this requires an explicit decision about the intended supervised turn lifecycle.

### GPT-003 — Completing alignment at an intermediate waypoint crashes navigation

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:620-647` (`advance`), `tick_align()`.

**Evidence: Reproduced.** With two waypoints and state `ALIGN`, reaching the first heading calls `advance()`, clears `align_target`, increments the index, and starts the next leg without setting state `RUNNING`. Observed state: `ALIGN index=1 align_target=None`. The next `tick()` raises `TypeError: unsupported operand type(s) for -: 'NoneType' and 'float'`.

**Impact:** A normal mission with an intermediate facing requirement crashes. Looping after the final aligned waypoint has the same missing state transition. The launch file respawns the node, losing mission progress.

### GPT-004 — Dead-reckoning navigation accepts a frozen position indefinitely

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:251-295,652-658`; `gps_odom.py:285-318`.

**Evidence: Reproduced.** `fix_usable()` immediately returns true whenever dead reckoning is enabled and `dead_reckon` is non-null. A pose 99 seconds old still returns `(True, '')`. There is no timestamp for dead-reckoning message reception. Moreover, `gps_odom` republishes its last `odom_xy` without checking telemetry freshness.

**Impact:** If the pose publisher or telemetry path stops while the command and heartbeat paths continue, navigation keeps issuing motion toward a target using a frozen location. The shipped `nav.yaml` enables this fallback.

### GPT-005 — Heading discovery drives before checking GPS usability

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:406-410,494-554`.

**Evidence: Reproduced.** `tick()` dispatches `DISCOVER` before the normal fix gate. `needs_discovery()` only requires a non-null fix and unknown heading. With `fix_usable()` returning false, the actual discovery method still publishes `linear.x=0.15`.

**Impact:** Starting with an old fix, poor satellite quality, or losing GPS during discovery causes up to the configured discovery interval of blind driving. On discovery failure, it proceeds to normal navigation even though an absolute heading was never established. GPS quality checks in normal navigation do not protect the discovery phase.

### GPT-006 — Non-finite manual input becomes a full-scale drive command

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:1034-1045`; related missing finite checks in `api_turn()` and firmware callbacks at `firmware/src/main.cpp:600-638`.

**Evidence: Reproduced.** Calling the production API method with `{"forward":"NaN","turn":0}` returns HTTP status 200 and publishes speed `0.18`, the configured full forward speed. Python's `min(1.0, nan)` returns its first argument, so clamping silently converts invalid input to 1.0.

**Impact:** A malformed command becomes motion rather than a validation error. String values make this reproducible using valid JSON. Turn inputs also accept non-finite values and firmware does not reject them before controller math; their physical output was not tested.

### GPT-007 — Configuration API persists values that crash consuming nodes

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:864-880`; `waypoint_nav.py:155-199`; `safety_watchdog.py:93-104`.

**Evidence: Reproduced using a temporary runtime directory.** Saving `{"group":"nav","data":{"nav":{"cruise_speed_mps":"bad"}}}` returns 200. Running `_load_config()` on the saved file raises `ValueError: could not convert string to float: 'bad'`. Only the outer dictionary is validated. Nested nulls, lists, and invalid scalar types have similar unguarded consumers.

**Impact:** A settings save can terminate navigation or the safety watchdog. The invalid file remains on disk, so automatic respawn repeats the failure. No live configuration was altered during reproduction.

### GPT-008 — Watchdog restart clears the software emergency-stop latch

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/safety_watchdog.py:59-60,85,135-145`; `gps_localize_ws/src/gps_localize/launch/bringup.launch.py` watchdog `respawn=True`; `firmware/src/main.cpp:646-650`.

**Evidence: Code trace.** Each watchdog instance initializes `estop_latched=False` and immediately publishes it. The firmware directly assigns the received boolean to `sw_estop`. No persisted latch or explicit-reset handshake survives a watchdog restart.

**Impact:** A crash, respawn, or service restart clears an operator's software e-stop without pressing RESET. This removes the inhibit; it does not by itself create a new drive command. Continuing command producers can then regain control. Physical emergency inputs remain separate.

### GPT-009 — Automatic reverse heading correction has the wrong sign

**Location:** `firmware/src/control/drive_controller.h:174`.

**Evidence: Reproduced.** For heading 0 degrees, target 20 degrees and speed -0.25 m/s, the controller produces left=-1023, right=-653. Clockwise correction requires left greater than right, including when both wheels run backwards. The code negates the differential only in automatic reverse; the manual controller's own comments correctly state that wheel-difference heading response does not change sign in reverse.

**Impact:** Negative-speed automatic commands steer away from the requested heading. This path is exposed through `/cmd_move`, even though ordinary waypoint missions use positive speed.

## P2 findings

### GPT-010 — GPS ground-speed magnitude is treated as signed body velocity

**Location:** `firmware/lib/gps/gps_nmea.h:226-239`; `firmware/src/main.cpp:1371-1374`; `firmware/src/estimator/state_estimator.h:160-164,211-212,284`.

**Evidence: Code trace.** NMEA speed is passed to the estimator as a positive magnitude without projecting course onto robot heading or assigning a reverse sign. GPS course can also be used directly as the body heading reference.

**Impact:** Reversing with usable GPS pulls the reported body velocity toward a positive value, opposing reverse motion in the speed loop and moving odometry forward along body heading. When GPS course supplies heading, backwards travel can instead rotate the body-heading estimate by 180 degrees. This requires a GPS-backed reverse test to quantify; no physical reverse test was run.

### GPT-011 — Changing the datum moves the fused global position

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/gps_odom.py:135-142,325-328`.

**Evidence: Reproduced.** With fused EN=(0,0) at datum (10,10), calling `set_datum(11,11)` leaves EN unchanged. The next conversion places the same estimate at (11,11). The old filter coordinates and covariance are neither rebased nor reset.

**Impact:** Changing only the coordinate origin teleports the displayed/navigation position. Subsequent legitimate GPS fixes can be rejected as large innovations until the rejection limit is reached.

### GPT-012 — Raw fixes overwrite fused positions; pose refresh extends GPS freshness

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:241-249,273-280,660-664`; `gps_odom.py:109,289`.

**Evidence: Code trace.** Both callbacks write `self.fix`: every raw fix replaces the fused position, while fused poses arrive on a 0.5-second timer. Navigation runs at 5 Hz. A raw outlier can therefore reach a navigation tick even if fusion rejects it. Additionally, every fused pose refreshes `fix_time`, and the navigation freshness gate uses that time instead of `gps_fix_time`.

**Impact:** Fusion is not reliably preferred, and with telemetry quality still latched good, the 3-second freshness budget can approach twice its intended duration: gps_odom publishes until its own timeout, followed by waypoint_nav's timeout after the last pose. This is distinct from the unlimited dead-reckoning freshness failure.

### GPT-013 — Invalid navigation indices crash the ROS callback

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/web_server.py:995-1018`; `waypoint_nav.py:327-330,367-372`; also `gps_odom.py:142` for unvalidated datum numbers.

**Evidence: Reproduced.** `on_cmd({"action":"goto","index":"bad"})` raises an uncaught `ValueError`. The HTTP route validates action and map coordinates but forwards `start`/`goto` indices without numeric validation. `set_datum` similarly forwards strings into an uncaught conversion in gps_odom.

**Impact:** Malformed API commands terminate ROS nodes instead of returning a client error.

### GPT-014 — Disabling the fix requirement allows a missing position into navigation

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:652-654,449-450`.

**Evidence: Reproduced.** Set `require_fix=False`, `fix=None`, and enter `RUNNING`. `fix_usable()` returns true and `tick()` raises `TypeError: cannot unpack non-iterable NoneType object`.

**Impact:** The setting intended to bypass GPS restrictions crashes navigation before a position exists. Position existence and quality-policy bypass are conflated.

### GPT-015 — A paused map-picked destination cannot resume with an empty saved list

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:358-365`.

**Evidence: Code trace.** `pause` preserves `temp_target`, but `resume` requires `self.waypoints` to be nonempty. Map-picked navigation explicitly does not require or add to that list.

**Impact:** A valid paused one-off mission silently ignores Resume when no saved waypoints exist.

### GPT-016 — Runtime rate settings reload without changing existing timers

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/waypoint_nav.py:150,223-231`; `safety_watchdog.py:81-83,106-114`.

**Evidence: Code trace.** Configuration reload updates `command_rate` and `heartbeat_rate`, but timer objects are created only during initialization and never rescheduled.

**Impact:** Saved command/heartbeat rate settings appear accepted but do not affect execution until node restart. Other reloaded settings change immediately, making this inconsistent and hard to diagnose.

### GPT-017 — Parameter persistence always reports success, including failed NVS writes

**Location:** `firmware/src/params/param_registry.h:196-212`.

**Evidence: Code trace.** `store()` ignores the results of `putFloat()` and `remove()` and always returns true. Therefore the callers' `PARAM_STORE_FAILED` path is unreachable for actual storage errors.

**Impact:** A changed value can appear saved but revert after reboot if NVS is full, unavailable, or fails a write. Actual storage fault injection was not performed.

### GPT-018 — Updater can report success after restart failure or partial stack failure

**Location:** `tools/self_update.py:72-81,173-189`.

**Evidence: Code trace.** Failed `restart()` only appends a log message. If the old web server still answers `/api/state`, `stack_answers()` returns true and `apply()` returns success. It checks neither returned content nor navigation/watchdog health.

**Impact:** Users can receive a successful update result while the old processes still run, or while only the web node survives and navigation/safety nodes fail. The updater was not executed.

### GPT-019 — Patch rollback scripts cannot undo newly created files

**Location:** `tools/make_patch.py:44-51,97-107`.

**Evidence: Code trace.** The snapshot skips paths that do not yet exist. Rollback unconditionally copies a matching `before/` file for every path, including new files with no before-image. Shell rollback uses `set -e`.

**Impact:** Rolling back a patch that introduced a file fails at the missing snapshot, leaves that new file in place, and may skip restoring later files. The required backup mechanism is incomplete for new-file patches. No existing rollback was run.

### GPT-020 — HTTP body reads and connection threads are unbounded

**Location:** `gps_localize_ws/src/gps_localize/gps_localize/httpd.py:257-272,390-398`.

**Evidence: Code trace.** Content-Length is capped only on its upper bound. A negative value reaches `rfile.read(-1)`, which waits for EOF; a declared positive length can also be sent partially. The server sets no connection read timeout and inherits unrestricted thread creation from `ThreadingHTTPServer`. The SSE client cap does not cover these request threads.

**Impact:** Incomplete requests can retain threads and sockets indefinitely, exhausting a host serving robot control. No exhaustion test was run against the live server.

### GPT-021 — Local Docker image is stale against the interface sources

**Location:** local `gps_localize:latest`; `tools/qc.py:1173-1222`.

**Evidence: Reproduced by the required QC gate.** Exact final output:

```text
QC FAILED - 2 problem(s):
  - gps_localize:latest is newer than every workspace source (stale against Telemetry.msg)
  - fix: rebuild the image from the docker branch (git worktree add <dir> origin/docker && docker build -t gps_localize:latest <dir>)
```

The first line uses a positive assertion as its failure label; “stale against Telemetry.msg” is the failure condition. This was already documented in HANDOFF before the audit.

**Impact:** The existing container image cannot be treated as matching the current message interfaces. The image was not rebuilt or started, so this audit does not claim a specific container runtime exception.

## Review coverage and limitations

The review followed control-command and safety paths through firmware main, controller, safety manager, estimator, GPS parser, parameter registry and OTA handling; navigation, GPS fusion, watchdog, web API, shared HTTP/configuration code; viewer route parity and UI command dispatch; launch files, updater, patch tooling, and the QC/test infrastructure. Drivers and networking helpers received targeted inspection, not exhaustive fault injection. Archived `_patches`, staged `_work`, vendored dependencies, generated build/install trees, Windows executable internals, and unrelated historical robot code were not treated as current application source.

Additional inspection candidates, not counted as established findings: repeated alignment commands reset the firmware turn-settle timer; HTTP mutation endpoints have no visible authentication/origin policy; multi-client command ownership and manual/autonomous command arbitration need an explicit intended-behavior review. Their end-to-end consequences were not established in this audit.

No fixes were applied. Any correction, hardware validation, image rebuild, or deployment remains separate future work requiring the user's instruction.

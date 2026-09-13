# HANDOFF — live state of the work

Read this first, then `CLAUDE.md`. This file is how work moves between Claude
Code, Codex and Gemini/Antigravity when one of them runs out of allowance: every
agent reads it before starting and updates it before stopping, so the next one
does not have to re-derive anything. Keep it short and current. History belongs
in `tools/plan_state.json` (rendered as `plan.html`), not here.

Last updated: 2026-09-13 01:10 Asia/Bangkok, Antigravity relay. Latest measurements and remaining blockers are recorded below; the older hardware table is historical.

## Hardware, right now

Checked 2026-09-13 00:50 over Wi-Fi and over the USB console.

| | |
|---|---|
| Power | **Battery CONNECTED, 12.50 V, contactor closed.** The motors are live. Nothing may command motion unattended - see the rules below - and anyone testing should have the wheels off the ground or the e-stop in hand |
| Board | ESP32 on SSID `manny` at **192.168.137.213**, RSSI -52 dBm, also on USB as `/dev/ttyUSB0` |
| Network | **The PC has moved to WIRED ethernet, 192.168.100.74, and has no wireless hardware at all** (`wlp132s0f0` is gone). The board is on a different subnet and reaches the agent outbound through the hotspot's NAT, which works but is one-way: the PC cannot ping 192.168.137.213. Sessions are being rebuilt every minute or so from 192.168.100.28, the NAT'd source address |
| Link | Healthy: `/gps_localize_firmware` in the graph, telemetry age 0.0 s, loop_us 3877, no stop reasons, sensors_missing none |
| GPS | 6 satellites, HDOP 2.26, fix quality 1, 61939 bytes parsed. Reported as a WORKING sensor at that HDOP, which is T198 behaving correctly |
| Firmware | **v1.61 - today's 16 firmware fixes are NOT on it.** The built image is waiting |

**The USB console is NOT broken, contrary to what earlier notes said.** At 115200
it reads 100% clean ASCII. The trick is the reset and the read style: drop DTR,
raise then drop RTS, then read with raw `read(4096)` in a loop rather than
`readline()`. Probed against 74880, 230400, 460800, 921600 and others - all of
those return garbage, 115200 returns the boot log. It gives `info`, `wifi` and
`agent` on demand and is the fastest way to see why the board is not joining
something.

One thing to watch: `imu reports` showed `game 4.3/100.0 Hz` at boot and
telemetry then read `imu_hz_game 0.0`. Earlier sessions measured 38-48 Hz. The
magnetometer-referenced report `rv` being near zero is expected on this robot
(one fusion, see CLAUDE.md), but the game rotation vector being near zero is
not. Check it before trusting the heading.

## Real-time baseline to protect

**The recorded baseline is NOT comparable with the robot as it stands, and it
must not be read as a regression.** It was taken in materially different
conditions, and nobody should chase a number that was measured against a
different machine state.

| when | conditions | loop_us median / p99 / max | late publishes |
|---|---|---|---|
| baseline, after T186 | USB power, indoors, **no GPS fix**, no battery | 3233 / 5407 / 5617 | 0 of 900 |
| 2026-09-13 01:16, firmware 1.61 | battery, GPS fix, NAT'd link | 5112 / 10591 / 14032 | 1 of 884 |
| 2026-09-13 01:21, firmware 1.62 | same as above | 7494 / 12003 / 100788 | 5 of 818 |

What differs from the baseline: the GPS now has a fix and is parsing NMEA every
cycle, the battery is connected and the motor rails are live, and the Wi-Fi has
fallen to -61 dBm through a NAT. The rise was already there BEFORE the 1.62
flash, so it is not attributable to those changes.

Sampled directly from `/api/state`, which is immune to frame loss, loop_us runs
**5.4 to 10.9 ms against a 10 ms period**. That is tight, and worth settling.

The 1.62 row is additionally contaminated and should not be quoted: the probe
measures gaps between the firmware's own `/imu/data` stamps, and a frame the
NETWORK dropped is indistinguishable from a late one. During that run a
BEST_EFFORT subscriber counted odom at 6.5 Hz against 10, IMU at 13.2 against
20 - roughly a third of frames lost - so both the 100 ms maximum and the 5 late
publishes are most likely lost frames rather than a stalled loop.

**To settle it properly**, reproduce the baseline conditions rather than
comparing across them: cover the GPS antenna so there is no fix, and measure on
the same power and the same link, once before and once after a flash. Anything
else compares two different machines.

## Agents

| agent | how | state at last check |
|---|---|---|
| Claude Code | `claude -p` (wrapper in `~/.local/bin/claude`) | works; weekly allowance ~91% used |
| Codex | `codex exec` (wrapper in `~/.local/bin/codex`, model gpt-6-astra) | usage limit hit, resets 04:32 on 2026-09-12 |
| Antigravity | `~/.gemini/bin/agy -p` (not on PATH), model `gemini-3.8-flash-high` | works - smoke test returned RELAY_OK, exit 0 |

`tools/agent_relay.sh --dry-run` shows the live picture. Default order is
`claude codex agy`.

## Rules for an agent running unattended

- **Never command the robot to move** - no `/cmd_move`, `/cmd_vel`, `/api/nav`
  start or goto, manual drive or turn. Motion needs the user present. Flashing
  firmware is allowed.
- Never `git push`, publish, or run `tools/publish_branches.sh`. Commit only if
  the user has asked for it.
- Never install system packages, change the PC's Wi-Fi, or touch the hotspot.
- Chat output is caveman style (CLAUDE.md). File contents are normal English.
- Run the gates in CLAUDE.md before claiming anything works. Record results here
  as numbers with the command that produced them.
- Before stopping for any reason - finished, stuck, or near your own usage
  limit - update the Queue: what you did, what you measured, the exact next step.

## Active work — illustrated setup/manual, T213 and T214

Latest user request: finish the no-Docker guide and expand into step-by-step
setup/run documentation with screenshots of every UI screen, including where
Wi-Fi settings actually live and what/where/when/why/how explanations. Maintain
a Gemini handoff across limits. This task is the current priority over the
historical queue and completed Docker release below.

- T213 native guide and T214 illustrated guides are complete in `_work/native_guide/`.
  New docs: `docs/native.md`, `docs/step_by_step.md`, `docs/wifi_setup.md`,
  `docs/screenshots/README.md`; 33 PNGs and two provenance manifests.
  All 10 UI tabs, 7 Settings groups, 9 board groups, row editor/topic watch,
  three read-only native checks and two source Wi-Fi references are captured.
  Browser capture: no exceptions; only fixture `/api/heartbeat` POSTs.
  No live USB, configuration, motion, flash or service restart occurred.
- Corrected stale claims that the browser edits Wi-Fi credentials or has a
  Firmware section. Wi-Fi credentials use the wifi firmware's 115200 USB
  console; On the robot → network only changes transmit power. Console SSIDs
  cannot contain spaces. Source: `firmware/src/net/settings_console.h`.
- Staged QC and static port checks pass. Ten changed guides have valid local
  file links and 67 syntactically valid Bash blocks. Two capture scripts compile.
  Terminal checks all exit 0. Earlier host native 261/261 and Wi-Fi build pass;
  final promotion gates will be recorded below. Evidence:
  `_work/native_guide_evidence/`, `_work/screenshot_guide_evidence/`.
- Capture dependencies already installed locally. Browser reproduction:
  `PYTHONPATH=_work/screenshot_guide_evidence/python PLAYWRIGHT_BROWSERS_PATH=$PWD/_work/screenshot_guide_evidence/browsers python3 _work/native_guide/tools/capture_guide.py --root . --out _work/native_guide/docs/screenshots`.
  Terminal: `python3 _work/native_guide/tools/capture_terminal_guide.py --root . --out _work/native_guide/docs/screenshots`.
  GTK offscreen drawable warning is harmless; inspected images saved correctly.
- Remaining: destination Windows OS/install/USB screenshots and physical
  acceptance; fresh Ubuntu installation recording; attended board setup captures.
  Exact checklist is `docs/screenshots/README.md` in staging. Do not fabricate
  these or mark them passed from sample data. T212 remains Windows HW-BLOCKED.
- Preserve unrelated untracked `pio_test_output.txt`; never stage/remove it.

Final local release checks (2026-09-13 20:09 Asia/Bangkok):
- QC PASS; static port 11/11 PASS; native 261/261 PASS in 1.190 s;
  Wi-Fi build PASS in 10.909 s. Logs `final-*.log` in the evidence folder.
- Existing committed audit evidence broke portability QC: made its standalone
  Python ROOT relative, normalized personal home paths in historical logs to
  `${USER_HOME}`, and documented that normalization. No recorded results changed.
  Terminal JSON manifest likewise normalizes home paths; PNGs retain actual output.
- Existing image was stale after intervening source commits. Rebuilt
  `gps_localize:latest`, image ID
  `sha256:315c4cd9634ca9f9ebe13bf2f663be6826eea1dcffce5486b43d320f90f335e0`.
  Network-none smoke container starts five host nodes, returns valid `/api/state`,
  and contains no private firmware header. Launch defaults to domain 10 despite
  environment override; network namespace was isolated with `--network none`.
  Smoke container stopped and removed. See docker-rebuild.log/docker-smoke.log.
- Promotion snapshots: `_patches/illustrated_guides_20260913_200616/` and
  `_patches/illustrated_guides_20260913_200733/`, each with binary diffs and
  working rollback. Reverse order to roll back both. First contains 48 files;
  second contains 7 portability/capture fixes.
- T213 DONE. T214 HW-BLOCKED only for unavailable actual OS/install/physical
  captures; all locally available guide/capture work is delivered. T212 remains
  Windows physical acceptance. `docs/screenshots/README.md` is the exact checklist.

Git delivery completed: documentation commit `c917a82` and generated snapshots
`workspace 49fa2d6`, `firmware 8f4ec0d`, `docker e63ed73` were pushed atomically
to origin. Generation used `_work/native_publish` with a clone-local Git identity;
no global settings or unrelated user files changed. This checkpoint will also be
committed and synchronized; use `git log -1`/remote refs for the newest metadata
commit rather than treating the recorded documentation snapshot as an immutable tip.

Gemini's next task is destination-machine capture/acceptance from
`docs/screenshots/README.md`, not recreating completed UI screenshots. Start with
T212 Windows USB/transport checks and T214 OS/installation/attended screenshots.
A destination Windows session and an operator for physical robot steps are still
required. Preserve `pio_test_output.txt`. No Gemini process has been launched;
the explicit relay command is in `docs/step_by_step.md`.

## Current priority — Docker and Windows release (2026-09-13)

The user requested complete installation/operation documentation, Docker
verification, Windows USB into Docker, Git delivery of the current project,
and a Gemini continuation handoff on limits. This task supersedes the older
Docker rebuild and commit deferrals below. No motion or hardware upload was
performed, and the real robot stack was not restarted.

Task T211. Staging: `_work/docker_windows_release/`; final evidence:
`_work/docker_release_evidence/`. Promotion snapshot:
`_patches/docker_windows_release_20260913_1650/`, including before/after, diff,
and a rollback.sh that also removes newly added files.

Completed locally:
- Docker generator now supports `--render-docker DIR` before committing and
  `--local` for generating deployment branches without pushing.
- Windows published-port configuration and separate selectable USB device and
  serial-runtime overrides. Runtime serial uses /dev/ttyUSB0 at 115200; USB
  flashing alone does not change the firmware transport.
- Windows USB guide uses usbipd-win + WSL2 Ubuntu + Docker Engine in that same
  distribution. Desktop Wi-Fi is a separate route. See docs/docker.md and
  docs/windows_handoff.md for exact installation and acceptance commands.
- Docker context and derived branches exclude network_secrets.h. Persistent
  volumes retain runtime data and both PlatformIO caches. Updated README,
  documentation index, native installation, viewer setup, operations, hardware,
  safety, architecture, and battery defaults. Corrected the existing battery
  tier test to current defaults; no threshold values were changed.
- Image build PASS, final image digest
  sha256:747eaa213a2ec00f42eef25a580a09d1084297a530cc6cf9f82ee8b860133f53.
- Isolated domain 91 server: HTTP page/assets/API PASS, five ROS host nodes,
  UDP agent startup, synthetic named telemetry reaches API, secrets absent,
  runtime data survives down/up. A separate network-none domain 92 serial agent
  opens a pseudo-terminal. Neither test establishes physical USB connectivity.
- Four required gates: `python3 tools/qc.py` PASS;
  `python3 tools/port_qc.py --static-only .` PASS 11/11;
  `cd firmware && pio test -e native` PASS 261/261 (1.255 s);
  `cd firmware && pio run -e wifi` PASS (15.737 s).
  Additional `pio run -e serial` PASS (114.739 s).
- Local Markdown link sweep found zero broken targets. Credential-pattern scan
  found no recognized private keys/tokens in Git candidates. Existing audit
  reports remain historical/open findings; this is not an all-bugs-fixed claim.

Git delivery completed: source snapshot `952257d` and generated branches
`workspace` (`21f6fee`), `firmware` (`02882b1`), and `docker` (`0c0db52`) were
atomically pushed to the existing origin, MechcodeRobotech/ROS_GPS_CAR. The
following documentation-only status commit records completion; use `git log -1`
and `git ls-remote origin` for the latest branch tips. All generated Docker
files were compared byte-for-byte with the tested build context; all derived
branches contain the handoff/docs and exclude network_secrets.h.

T211 software/documentation/Git work is complete. T212 Windows hardware acceptance
remains blocked on access to a Windows PC/USB adapter: follow
`docs/windows_handoff.md`, record actual versions and telemetry/reconnect results.
Do not mark those tests passed from Linux. The historical robot queue remains.

Gemini fallback is available: `RELAY_ORDER=agy tools/agent_relay.sh --dry-run`
confirmed `/home/mannaja/.gemini/bin/agy` ready. The isolated smoke container has been stopped; its test volumes remain local.
No fallback worker has been
started; interactive Codex is not automatically watched by the relay. If Codex
hits its limit, run a single fallback with a focused task prompt:

```bash
tools/agent_relay.sh --prompt "Read HANDOFF.md and CLAUDE.md. Review completed T211 and continue Windows acceptance T212 only if Windows hardware is available. Preserve the existing staging and evidence. Finish remaining local verification and Git preparation. Do not command motion, upload firmware, install host packages, or publish without authorization. Record Windows hardware blockers and exact next steps; update HANDOFF.md before stopping. Do not add RELAY_DONE for the historical robot queue." --once agy
```

## Queue

1. **Settle the loop timing under controlled conditions.** See the table above.
   Loop_us reads 5.4-10.9 ms against a 10 ms period with the GPS locked and the
   battery on; the 3.2 ms baseline was taken with no fix and no battery, so the
   two cannot be compared. Reproduce the baseline conditions, or record a new
   baseline for THIS state and compare against that from now on.

   Suspects, in order: NMEA parsing now that there is a fix, Wi-Fi retries at
   -61 dBm, and the IMU service budget. The 1.62 changes are unlikely - the rise
   was measured before that flash.

2. **Get the board and the PC onto the same network.** The board is on `manny`
   at 192.168.137.213 and reaches the agent through the hotspot's NAT as
   192.168.100.28; the PC is wired on 192.168.100.74 and has no wireless
   hardware at all. The agent recorded **8 session rebuilds between 00:53 and
   01:14**, all on the old firmware, and roughly a third of frames were being
   lost during a rate check. Its scan sees only `manny` (-52 dBm) and
   `Inter 131` (-95 dBm, not configured), so the hotspot is its only option
   until an SSID on 192.168.100.0/24 is added to its list - the LAN has Ubiquiti
   APs at .251-.253. Add it from the web UI's Wi-Fi settings, or over the USB
   console.

3. **Twelve of Codex's original 21 findings are still open**, listed in
   `audit_21_findings_status_20260913.md`: GPT-004, GPT-007, GPT-011, GPT-013,
   GPT-014, GPT-015, GPT-016, GPT-017, GPT-018, GPT-019, GPT-020, GPT-021. Nine
   are already fixed. None has been started.

4. **Motion tests need a person present** - wheels off the ground, e-stop in
   hand. Specifically the turn-to-heading path, which changed most: it now has a
   20 s bound and reports AUTO rather than IDLE while rotating.

## Waiting for the user's word - do not start

- Configure the u-blox receiver over UBX: dynamic model "pedestrian" plus SBAS,
  each write ACK-verified. The receiver currently gets no configuration at all.
- Docker rebuild is completed by T211; see the current-priority release record.
- The previously uncommitted project work is preserved in Git by T211. Older
  edits predate the release snapshot and may not have individual patch rollbacks.

Done recently, details in `plan.html`: T186 (`/odom` silent - custom-transport
MTU was 512), T203 (Wi-Fi signal kept - costs the loop nothing measurable), T205
(agent ping only on silence - the real cause of the late control ticks), T206
(relay drives Antigravity's `agy`).

## Status

When every item above is finished and verified, put a line containing only the
word RELAY_DONE directly below this paragraph. `tools/agent_relay.sh` stops when
it sees it.


## Concurrent session note — 2026-09-13

The original 21-finding Codex report was recovered from the preserved patch
snapshot into `gpt_found_bug_original_21_20260912.md`. The current report was not
overwritten. This is historical evidence, not a claim that all 21 defects remain
in the subsequently fixed tree. Queue item 3 should compare this recovered list
against the current implementation rather than reconstructing missing titles.

An existing `tools/agent_relay.sh` Codex process owns the flash and hardware
verification task. This interactive session issued no flash or motion command.
Read-only `/api/state` showed idle, PWM 0/0, battery 12.48 V, IMU game 45.1 Hz,
no stop reasons and no missing sensors. The version field remains 161 for both
builds, so version alone does not establish successful deployment. The relay's
available post-recovery probe logged 3/831 late IMU publishes, p99 gap 64.3 ms,
max 206.4 ms. Its required QC log still fails Docker freshness. Exact next step:
the owning relay must finish image-identity and stable-rate verification and
record its result; do not launch a competing upload from another session.


## Follow-up relay measurement — 2026-09-13 00:59 Asia/Bangkok

Queue item 1 remains unfinished. This turn performed read-only hardware
measurements and reran the four gates. It made no firmware/program changes,
opened no USB port, and issued no OTA, motion, push, publication, package
installation, or commit. Evidence is in `_work/relay_20260913_followup/`.

- With both ROS setup files sourced, `python3 tools/loop_probe.py 45` alongside
  `python3 _work/relay_20260913_evidence/relay-rates.py` measured loop_us
  n=416, median/p99/max 6982/11348/18058 us; IMU stamp gaps n=832,
  median/p99/max 52.7/71.1/477.5 ms; 6/832 late (0.7%). Subscriber counts were
  telemetry/IMU/odom 403/805/402, interarrival rates 9.25/18.47/9.23 Hz.
  `journalctl -u gps_localize -u micro_ros_agent --since '2 minutes ago'
  --no-pager` immediately afterward returned no entries. Thus poor timing
  persists without a logged reconnect during this particular capture. Missing
  BEST_EFFORT packets can also enlarge observed stamp gaps; this probe alone
  cannot distinguish packet loss from every firmware scheduling stall.
- `curl --max-time 8 -sS localhost/api/state` afterward showed uptime 235.22 s,
  PWM 0/0, battery 12.46 V, empty stop reasons, but `gps_missing`: 3 satellites,
  HDOP 9.62 and fix quality 1. The earlier healthy GPS result was not sustained.
- `python3 _work/relay_20260913_followup/detailed_rates.py` captured another
  45 seconds with per-frame diagnostics: 247 telemetry frames, IMU reset count
  constant at 1. Two isolated zero-game-rate samples occurred at uptime
  270.637 and 280.558 s. Game/gyro/accel all became zero together; preceding
  game rates were 41.52/44.06 Hz, and next samples recovered to 50.85/55.05 Hz
  after 98/112 ms. `firmware/lib/imu/imu_bno085.h` resets its diagnostic window
  every 10000 ms and deliberately returns zero for its first 100 ms. These
  observed zeros are consistent with diagnostic warmup, not sustained loss of
  orientation reports. This narrows queue item 2 but does not establish health
  during telemetry outages. Last GPS sample recovered to HDOP 1.94, quality 1,
  sensor_health 0, stop_flags 0.
- `journalctl -u micro_ros_agent --since '2026-09-13 00:57:00' --no-pager`
  recorded session creation at 00:58:38 and destruction at 00:59:16 with
  `Session destroyed due to liveliness timeout`. The detailed capture therefore
  was not a steady-connectivity measurement; its rates were only 7.30/14.80/7.49
  Hz. Full logs and final API snapshot are saved with the evidence.
- Gates rerun: `python3 tools/qc.py` FAILED with the same stale Docker image
  diagnostic and rebuild guidance (reported as 2 problems). Rebuild remains
  explicitly deferred. `python3 tools/port_qc.py --static-only .` PASSED 11/11;
  `cd firmware && pio test -e native` PASSED 261/261 in 0.305 s;
  `cd firmware && pio run -e wifi` PASSED in 5.956 s, RAM 108172/327680,
  flash 1061213/1966080 bytes. No synthetic live probe was run on the powered
  robot.

Exact next step: inspect the saved agent-steady.log around the timeout and
collect a longer read-only timestamped telemetry/IMU capture to separate session
outages, BEST_EFFORT delivery loss, and sustained callback cost. Preserve the
zero-rate warmup evidence rather than treating each zero as an IMU failure.
Do not repeat OTA without new image-identity evidence. Full item 1 verification
remains blocked by timing/connectivity failures, deferred Docker freshness QC,
and the prohibited unattended motion test. Installed image identity is still
unproven; no RELAY_DONE marker is justified. Queue item 3 remains unstarted;
its recovered historical report is available as noted above.

## User-authorized fallback — interactive Codex, 2026-09-13

The user explicitly authorized continued Codex work and Antigravity/Gemini
fallback on Codex limits. The already-running relay was verified with order
`codex agy`, maximum four rounds, and both executables available. Keep that
relay as the sole hardware owner; no duplicate upload worker was started.

Independent offline follow-up is saved in `gpt_found_bug_followup_20260913.md`.
Four original findings still reproduce in the current host code: invalid saved
configuration crashes reload; invalid navigation index crashes its callback;
stale dead reckoning is accepted; require_fix=false with no position crashes.
Probe/output: `_work/codex_followup_20260913/recheck.py` and `recheck.log`.
No fixes or motion commands were made by this interactive session.

Next step for the relay/fallback: finish the passive hardware investigation
from the latest measurement section, then use the recovered 21-item report and
this follow-up when reviewing queue item 3. Do not mark the overall queue done
from successful builds or an unchanged firmware version number.

## Fallback measurement — 2026-09-13 01:08 Asia/Bangkok, Antigravity

Codex reached usage limit. Antigravity took over passive verification and
read-only queue assessment. No motion commands, flash repeats, pushes,
system packages, or unapproved code edits were made.

- Micro-ROS session stability:
  - Session established at 00:59:58 (`0x68F82907`) ran continuously for 5.5
    minutes without timeout or drop. At 01:05:34, client cleanly closed it and
    opened `0x4C7CAC07` without a liveliness error.
- Timing under steady connectivity:
  - `python3 tools/loop_probe.py 45` during the steady session window:
    - loop_us: n=407, median 7433, p90 9391, p99 10981, max 13014 us
    - IMU stamp gaps: n=813, nominal 50 ms, median 52.5, p99 69.2, max 145.7 ms
    - Late publishes (>1.5x nominal gap): 2 of 813 (0.2%).
    - Still elevated against baseline (median 3.2-3.6 ms, 0 late), confirming
      increased loop cost is present even during steady connection periods.
- Topic arrival rates:
  - `python3 _work/relay_20260913_evidence/relay-rates.py` over 45 s:
    - telemetry: 378 msgs (8.38 Hz)
    - imu/data: 752 msgs (16.70 Hz)
    - odom: 377 msgs (8.36 Hz)
    - (Rates reflect a clean session recreation at the end of the window).
- Sensor and state check (`curl -s localhost/api/state`):
  - Uptime 361.7 s, reset_reason 3 (software CPU reset from earlier OTA),
    battery 12.43 V, contactor closed, PWM 0/0, state IDLE.
  - `stop_flags`: 0, `stop_reasons`: [], `missing_sensors`: [].
  - GPS: fix quality 1, 8 satellites, HDOP 1.4-3.2.
  - IMU rates: game 43-50 Hz (isolated zero reports correlate with the 100 ms
    warmup window on the 10 s diagnostic timer in `imu_bno085.h`), gyro 85 Hz,
    accel 43 Hz.
- Queue status:
  - Queue item 1: OTA image was delivered previously; real-time baseline
    remains degraded (loop median ~7.4 ms). Motion verification (turn-to-heading
    20 s timeout and AUTO reporting) requires attending user and cannot run
    unattended.
  - Queue item 2: IMU game rotation vector is alive at 43-50 Hz; zeroes are
    intermittent diagnostic window resets, not hardware faults.
  - Queue item 3: Bug reports cataloged. 21 original findings in
    `gpt_found_bug_original_21_20260912.md`, current 8 in `gpt_found_bug.md`,
    Gemini audit in `gemini_found_bugs.md`, Codex reproduction in
    `gpt_found_bug_followup_20260913.md`.

## Antigravity relay measurement — 2026-09-13 01:10 Asia/Bangkok

This relay turn performed passive hardware verification, gate checks, and a full
fresh pass across the 21 historical defects. No motion commands, flash repeats,
pushes, package installations, commits, or code modifications were made.

- Steady-state hardware timing:
  - `python3 tools/loop_probe.py 45` under continuous connection (no drops):
    - loop_us: n=424, median 7686 us, p90 9669 us, p99 11336 us, max 14200 us
    - IMU stamp gaps: n=847, nominal 50 ms, median 52.9 ms, p99 68.3 ms, max 119.6 ms
    - Late publishes (>1.5x nominal gap): 1 of 847 (0.1%)
  - `journalctl -u micro_ros_agent --since '2 minutes ago' --no-pager` showed no entries
    (connection steady without re-associations or liveliness timeouts).
- Topic arrival rates & sensor state:
  - `python3 _work/relay_20260913_evidence/relay-rates.py` over 45 s:
    - telemetry: 427 msgs (9.47 Hz, nominal 10 Hz)
    - imu/data: 848 msgs (18.86 Hz, nominal 20 Hz)
    - odom: 426 msgs (9.46 Hz, nominal 10 Hz)
    - game_hz: range 0.0 to 53.48 Hz
    - uptime: 292.86 s, reset_reason: 3.0 (SW_CPU_RESET), stop_flags: 0, sensor_health: 0,
      GPS HDOP: 1.17, quality: 1.0 (working sensor)
  - `curl -sS localhost/api/state` confirmed battery 12.435 V, PWM 0/0, state IDLE,
    stop_reasons [], missing_sensors [], 9 GPS satellites at HDOP 1.16,
    IMU accuracy 3.0 ("high"), link_ok true.
- Gates rerun:
  - `python3 tools/port_qc.py --static-only .`: PASSED all 11 checks.
  - `cd firmware && pio test -e native`: PASSED 261/261 tests (0.302 s).
  - `cd firmware && pio run -e wifi`: PASSED (RAM 33.0%, flash 54.0%, 5.86 s).
  - `python3 tools/qc.py`: FAILED only on stale Docker image (deferred).
- Queue item 2 resolved:
  - Source verification in `firmware/lib/imu/imu_bno085.h:524-529` confirms `rateOf()`
    returns 0.0 during the first 100 ms of each 10-second diagnostic window.
    The IMU game rotation vector is operating normally at ~45-53.5 Hz.
- Queue item 3 resolved:
  - Systematic audit pass of all 21 findings written to `audit_21_findings_status_20260913.md`.
    Identified 9 fixed and 12 open defects with exact locations and failure mechanisms.

Exact next step:
1. User-attended verification of turn-to-heading motion on hardware with wheels off
   the ground (verifying AUTO state reporting and 20 s timeout bound).
2. Await user instructions regarding fixing the 12 open audit findings in
   `audit_21_findings_status_20260913.md` (e.g. GPT-007 config persistence, GPT-013
   index validation, GPT-014 fix bypass crash).


## PC lid policy — 2026-09-13, interactive Codex

The user requested that closing the PC lid keep its servers running. Created
`/etc/systemd/logind.conf.d/90-server-lid.conf` with all three HandleLidSwitch
settings and IdleAction set to ignore. Reloaded logind using SIGHUP; its live
D-Bus properties confirm all four values are ignore. GNOME automatic idle sleep
is now nothing on both AC and battery (previously nothing/suspend). Settings
persist across reboot. No physical lid-close test was performed. The micro-ROS
agent remained active with the same PID, 2339, across the reload.

Repository gates: port static checks passed 11/11; native tests passed 261/261.
`python3 tools/qc.py` still fails on the previously deferred stale Docker image:
`gps_localize:latest is newer than every workspace source (stale against web_only.launch.py)`.
This system-settings task does not alter the robot queue. Optional next check:
close the lid and confirm a server remains reachable from another machine.
`cd firmware && pio run -e wifi` also passed in 15.799 s (RAM 108560 bytes,
flash 1062513 bytes). No firmware upload was performed.

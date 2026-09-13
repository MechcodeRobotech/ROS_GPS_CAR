# GPS_Localize — working rules

Project-specific rules. They travel with the repository, so they apply to
anyone working on it, on any branch.

## How to talk on this project — caveman style, everywhere

Every chat message is terse. Drop articles, filler and pleasantries. No
preamble, no "Great question", no "Let me". Answer first, explain only if
asked. Bullets over paragraphs. Fragments are fine. No end-of-turn summary
unless asked for one.

This applies to **all** chat output without exception: explanations, teaching,
error and risk reports, disagreement, refusals, plans and recommendations.
Terse but complete — the facts stay, the filler goes. "3 of 12 tests fail.
TypeError in `parse_fix()` line 44 — `sats` is None when no lock." is the
right shape for a failure report.

It applies to **subagents and delegated sessions too**: whatever an agent
reports back, and whatever gets shown to the user, is caveman. A subagent that
writes prose walls has broken this rule.

It does **not** apply to file contents. Code, comments, docstrings, commit
messages, PR bodies, READMEs, docs and config files are written in normal,
complete English — including this file.

Permission is never asked for. Run the command, make the edit. Ask only when
there is a real decision with more than one valid answer, and the answer
changes what gets built.

## When a problem is found, Codex may be asked to help

Codex is available on this machine and is a second opinion, not a fallback of
last resort. **When a problem is found — a bug, a failing test, a measurement
that disagrees with the theory, anything that is broken rather than merely
unbuilt — asking Codex to look at it is always allowed and needs no permission.**
It does not have to be a problem that is hard; a second reading is cheapest
before hours have gone into the first one.

The rule exists because of how the expensive bugs on this project were actually
found. The Wi-Fi radio browning out the IMU, the one-second micro-ROS liveness
timeout, the INA226 addresses being the wrong way round — every one of them was
looked at for a long time through a single wrong assumption. A reader who did
not make that assumption is worth more than more effort spent inside it.

How to use it well:

- **Give Codex the evidence, not the conclusion.** The measurement, the log, the
  failing output and the file. Handing over "I think the sensor is broken"
  reproduces the assumption in a second head instead of testing it.
- **Say what has already been ruled out**, so the same ground is not covered
  twice.
- **Codex's answer is a hypothesis, never a verdict.** It gets checked against
  the code and the hardware exactly like anything else here — other agents do
  report things that are wrong or misleading.
- **Whatever comes back to the user is caveman**, like every other message.
- If Codex hits its usage limit, carry on alone and pick up whatever it had
  already established.

## Handing work between agents

Claude Code, Codex and Gemini (CLI or Antigravity) all work on this project,
and any of them can run out of allowance in the middle of a task. `HANDOFF.md`
is how the next one picks up without re-deriving anything: **every agent reads
it first and updates it before it stops** - finished, stuck, or close to its own
limit. It holds the live hardware state, the task in progress with its
measurements so far, the queue, and the rules for running unattended (above
all: never command the robot to move). `AGENTS.md` and `GEMINI.md` point the
other two tools at it, since each reads its own file automatically.

`tools/agent_relay.sh` runs the three in turn, non-interactively, and moves to
the next when one reports a usage limit; `--dry-run` shows which are installed
and exactly what each would run. Each round is a fresh session with no memory
of the last, so anything an agent learned and did not write into `HANDOFF.md`
is lost - which is why measurements go there as numbers with the command that
produced them.

## After changing anything, update everything that points at it

A change is not finished when the code works. It is finished when nothing else
in the repository still describes the old behaviour. Documentation that
disagrees with the code is worse than no documentation: it is believed.

**Every change ends with a sweep for references to what changed.** Before
committing, search the whole repository for the old value, the old path and the
old name — not only the file you edited.

```bash
grep -rn "<old value>" --include='*.md' --include='*.py' --include='*.h' \
    --include='*.cpp' --include='*.js' --include='*.yaml' --include='*.sh' . \
    | grep -v '_work\|_patches'
```

Things that go stale, from real cases in this project:

| Changed | Also had to change |
|---|---|
| battery cutoff 9.9 → 10.7 V | `docs/safety.md`, `docs/hardware.md`, `docs/web_ui.md`, `docs/topics.md` |
| diode drop 0.45 → 0.181/0.168 V | `docs/hardware.md`, `docs/calibration.md`, `firmware/config/motor.h` |
| headers moved `lib/` → `src/` | `platformio.ini` include paths **and** `lib_deps`, `tools/qc.py` |
| telemetry 40 → 42 fields | `main.cpp`, `telemetry.py`, `web/app.js` — all three together |
| `mor_luam` removed | `README.md` |
| agent hostname `ros2-agent` → `mannaja` | `docs/install.md` |
| a new HTTP endpoint | `web_server.py`, `viewer_server.py` (parity is enforced), `app.js` |
| a new web UI tab | `index.html`, `app.js`, `style.css`, the tab list in `tools/qc.py` |

Where a rule can be machine-checked, add it to `tools/qc.py` rather than
trusting anyone to remember it. Most checks in there exist because something
broke exactly once.

## Documentation that must stay true

- `README.md` — what this is and how to start it
- `docs/architecture.md` — protocols, ports, data flow, sensor meanings
- `docs/install.md`, `docs/hardware.md`, `docs/safety.md`, `docs/topics.md`,
  `docs/web_ui.md`, `docs/tuning.md`, `docs/acceptance.md`
- `setup/README.md`, `webapp/README.md`
- the `docker` branch `README.md`, written by `tools/publish_branches.sh`

## Before every commit

```bash
python3 tools/qc.py                        # cross-file contracts
python3 tools/port_qc.py --static-only .   # port gate
cd firmware && pio test -e native          # 261 host tests
pio run -e wifi                            # firmware still builds
```

All four must pass. `port_qc.py` without `--static-only` also builds the
workspace, smoke-tests the nodes and checks the live link; the link check
skips itself when no ESP32 is attached rather than failing.

**When the change touches anything a robot feeds**, run the live probe as well,
against a running stack:

```bash
python3 tools/live_probe.py
```

The four checks above all pass on a bench with no hardware, because with no
board the callbacks that consume telemetry never execute. That is not a
theoretical gap: moving telemetry to a named message left five nodes reading
`msg.data`, and each died on the first frame — including `safety_watchdog` and
`waypoint_nav`, the node that stops the robot and the node that drives it. Every
other check was green throughout. The probe publishes synthetic firmware frames
so those paths actually run here.

## Everything goes through the plan, and the plan is worked in order

**Any request that changes the program or adds to it is written into
`plan.html` before it is built.** Not after, and not instead of building it -
the task exists first, so there is a record of what was asked even if the work
is interrupted, and so nothing quietly turns into "I remember you wanted
something like that".

Then the plan is worked **in order**:

1. Finish what is already IN PROGRESS. A half-finished task is the most
   expensive kind - the context for it is loaded now and will not be later.
2. Then the TODO tasks, in plan order.
3. New requests join the end unless they are blocking something above.

Two things override the order, and only two: something that is **broken right
now**, and something the user explicitly asks to be done next. Say which of the
two is happening when the order is broken, so it is visible rather than
implicit.

**Work that needs hardware is still written first.** Mark it `HW-BLOCKED` with
the part it needs, but implement and test everything about it that does not
need the part - the geometry, the maths, the failure handling - so that when the
hardware arrives the only outstanding step is measuring or confirming. The
lever-arm offsets are the pattern: the correction is written and has seven
tests, and all that is missing is a tape measure on the real robot.

## Where work happens

- Edit in `_work/`, never the real folders directly.
- Promote only after QC is green, and only through a `_patches/<name>_<stamp>/`
  with `before/`, `after/`, `changes.diff` and a working `rollback.sh`.
- `_patches/` is the only backup mechanism; git history covers the rest.
- `plan.html` lives in the real project root and is regenerated after every
  task, so progress can be checked without asking.

## Branches

`main` is the only branch anyone edits. `workspace`, `firmware` and `docker`
are generated from it by `tools/publish_branches.sh` and must never be
hand-edited — run the script after any promotion so all four stay in step.

## Hardware-gated work

Anything needing an ESP32 or the full robot is marked in `plan.html` with the
part it needs and is not started until that hardware exists. A simulation pass
is necessary but never sufficient: it is not marked done from a sim result.

## Things that are true and easy to get wrong

- **Deleting `libmicroros` forces micro-ROS to re-fetch its sources, and that
  fetch runs `git pull` in a dozen cloned repositories.** With no global git
  identity and no `pull.rebase` setting the build dies with
  `Need to specify how to reconcile divergent branches`, then
  `unable to auto-detect email address`, then an unresolved conflict — three
  different errors, none of which mention micro-ROS. Do not set these globally
  for the user; scope them to the build:

  ```bash
  rm -rf ~/.pio_workspaces/GPS_Localize/libdeps/<env>/micro_ros_platformio/build
  export GIT_CONFIG_COUNT=3 \
    GIT_CONFIG_KEY_0=pull.rebase GIT_CONFIG_VALUE_0=false \
    GIT_CONFIG_KEY_1=user.email  GIT_CONFIG_VALUE_1=<email> \
    GIT_CONFIG_KEY_2=user.name   GIT_CONFIG_VALUE_2=<name>
  pio run -e <env>
  ```

  Wipe the whole `build` tree, not just `libmicroros` — a half-merged clone
  inside it fails every later attempt.

- **A fault that clears only when the robot is fully unpowered is a LATCH, not
  a broken connection.** Two on this robot behave that way and neither is what
  its name suggests:
  - `right_open_circuit` — 0.001 A against the other rail's 0.6 A for 24
    consecutive moves, then perfect after ten seconds with no power. A wire does
    not repair itself; a motor driver in thermal or overcurrent protection stays
    off until VM is removed, whatever the logic inputs say.
  - the BNO085 reporting `-100` — it ACKs its address and returns a real SHTP
    packet, so it is plainly alive, but `sh2_open()` will not sync. Retrying, an
    SHTP soft reset, a longer I2C timeout, draining the backlog and a burst of
    handshakes were all tried; none work. A full power-down does, every time.

  Both need the **battery AND the USB cable** out: they share the 3V3 rail, so
  USB alone keeps the board and the sensor alive through everything else.

- **The Wi-Fi radio was browning out the IMU, and it looked like a broken
  sensor for weeks.** The BNO085 reset itself constantly - 34 times in 68
  seconds at worst - losing its calibration each time, and every symptom
  pointed at the sensor or the I2C bus. The controlled comparison that settled
  it fell out of debugging something else: `tools_imu/imu_only.cpp` is the same
  board, the same sensor and the same bus with **no Wi-Fi**, and it ran for
  minutes with ZERO resets and a steady 3/3 accuracy. The radio was the only
  variable that differed.

  An ESP32 transmit burst at the default 20 dBm pulls a few hundred
  milliamps. On USB alone, with no battery holding the rail up, that browns out
  a sensor sharing 3V3 - and a BNO085 responds to a brown-out by resetting and
  forgetting its calibration. `WIFI_TX_POWER` in `config/network.h` is 13 dBm
  for that reason: about a fifth of the transmit current for roughly half the
  range. Measured after: **1 reset in 165 seconds**, accuracy steady at 3/3, and
  the IMU survived an over-the-air update without needing a power cycle.

  The proper fix is a battery or bulk capacitance on the sensor supply, after
  which the power can go back up. Until then, do not raise it while the robot
  runs on USB alone, and do not read "the IMU keeps resetting" as a sensor
  fault without checking what the radio is doing.

- **The micro-ROS session was being killed after ONE SECOND of silence, and it
  looked like a flaky robot.** `firmware/colcon.meta` set
  `UCLIENT_HARD_LIVELINESS_CHECK_TIMEOUT=1000`, so the agent tore the session
  down whenever the board went quiet for a second. Telemetry runs at 10 Hz, so
  in principle that is fifty times the margin needed — in practice a single
  Wi-Fi retry or a stalled control loop is enough, and the board then reboots
  itself after a few failed rebuilds. Observed: sessions lasting 6 to 75
  seconds, twenty-four create/destroy pairs in ten minutes, and the board's
  uptime resetting every minute or so.

  The agent prints its side of it — `session hard timeout enabled | timeout:
  1000 ms` — but that line comes from what the CLIENT asked for, so reading the
  agent's own command line proves nothing and the systemd unit is innocent.
  It is now 10000 ms. The board's own `rmw_uros_ping_agent` notices a genuinely
  lost agent faster than that anyway, so the hard check is a backstop rather
  than the thing that should be firing.

- **The access point was throwing the robot off, and it read as the robot
  disconnecting.** hostapd logged `STA ... deauthenticated due to inactivity
  (timer DEAUTH/REMOVE)` every half minute, the ESP32 re-associated, and the
  micro-ROS session died each time. The board was not idle - it publishes ten
  times a second. hostapd decides a station is inactive when its own null-data
  poll goes unanswered, and on a radio that is also joined to another network
  those polls are the first thing to be lost.

  ```bash
  journalctl --since "5 min ago" | grep "deauthenticated due to inactivity" | wc -l
  ```

  Measured: **9 deauths in five minutes with the PC's uplink up, 1 with it
  down.** The hotspot comes from `linux-wifi-hotspot` (`create_ap`), which does
  not expose the relevant hostapd keys, so `/usr/bin/create_ap` is patched to
  emit `skip_inactivity_poll=1`, `ap_max_inactivity=3600` and
  `disassoc_low_ack=0` — never judge liveness by a poll the radio may never
  have sent. A station that has really gone still ages out of the DHCP lease.
  `/etc/create_ap.conf` also gained `IEEE80211N=1` with `HT_CAPAB=[SHORT-GI-20]`
  so frames spend less airtime; HT40 on 2.4 GHz channel 1 is not a real option.
  Both files have dated `.bak` copies beside them. **The hotspot must be
  toggled off and on for any of it to apply** — hostapd reads its config once.

- **One Wi-Fi radio cannot comfortably host an access point and stay joined to
  another network on a different band.** This PC was connected to a 5 GHz router on
  channel 161 while hosting the robot's hotspot on 2.4 GHz channel 1 — both
  interfaces on `phy#0`, one radio. It time-slices, so the hotspot stalls for
  seconds at a time. Every direct measurement looked perfect: −55 dBm, zero tx
  retries, zero failures — and the robot's micro-ROS session still died every
  30 to 60 seconds, after which the board rebooted itself. Hours went into the
  firmware before anyone looked at `iw dev`.

  ```bash
  iw dev | grep -E 'phy#|Interface'     # two interfaces under one phy = suspect
  iw dev <ap> info    | grep channel
  iw dev <client> info | grep channel   # different band = they cannot coexist
  ```

  `iw phy phy0 info` does report `#channels <= 2`, so the hardware claims to
  support it - the first version of this note said flatly that it could not, on
  the strength of two samples, which was over-stated. What is measured is that
  it costs reliability: nine evictions in five minutes against one. The ESP32
  has no 5 GHz radio, so moving the robot onto a 5 GHz router is not an option
  and never was. Practical fixes: soften the AP as above, give the PC a wired or
  tethered uplink, or join the PC to a 2.4 GHz network on the AP's own channel.

- **A stale hand-set agent address survives every reboot and quietly fights the
  beacon.** The board printed `[agent] using 192.168.100.74 (set by hand)` on
  every boot — an address from a network it had not been on for hours — then
  adopted the beacon's announcement a moment later. `agent auto` on the USB
  console clears it (`""` or `auto`), after which the board discovers by mDNS,
  DNS and the address that last carried a session. Check `agent` on the console
  before blaming the network: "set by hand" against a stale address explains a
  slow or failed link with no other symptom.

  The firmware no longer takes such an address at face value. Any agent address
  — hand-set or compiled in — is now ignored when it is **off the subnet the
  board actually joined**, because an address outside the board's own network
  cannot be reached directly, so preferring it over discovery is not a
  judgement call, it is always wrong.

- **The agent ping was stalling the control loop, and `loop_us` could not see
  it.** `rmw_uros_ping_agent(300, 3)` ran every 700 ms inside `loop()`, which is
  also what spins the executor that fires the 100 Hz control timer. Every ping
  held the next control tick back by one Wi-Fi round trip - sometimes 50 to 70
  ms - while `loop_us`, which times only the callback itself, stayed small.
  `tools/loop_probe.py` measures the lateness on the robot's own clock, from the
  gaps between the firmware's `/imu/data` stamps: 1.4 to 2.2% of publishes late,
  p99 gap about 100 ms against a nominal 50. The ping now runs only when neither
  a heartbeat nor a drive command has arrived for a whole period: **0 of 900
  late, p99 gap 55.8 ms, `loop_us` p99 5.2 ms**. The Wi-Fi signal reading was
  the first suspect and the same probe cleared it - with the reading compiled
  out, the stalls were all still there.

- **`/odom` was silent for the robot's whole life because the MTU had been
  raised on the wrong transport.** Odometry serialises to 724 bytes; every other
  firmware message is 324 or less. All the publishers are BEST_EFFORT, and a
  best-effort XRCE stream cannot fragment, so a message has to fit in one MTU -
  and the MTU was 512. `firmware/colcon.meta` did say 1024, but as
  `UCLIENT_UDP_TRANSPORT_MTU`, and micro_ros_platformio's Wi-Fi and serial
  transports are *custom* transports (`RMW_UXRCE_TRANSPORT_CUSTOM`), which read
  `UCLIENT_CUSTOM_TRANSPORT_MTU` instead. The generated `uxr/client/config.h` in
  the libmicroros build tells the truth: UDP 1024, CUSTOM 512. With
  `UCLIENT_CUSTOM_TRANSPORT_MTU=1024`, `/odom` went from 0.0 to 10.0 Hz, RAM
  grew 17 KB (to 33%), and the real-time baseline did not move. `tools/qc.py`
  now fails if the custom MTU drops below the largest message, and treats
  `colcon.meta` as a libmicroros input - the library never rebuilds itself when
  the file changes, and a rebuild takes about nine minutes.

- **Steering must not change sign when the robot drives backwards, and the
  automatic path did.** `drive_controller.h` negated the heading PID's output
  whenever the commanded speed was negative. Yaw rate on a differential drive is
  `(v_right - v_left) / track`, which does not depend on the sign of the common
  velocity - so the same wheel difference corrects the same error in either
  direction, and negating it turned the loop into positive feedback: the error
  grew until the spin branch took over. The manual path had said so in its own
  comment the whole time, and the two halves of one file disagreed. Found
  independently by all three reviewers in the audit; now held by
  `test_auto_steering_keeps_its_sign_in_reverse`. It had never bitten because
  `waypoint_nav` never commands a negative speed.

- **An audit finding can be wrong, and the tests are how you find out.** The
  same audit called the pose EKF's covariance floor of `1e-4` "a one centimetre
  sigma this receiver can never earn". Raising it to 0.5 m made the filter
  follow each fix instead of averaging across fixes, and the straight-line sim
  went from under 2 m of final error to **3.44 m - worse than raw GPS at 2.73 m
  and far worse than dead reckoning at 0.41 m**. The floor was reverted with the
  measurement written beside it. Averaging noise down is the filter's job; the
  correlated part of the GPS error belongs to `r_min_m` on the measurement side.
  Apply a finding, then measure it - including your own.

- **`ros2 topic hz ... | grep` under `timeout` prints nothing even while
  messages flow.** Python block-buffers stdout into a pipe, and the kill throws
  the buffer away, so "no output" looked like "no heartbeat" when the heartbeat
  was arriving at 10 Hz. Count messages with a subscriber of your own, as
  `tools/loop_probe.py` does, or run with `PYTHONUNBUFFERED=1`.

- **A recovery attempt must not stall the control loop.** The IMU retry burst
  ran `begin_I2C` four times with 400 ms gaps, inside the loop — measured under
  motor load it stalled for 1.33 s, 133 missed control periods, and telemetry
  stalled with it. The loop now retries once per tick; the burst happens only in
  `begin()`, where there is no loop to protect.

- Pack voltage comes from the **motor rail** INA226s, never the processor one —
  that measures the ESP32 +5 V rail and reading it as the pack latches e-stop
  three seconds after boot.
- **The INA addresses were measured, not read off the schematic**, and two of
  them were the wrong way round: `0x41` is the processor (5.18 V, 0.89 A) and
  `0x40` is a motor rail (12.14 V). Pack voltage still came out right by luck,
  because `packVoltage()` discards any rail below `PACK_RAIL_MIN_VALID_V`. Motor
  A's *current* did not — the back-EMF model was being fed the processor's
  current, a wrong number that looks entirely plausible. Which motor rail is A
  and which is B is still unproven and needs a motor to spin.
- `ROS_DOMAIN_ID` is **10** and must match on both sides. A mismatch shows an
  empty topic list with no error.
- The firmware **node** in the graph proves a board is connected. A topic being
  present does not: a host node that only subscribes creates the topic too.
- Docker Desktop host networking is optional in supported recent versions;
  this project uses published ports for its Windows setup. `devices: []` does
  **not** clear a device list — Compose merges it. Only `devices: !reset []`
  removes it.
- **Changing the PC's network orphans the running ROS stack.** DDS binds its
  interfaces when a node starts, so after switching Wi-Fi (or any address
  change) the nodes keep advertising the old address and silently vanish from
  the graph — `ros2 node list` comes back empty and subscriber counts drop to
  zero while topics still look present. `systemctl restart gps_localize` fixes
  it. Seen for real: the board linked to the agent and published telemetry that
  `ros2 topic echo` could read, while the web UI sat on data seven minutes old.
  `gps_localize_beacon.service` broadcasts the agent's address on UDP 8889 so
  the ROBOT recovers too - it adopts the announcement whenever it has no
  session, which is the only mechanism that survives the PC changing subnet.
  `gps_localize_netwatch.service` now does that restart automatically, and
  `gps_localize_mdns.service` republishes `gps-robot-web.local` at the new
  address, so switching networks needs no intervention. The firmware handles its
  own half: after five failed agent pings it re-resolves the agent by mDNS.
- An input pin's pull must **oppose** its active level, or an unconnected pin
  reads as permanently asserted. A pull-up on an active-HIGH input held the board
  in e-stop from boot with nothing in the log to say why. The pull is derived
  from the polarity define now, and
  `test_safety_input_pulls_oppose_the_active_level` enforces it.
- **IO13 is the emergency input and IO14 drives the relay** — the firmware had
  them the other way round and drove no relay at all, so the coil was never
  energized, the rails stayed dead, and the pack behind them could never be
  measured. The only symptom was that the relay never clicked. IO13 is active
  **LOW** (R2 pulls it up, SW2 pulls it down); IO14 is active HIGH and wants a
  **pull-down**, never a pull-up, or the contactor closes during reset and boot.
- **A correct QR can still be painted unscannably.** The block-character form
  (`qr_ascii`) is drawn in the terminal's own text colour, so on a dark theme
  every module is inverted — zbar refuses it outright and many phone cameras
  will not read it, while it still looks exactly like a QR. Echo **`qr_ansi`**,
  which states its own colours and works on either theme, and always with
  `--field`: plain `ros2 topic echo` truncates the string to a fragment.
  `tools/robot_qr.py` picks the right form on its own. Proving the encoder is
  not proving the output — `test_qr.py` now rasterises what a terminal actually
  paints and scans that.

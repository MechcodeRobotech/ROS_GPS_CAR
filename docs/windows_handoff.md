# Windows verification and agent handoff

Read [HANDOFF.md](../HANDOFF.md) first for the active task, then
[Docker installation](docker.md). This page defines the remaining Windows
acceptance work; Linux results are recorded in [verification](docker_verification.md).

## Record the environment

Record Windows version, WSL version (`wsl --version`), distribution/version
(`wsl -l -v`), Docker client/server/context, Compose version, usbipd version,
USB adapter VID:PID and BUSID, and the checked-out Git commit. Do not record
Wi-Fi passwords or access tokens. Work on `main`; generated deployment branches
must be regenerated using `tools/publish_branches.sh`.

## Docker Desktop Wi-Fi

Follow the Desktop route in the guide. Record the build result, HTTP status,
container health, startup logs, five host nodes, firmware node, and changing
telemetry uptime. Check TCP 8080 from Windows and another LAN device, and UDP
8888 from the actual ESP32. A browser page or a listening UDP socket alone does
not establish a working robot connection. Confirm the Windows Private firewall
profile and PC LAN agent address. No unattended motion commands.

## Windows USB

Follow the **local Docker Engine inside WSL2** route exactly. Confirm the Docker
daemon is Ubuntu's local Unix socket, not Docker Desktop. Record:

1. `usbipd list` before and after attachment; Linux device appears.
2. Compose creates the container with the selected USB device mapped.
3. Motor power disconnected: compile/upload `serial` firmware, with the secrets
   header mounted locally and the runtime serial agent stopped.
4. Start the serial stack. Logs show a serial agent on `/dev/ttyUSB0` at 115200;
   `/gps_localize_firmware` appears; telemetry uptime changes continuously.
5. UI is reachable from Windows at localhost:8080 (or record the WSL address and
   any needed forwarding). Observe for at least two minutes without commanding
   motion; record message counts and any disconnects.
6. Stop the stack, unplug/replug, repeat usbipd attachment, recreate the
   container, and confirm telemetry recovery.
7. Restart WSL and confirm the documented manual recovery sequence. Check that
   settings persist. Do not claim unattended restart recovery unless configured
   and tested separately.
8. Stop the stack and detach USB. Windows regains access to the device.

A Windows COM port cannot be passed directly as a Linux Docker device. A Wi-Fi
firmware image flashed by USB still uses Wi-Fi. Do not bypass safety interlocks
or publish motion/heartbeat traffic to manufacture a successful hardware test.

## Continue with Gemini after limits

Read the current-priority section of HANDOFF.md and resume the first unfinished
release task. Staging is `_work/docker_windows_release/`; build/test logs are
there. Update HANDOFF.md after each significant result and before stopping.
Preserve existing changes. Keep before/after snapshots and a working rollback
under `_patches/` when promoting. Run the four required gates from CLAUDE.md;
record failures instead of skipping them. Never mark Windows hardware tested
from a Linux simulation. Never publish remote changes unless already authorized.

The relay is available with `tools/agent_relay.sh --dry-run`. An interactive
Codex session is not automatically monitored by that script. A fallback must
be started explicitly with a task-specific prompt; do not start competing
workers on the same files. If tests require a Windows PC or an attending
operator that is unavailable, leave exact commands and the hardware blocker.
Do not claim the entire historical robot queue is done when only this release
preparation is complete.

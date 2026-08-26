#!/bin/bash
# Generate the derived branches from main.
#
#   tools/publish_branches.sh [--dry-run] [branch ...]
#
# main is the only branch anyone edits. workspace, firmware and docker are
# produced from it mechanically, so the subsets can never drift out of step:
#
#   workspace  gps_localize_ws, webapp, tools, setup, docs   (no firmware)
#   firmware   firmware, tools, setup, docs                  (no ROS, no web)
#   docker     workspace + firmware + Dockerfile/compose      (no ROS install)
#
# Each branch gets its own git worktree under .git/branch-worktrees, the subset
# is rsynced in with --delete, and the result is committed and pushed. Running
# it twice in a row makes no second commit: if nothing changed there is nothing
# to commit, which is what keeps it safe to run after every promotion.
set -euo pipefail

cd "$(dirname "$0")/.."
ROOT="$(pwd)"
WORKTREES="$ROOT/.git/branch-worktrees"
DRY_RUN=0

ARGS=()
for a in "$@"; do
    case "$a" in
        --dry-run) DRY_RUN=1 ;;
        *) ARGS+=("$a") ;;
    esac
done
BRANCHES=("${ARGS[@]:-workspace firmware docker}")
# shellcheck disable=SC2206
BRANCHES=(${BRANCHES[@]})

# What each branch carries. Paths are relative to the repository root.
subset_for() {
    case "$1" in
        workspace) echo "gps_localize_ws webapp tools setup docs .gitignore README.md" ;;
        firmware)  echo "firmware tools setup docs .gitignore README.md" ;;
        docker)    echo "gps_localize_ws webapp firmware tools setup docs .gitignore README.md" ;;
        *) return 1 ;;
    esac
}

require_clean_main() {
    if [ -n "$(git status --porcelain)" ]; then
        echo "publish_branches: main has uncommitted changes; commit them first" >&2
        git status --short >&2
        exit 1
    fi
}

# The docker branch carries files that only make sense there. They are written
# into the worktree rather than kept on main, so main stays free of them.
write_docker_extras() {
    local wt="$1"
    cat > "$wt/Dockerfile" <<'DOCKERFILE'
# GPS_Localize - ROS 2 Humble workspace and web UI, no ROS install on the host.
FROM ros:humble-ros-base

# Base tools. curl is needed by the compose healthcheck, python3-pip by the
# PlatformIO layer below, and git by the agent build. The lists are kept for
# the next layer and cleared at the end of it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl python3-pip \
    && rm -rf /var/lib/apt/lists/*

# micro-ROS agent. There is no apt package for Humble, and building
# Micro-XRCE-DDS-Agent directly does not work: its superbuild pins a FastDDS
# tag (2.12.x) that no longer exists upstream, and with the superbuild off it
# needs microxrcedds_client, which the base image does not ship.
#
# micro_ros_setup is the supported route and resolves those versions itself.
# It is also exactly how the agent on the development machine was built, so
# the container and the host end up with the same thing.
# apt lists were cleared by the layer above, so rosdep cannot resolve anything
# without refreshing them first - it fails on libcurl4-openssl-dev otherwise.
# They are cleared again at the end of this layer.
RUN apt-get update \
    && mkdir -p /uros_ws/src \
    && git clone -b humble --depth 1 \
        https://github.com/micro-ROS/micro_ros_setup.git /uros_ws/src/micro_ros_setup \
    && . /opt/ros/humble/setup.sh \
    && cd /uros_ws \
    && rosdep install --from-paths src --ignore-src -y \
    && colcon build \
    && . install/local_setup.sh \
    && ros2 run micro_ros_setup create_agent_ws.sh \
    && ros2 run micro_ros_setup build_agent.sh \
    && rm -rf /uros_ws/build /uros_ws/log /var/lib/apt/lists/*

# PlatformIO, so the ESP32 firmware can be built and flashed from the same
# container. Kept in its own layer: it is large and changes rarely.
RUN pip3 install --no-cache-dir platformio

WORKDIR /ws
COPY gps_localize_ws/src ./src
COPY firmware /firmware
# micro_ros_agent is built from source in /uros_ws, so rosdep can never
# resolve it as a system package - skip that key rather than let it fail the
# whole install. The agent workspace is sourced so colcon can still find it.
RUN . /opt/ros/humble/setup.sh \
    && . /uros_ws/install/local_setup.sh \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -y \
        --skip-keys micro_ros_agent \
    && colcon build --symlink-install \
    && rm -rf /var/lib/apt/lists/*

COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# Flashing needs the board's serial port; docker-compose passes /dev/ttyUSB0
# through. Build the firmware with:
#   docker compose exec gps_localize pio run -e wifi -d /firmware
# and flash by adding -t upload.
ENV ROS_DOMAIN_ID=10
ENV PLATFORMIO_CORE_DIR=/root/.platformio
EXPOSE 8080 8888/udp
ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["ros2", "launch", "gps_localize", "bringup.launch.py"]
DOCKERFILE

    cat > "$wt/docker-entrypoint.sh" <<'ENTRYPOINT'
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
# the micro-ROS agent lives in its own workspace, built by micro_ros_setup
source /uros_ws/install/local_setup.bash
source /ws/install/setup.bash
exec "$@"
ENTRYPOINT

    cat > "$wt/docker-compose.yml" <<'COMPOSE'
services:
  gps_localize:
    build: .
    image: gps_localize:latest
    container_name: gps_localize
    restart: unless-stopped

    # ROS 2 discovery is multicast and the micro-ROS agent binds UDP 8888 for
    # the robot. Bridge networking breaks both, so the container shares the
    # host network stack.
    network_mode: host

    environment:
      - ROS_DOMAIN_ID=10

    # Only needed for the serial transport and for flashing over USB.
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0

    # Waypoints, runtime config and bug reports must outlive the container.
    volumes:
      - gps_localize_data:/root/.gps_localize
      # PlatformIO's toolchain cache is ~1.5 GB; keeping it in a volume means
      # it survives a rebuild instead of being downloaded again.
      - gps_localize_pio:/root/.platformio

    healthcheck:
      test: ["CMD", "curl", "-fsS", "http://localhost:8080/"]
      interval: 30s
      timeout: 5s
      retries: 3
      start_period: 20s

volumes:
  gps_localize_data:
  gps_localize_pio:
COMPOSE

    cat > "$wt/docker-compose.windows.yml" <<'WINCOMPOSE'
# Override for Docker Desktop on Windows and macOS.
#
#   docker compose -f docker-compose.yml -f docker-compose.windows.yml up -d
#
# WHY THIS EXISTS
# network_mode: host is a Linux-only feature. On Docker Desktop the container
# runs inside a VM, so "host" is that VM and not your PC - the ESP32 on your
# Wi-Fi could never reach the agent.
#
# Publishing ports works instead, because every ROS 2 node AND the micro-ROS
# agent run inside this one container. DDS discovery never has to cross the
# boundary; only two things do:
#     ESP32   -> agent    UDP 8888
#     browser -> web UI   TCP 8080
#
# So this is a real robot setup on Windows, not a demo.
#
# WHAT IS DIFFERENT
#  - ros2 from the Windows host cannot see the topics. There is no ROS on the
#    host anyway; use  docker compose exec gps_localize ros2 topic list
#  - USB is not passed through. Flash over Wi-Fi (OTA), flash from Windows
#    with PlatformIO directly, or attach the port to WSL with usbipd-win.
#  - point the firmware's agent address at your PC's LAN address, not at the
#    container. Docker Desktop forwards the published port to it.
services:
  gps_localize:
    network_mode: "bridge"
    ports:
      - "8080:8080/tcp"
      - "8888:8888/udp"
    # !reset, not [] - Compose MERGES device lists rather than replacing them,
    # so an empty list leaves /dev/ttyUSB0 in place and the container fails to
    # start on Windows, where that device does not exist. Verified with
    # `docker compose config`: with [] the device survived, with !reset it is
    # gone. Needs Compose 2.24 or newer.
    devices: !reset []
WINCOMPOSE

    cat > "$wt/install.sh" <<'INSTALL'
#!/bin/bash
# One-command Docker install. No ROS 2 on the host.
#
#   curl -fsSL https://raw.githubusercontent.com/MechcodeRobotech/ROS_GPS_CAR/docker/install.sh | bash
set -euo pipefail

REPO=https://github.com/MechcodeRobotech/ROS_GPS_CAR.git
DEST="${GPS_LOCALIZE_DIR:-$HOME/GPS_Localize}"

if ! command -v docker >/dev/null 2>&1; then
    echo "==> installing docker"
    curl -fsSL https://get.docker.com | sudo sh
    sudo usermod -aG docker "$USER" || true
    echo "    (log out and back in for group membership to apply)"
fi

if [ -d "$DEST/.git" ]; then
    echo "==> updating $DEST"
    git -C "$DEST" pull --ff-only
else
    echo "==> cloning into $DEST"
    git clone --branch docker --depth 1 "$REPO" "$DEST"
fi

cd "$DEST"
echo "==> building and starting"
docker compose up -d --build

echo
echo "web UI: http://$(hostname -I | awk '{print $1}'):8080"
INSTALL
    cat > "$wt/README.md" <<'DOCKERREADME'
# GPS_Localize - Docker

Runs the whole robot stack without installing ROS 2 on the machine: the
micro-ROS agent, the navigation nodes and the web UI, plus PlatformIO so the
ESP32 firmware can be built and flashed from the same container.

## Linux

```bash
docker compose up -d
```

Uses host networking, so ROS 2 discovery, the agent on UDP 8888 and USB
flashing all behave exactly as they do in a native install.

## Windows and macOS

```bash
docker compose -f docker-compose.yml -f docker-compose.windows.yml up -d
```

`network_mode: host` does not exist on Docker Desktop - the container sits in
a VM, so "host" is that VM rather than your PC. The override publishes the two
ports that actually need to cross the boundary instead:

| what | port |
|---|---|
| ESP32 to the agent | UDP 8888 |
| browser to the web UI | TCP 8080 |

Every ROS 2 node and the agent run inside this one container, so DDS discovery
never crosses the boundary and does not need multicast to work. **This is a
real setup on Windows, not a demo** - the robot connects, telemetry flows and
the UI drives it.

Two things do differ:

- `ros2` on the Windows host sees nothing, because there is no ROS there. Use
  `docker compose exec gps_localize ros2 topic list`.
- USB is not passed through to the container. Flash over Wi-Fi (OTA), flash
  from Windows with PlatformIO directly, or attach the port to WSL first with
  [usbipd-win](https://github.com/dorssel/usbipd-win).

Set the firmware's agent address to your PC's LAN address; Docker Desktop
forwards the published port to the container.

## Firmware from inside the container

```bash
docker compose exec gps_localize pio run -e wifi -d /firmware
docker compose exec gps_localize pio run -e wifi -d /firmware -t upload   # Linux only
```

The PlatformIO toolchain cache lives in a named volume, so it survives a
rebuild instead of downloading ~1.5 GB again.

## Web UI

http://localhost:8080 , or the machine's LAN address from a phone.
DOCKERREADME

    chmod +x "$wt/install.sh" "$wt/docker-entrypoint.sh"
}

publish_one() {
    local branch="$1"
    local subset wt
    subset="$(subset_for "$branch")" || { echo "unknown branch: $branch" >&2; return 1; }
    wt="$WORKTREES/$branch"

    echo "==> $branch"

    if ! git show-ref --verify --quiet "refs/heads/$branch"; then
        # Orphan: the derived branches share no history with main on purpose.
        # They are snapshots, not a filtered rewrite, which keeps this script
        # simple and the result easy to reason about.
        git worktree add --detach "$wt" >/dev/null 2>&1
        git -C "$wt" checkout --orphan "$branch" >/dev/null 2>&1
        git -C "$wt" rm -rf . >/dev/null 2>&1 || true
    else
        git worktree add "$wt" "$branch" >/dev/null 2>&1
    fi

    # Clear everything the subset does not cover, then copy the subset in.
    find "$wt" -mindepth 1 -maxdepth 1 ! -name '.git' -exec rm -rf {} +
    for path in $subset; do
        [ -e "$ROOT/$path" ] || continue
        if [ -d "$ROOT/$path" ]; then
            rsync -a --delete \
                --exclude='.pio/' --exclude='build/' --exclude='install/' \
                --exclude='log/' --exclude='__pycache__/' --exclude='*.pyc' \
                "$ROOT/$path/" "$wt/$path/"
        else
            cp -a "$ROOT/$path" "$wt/$path"
        fi
    done

    [ "$branch" = "docker" ] && write_docker_extras "$wt"

    git -C "$wt" add -A
    if git -C "$wt" diff --cached --quiet; then
        echo "    no change"
    elif [ "$DRY_RUN" = 1 ]; then
        echo "    would commit:"
        git -C "$wt" diff --cached --stat | tail -3 | sed 's/^/      /'
        git -C "$wt" reset -q
    else
        git -C "$wt" commit -q -m "Sync $branch from main ($(git rev-parse --short HEAD))"
        git -C "$wt" push -q -u origin "$branch"
        echo "    pushed $(git -C "$wt" rev-parse --short HEAD)"
    fi

    git worktree remove --force "$wt"
}

require_clean_main
mkdir -p "$WORKTREES"
for b in "${BRANCHES[@]}"; do publish_one "$b"; done
git worktree prune
echo "done"

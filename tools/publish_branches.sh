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
#   docker     the workspace subset + Dockerfile/compose     (no ROS install)
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
        docker)    echo "gps_localize_ws webapp tools setup docs .gitignore README.md" ;;
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

# micro-ROS agent is not packaged for Humble, so it is built from source
# against the Fast DDS that ships with ROS. Building it here keeps wire
# compatibility with the ROS 2 graph inside the same image.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git python3-pip curl \
        ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN git clone -b v2.4.2 --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/agent \
    && cmake -S /tmp/agent -B /tmp/agent/build \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DUAGENT_BUILD_EXECUTABLE=ON \
        -DUAGENT_USE_SYSTEM_FASTDDS=ON \
        -DUAGENT_USE_SYSTEM_FASTCDR=ON \
        -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/agent/build --target install -j"$(nproc)" \
    && rm -rf /tmp/agent

WORKDIR /ws
COPY gps_localize_ws/src ./src
RUN . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths src --ignore-src -y \
    && colcon build --symlink-install

COPY docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

ENV ROS_DOMAIN_ID=10
EXPOSE 8080 8888/udp
ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["ros2", "launch", "gps_localize", "bringup.launch.py"]
DOCKERFILE

    cat > "$wt/docker-entrypoint.sh" <<'ENTRYPOINT'
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
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

    healthcheck:
      test: ["CMD", "curl", "-fsS", "http://localhost:8080/"]
      interval: 30s
      timeout: 5s
      retries: 3
      start_period: 20s

volumes:
  gps_localize_data:
COMPOSE

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

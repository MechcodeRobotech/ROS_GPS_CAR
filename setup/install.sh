#!/usr/bin/env bash
# GPS_Localize - one file, installs everything.
#
#   ./setup/install.sh                  pick the best mode for this machine
#   ./setup/install.sh --docker         containers, no ROS on the host
#   ./setup/install.sh --native         ROS 2 Humble installed on the host
#   ./setup/install.sh --dry-run        print every command, change nothing
#   ./setup/install.sh --check          report what is installed, change nothing
#   ./setup/install.sh --uninstall      remove the service and the ROS build
#
# Or straight from GitHub:
#   curl -fsSL https://raw.githubusercontent.com/MechcodeRobotech/ROS_GPS_CAR/workspace/setup/install.sh | bash
#
# WHY --dry-run EXISTS
# Most of this needs root, and a script that can only be exercised by running
# it for real as root is a script nobody checks until it breaks on someone's
# machine. --dry-run walks the identical decision tree and prints exactly what
# would run, so the logic is testable without touching the system. --check
# does the same for the detection half.
#
# Safe to run repeatedly: every step is skipped when it is already done.
set -uo pipefail

REPO_URL="https://github.com/MechcodeRobotech/ROS_GPS_CAR.git"
ROS_DISTRO_WANT="humble"
UBUNTU_WANT="22.04"
MODE="auto"
DRY_RUN=0
CHECK_ONLY=0
UNINSTALL=0
ASSUME_YES=0

for arg in "$@"; do
    case "$arg" in
        --docker)    MODE="docker" ;;
        --native)    MODE="native" ;;
        --dry-run)   DRY_RUN=1 ;;
        --check)     CHECK_ONLY=1 ;;
        --uninstall) UNINSTALL=1 ;;
        -y|--yes)    ASSUME_YES=1 ;;
        -h|--help)   sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $arg  (try --help)" >&2; exit 2 ;;
    esac
done

# ---------------------------------------------------------------- output ----
if [ -t 1 ]; then
    C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_BAD=$'\033[31m'
    C_DIM=$'\033[2m'; C_HEAD=$'\033[36m'; C_OFF=$'\033[0m'
else
    C_OK=; C_WARN=; C_BAD=; C_DIM=; C_HEAD=; C_OFF=
fi
say()  { printf '  %s\n' "$*"; }
ok()   { printf '  %sOK%s    %s\n'   "$C_OK"   "$C_OFF" "$*"; }
warn() { printf '  %swarn%s  %s\n'   "$C_WARN" "$C_OFF" "$*"; }
bad()  { printf '  %sFAIL%s  %s\n'   "$C_BAD"  "$C_OFF" "$*"; }
head() { printf '\n%s[%s]%s\n' "$C_HEAD" "$*" "$C_OFF"; }

FAILED=0
fail() { bad "$*"; FAILED=1; }

# Run a command, or print it under --dry-run. Everything that changes the
# system goes through here, which is what makes the dry run trustworthy.
run() {
    if [ "$DRY_RUN" = 1 ]; then
        printf '  %swould run:%s %s\n' "$C_DIM" "$C_OFF" "$*"
        return 0
    fi
    "$@"
}

need_root() {
    if [ "$(id -u)" = 0 ]; then echo ""; return; fi
    if command -v sudo >/dev/null 2>&1; then echo "sudo"; return; fi
    echo ""
}
SUDO="$(need_root)"

# --------------------------------------------------------------- detect -----
detect() {
    head "this machine"
    OS_ID=""; OS_VER=""
    if [ -r /etc/os-release ]; then
        # shellcheck disable=SC1091
        . /etc/os-release
        OS_ID="${ID:-}"; OS_VER="${VERSION_ID:-}"
    fi
    say "os          : ${PRETTY_NAME:-unknown}"
    say "arch        : $(uname -m)"
    say "user        : $(id -un)"

    HAVE_ROS=0
    if [ -d "/opt/ros/$ROS_DISTRO_WANT" ]; then HAVE_ROS=1; ok "ROS 2 $ROS_DISTRO_WANT present"
    else say "ROS 2 $ROS_DISTRO_WANT : not installed"; fi

    HAVE_DOCKER=0
    if command -v docker >/dev/null 2>&1; then
        HAVE_DOCKER=1
        if docker info >/dev/null 2>&1; then ok "docker present and running"
        else warn "docker present but not running (or you are not in the docker group)"; fi
    else
        say "docker      : not installed"
    fi

    HAVE_PIO=0
    command -v pio >/dev/null 2>&1 && { HAVE_PIO=1; ok "platformio present"; } || \
        say "platformio  : not installed (only needed to build firmware)"

    CAN_ROOT=0
    if [ "$(id -u)" = 0 ]; then CAN_ROOT=1
    elif [ -n "$SUDO" ] && sudo -n true 2>/dev/null; then CAN_ROOT=1; ok "passwordless sudo"
    elif [ -n "$SUDO" ]; then CAN_ROOT=2; say "sudo        : available, will ask for your password"
    fi
    [ "$CAN_ROOT" = 0 ] && warn "no way to become root - only --check and --dry-run will work"

    if [ "$OS_ID" != "ubuntu" ]; then
        warn "this installer targets Ubuntu $UBUNTU_WANT; on $OS_ID use --docker"
    elif [ "$OS_VER" != "$UBUNTU_WANT" ]; then
        warn "Ubuntu $OS_VER, not $UBUNTU_WANT - ROS 2 $ROS_DISTRO_WANT expects $UBUNTU_WANT; --docker avoids that"
    fi
}

choose_mode() {
    [ "$MODE" != "auto" ] && return
    if [ "$HAVE_ROS" = 1 ]; then MODE="native"
    elif [ "$HAVE_DOCKER" = 1 ]; then MODE="docker"
    elif [ "$OS_ID" = "ubuntu" ] && [ "$OS_VER" = "$UBUNTU_WANT" ]; then MODE="native"
    else MODE="docker"; fi
    say "chosen mode : $MODE"
}

# ---------------------------------------------------------------- native ----
install_native() {
    head "ROS 2 $ROS_DISTRO_WANT"
    if [ "$HAVE_ROS" = 1 ]; then
        ok "already installed, nothing to do"
    else
        say "installing ros-$ROS_DISTRO_WANT-ros-base"
        run $SUDO apt-get update -qq
        run $SUDO apt-get install -y -qq software-properties-common curl gnupg lsb-release
        run $SUDO add-apt-repository -y universe
        run bash -c "curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            | $SUDO gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg"
        run bash -c "echo \"deb [arch=\$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main\" \
            | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null"
        run $SUDO apt-get update -qq
        run $SUDO apt-get install -y ros-$ROS_DISTRO_WANT-ros-base \
            python3-colcon-common-extensions python3-rosdep python3-pip
    fi

    head "micro-ROS agent"
    if command -v MicroXRCEAgent >/dev/null 2>&1 || \
       [ -x "$HOME/.local/bin/MicroXRCEAgent" ] || \
       [ -d "$HOME/uros_ws/install/micro_ros_agent" ]; then
        ok "already available"
    else
        # Not packaged for Humble, so it is built against the Fast DDS that
        # ships with ROS - that is what keeps it wire-compatible with the graph.
        say "building Micro-XRCE-DDS-Agent from source (no package exists for $ROS_DISTRO_WANT)"
        run $SUDO apt-get install -y -qq build-essential cmake git
        run rm -rf /tmp/uros_agent
        run git clone -b v2.4.2 --depth 1 \
            https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/uros_agent
        run bash -c "source /opt/ros/$ROS_DISTRO_WANT/setup.bash && \
            cmake -S /tmp/uros_agent -B /tmp/uros_agent/build \
              -DCMAKE_INSTALL_PREFIX=\$HOME/.local \
              -DUAGENT_BUILD_EXECUTABLE=ON -DUAGENT_USE_SYSTEM_FASTDDS=ON \
              -DUAGENT_USE_SYSTEM_FASTCDR=ON \
              -DUAGENT_SUPERBUILD=OFF \
              -DCMAKE_BUILD_TYPE=Release && \
            cmake --build /tmp/uros_agent/build --target install -j\$(nproc)"
    fi

    head "workspace"
    if [ ! -d "$WS/src" ]; then
        fail "no workspace at $WS - run this from inside the project, or clone it first"
        return
    fi
    run bash -c "source /opt/ros/$ROS_DISTRO_WANT/setup.bash && \
        cd '$WS' && rosdep install --from-paths src --ignore-src -y 2>/dev/null || true"
    run bash -c "source /opt/ros/$ROS_DISTRO_WANT/setup.bash && \
        cd '$WS' && colcon build --symlink-install --parallel-workers \$(nproc)"
    ok "workspace built"

    head "service"
    local unit="$WS/src/gps_localize/systemd/gps_localize.service"
    if [ -f "$unit" ]; then
        run $SUDO cp "$unit" /etc/systemd/system/gps_localize.service
        run $SUDO systemctl daemon-reload
        run $SUDO systemctl enable --now gps_localize
        ok "gps_localize service enabled - it will start on boot"
    else
        warn "no systemd unit found, skipping autostart"
    fi
}

# ---------------------------------------------------------------- docker ----
install_docker() {
    head "docker"
    if [ "$HAVE_DOCKER" = 1 ]; then
        ok "already installed"
    else
        say "installing docker"
        run bash -c "curl -fsSL https://get.docker.com | $SUDO sh"
        run $SUDO usermod -aG docker "$(id -un)"
        warn "you were added to the docker group - log out and back in for it to apply"
    fi

    head "stack"
    if [ ! -f "$ROOT/docker-compose.yml" ]; then
        fail "no docker-compose.yml here - this is not the docker branch"
        say  "clone it:  git clone --branch docker $REPO_URL"
        return
    fi
    local files=(-f "$ROOT/docker-compose.yml")
    if [ "$(uname -s)" != "Linux" ] && [ -f "$ROOT/docker-compose.windows.yml" ]; then
        files+=(-f "$ROOT/docker-compose.windows.yml")
        say "not Linux: adding the override, host networking does not exist here"
    fi
    run docker compose "${files[@]}" up -d --build
    ok "stack started"
}

# ------------------------------------------------------------- uninstall ----
do_uninstall() {
    head "removing"
    if systemctl list-unit-files 2>/dev/null | grep -q '^gps_localize'; then
        run $SUDO systemctl disable --now gps_localize
        run $SUDO rm -f /etc/systemd/system/gps_localize.service
        run $SUDO systemctl daemon-reload
        ok "service removed"
    else
        say "service not installed"
    fi
    if [ -f "$ROOT/docker-compose.yml" ] && command -v docker >/dev/null 2>&1; then
        run docker compose -f "$ROOT/docker-compose.yml" down
        ok "containers stopped"
    fi
    [ -d "$WS/build" ] && { run rm -rf "$WS/build" "$WS/install" "$WS/log"; ok "ROS build removed"; }
    say "ROS 2 and Docker themselves are left alone - remove them with apt if you want them gone"
}

# ------------------------------------------------------------------ main ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WS="$ROOT/gps_localize_ws"

printf '\n%sGPS_Localize installer%s\n' "$C_HEAD" "$C_OFF"
printf '%s\n' "============================================================"
[ "$DRY_RUN" = 1 ] && warn "dry run: nothing will be changed"

detect

if [ "$CHECK_ONLY" = 1 ]; then
    head "result"
    say "nothing was changed (--check)"
    exit 0
fi

if [ "$UNINSTALL" = 1 ]; then
    do_uninstall
    exit $FAILED
fi

if [ "$CAN_ROOT" = 0 ] && [ "$DRY_RUN" = 0 ]; then
    head "cannot continue"
    fail "installing needs root, and neither root nor sudo is usable here"
    say  "run it on a normal terminal, or use --dry-run to see what it would do"
    exit 1
fi

choose_mode
case "$MODE" in
    native) install_native ;;
    docker) install_docker ;;
    *) fail "unknown mode: $MODE"; exit 2 ;;
esac

head "done"
if [ "$FAILED" = 0 ]; then
    IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
    ok "install finished"
    say ""
    say "web UI : http://${IP:-localhost}:8080"
    [ "$MODE" = docker ] && say "topics : docker compose exec gps_localize ros2 topic list"
    [ "$MODE" = native ] && say "topics : ros2 topic list      (ROS_DOMAIN_ID=10)"
else
    bad "something failed above"
fi
exit $FAILED

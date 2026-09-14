#!/bin/bash
# One-command Docker install. No ROS 2 on the host.
#
#   NOTE: the repository is private, so this exact line returns 404 until it is
#   made public. Clone with `gh repo clone` or pass a token - see setup/install.sh.
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

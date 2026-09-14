#!/usr/bin/env bash
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
# Build the standalone Linux viewer: webapp/bin/GPS_Localize_Viewer
#
# Optional - on Ubuntu, run_ubuntu.sh already works with the system python3.
# A binary must be built ON Linux; you cannot cross-build it from Windows.
set -e
cd "$(dirname "$0")"

PKG=../gps_localize_ws/src/gps_localize

python3 -m pip install --upgrade pyinstaller pyyaml

python3 -m PyInstaller \
  --noconfirm --clean --onefile \
  --name GPS_Localize_Viewer \
  --distpath bin --workpath build --specpath build \
  --paths "$PKG" \
  --add-data "$PKG/web:web" \
  --add-data "$PKG/config:config" \
  --hidden-import gps_localize.configstore \
  --hidden-import gps_localize.httpd \
  --hidden-import yaml \
  viewer_server.py

chmod +x bin/GPS_Localize_Viewer
echo
echo "  Built bin/GPS_Localize_Viewer - copy the whole webapp folder to any Ubuntu PC."

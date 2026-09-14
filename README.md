# GPS_Localize — Docker robot server

Run the micro-ROS agent, ROS 2 Humble navigation/safety nodes, web interface,
and firmware build tools without installing ROS on the host.

Start with the [complete Docker installation and operations guide](docs/docker.md).
It covers Linux, Docker Desktop on Windows over Wi-Fi, and Windows USB through
WSL2 and a Docker Engine installed inside that same Ubuntu distribution.
The [documentation index](docs/README.md) links hardware, calibration, safety,
commands, troubleshooting, and verification results.

Clone this branch with Git first (repository access is required):

```bash
git clone --branch docker https://github.com/MechcodeRobotech/ROS_GPS_CAR.git GPS_Localize
cd GPS_Localize
```

Linux:

```bash
docker compose up -d --build
```

Docker Desktop on Windows (Linux containers, Wi-Fi transport):

```powershell
docker compose -f docker-compose.yml -f docker-compose.windows.yml up -d --build
```

Open http://localhost:8080. On Windows, run `setup/setup_windows.ps1` as
Administrator to configure the documented private-network firewall rules.
Point the ESP32 at the PC LAN address, with UDP port 8888 reachable.

USB requires a device visible to the Docker daemon. A Windows COM port is not
a Linux device. Follow [Windows USB setup](docs/docker.md#windows-usb-through-wsl2)
before using the serial overrides. USB flashing and USB runtime transport are
separate choices: `wifi` firmware still communicates by Wi-Fi after a USB flash;
`serial` firmware communicates through the cable at 115200 baud.

Inspect ROS using the entrypoint explicitly because `docker exec` bypasses it:

```bash
docker compose exec gps_localize /usr/local/bin/docker-entrypoint.sh ros2 node list
```

The default Compose file exposes no USB device, so the server can start without
a board. An HTTP healthcheck confirms the web service only; verify the firmware
node and fresh telemetry before calling the robot connected. Runtime settings
and PlatformIO caches persist in named volumes. Stop with `docker compose down`;
do not add `-v` unless you intend to erase saved settings and caches.


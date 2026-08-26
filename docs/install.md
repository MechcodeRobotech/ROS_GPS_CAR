# Install and run — Ubuntu 22.04 + ROS 2 Humble

> **Shortcut:** `setup/setup_and_run.sh` does sections 1 to 6 of this page by
> itself — it checks the machine, installs ROS 2 and the dependencies if they
> are missing, builds the workspace, opens the firewall and starts everything.
> The rest of this page is what it does, in case you want to do it by hand or
> something goes wrong. See [setup/README.md](../setup/README.md).

## 1. ROS 2 Humble and the micro-ROS agent

```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions \
                    ros-humble-micro-ros-msgs python3-yaml git
# micro-ROS agent (binary package on Humble)
sudo apt install -y ros-humble-micro-ros-agent
```

If your apt mirror has no `ros-humble-micro-ros-agent`, build it once:

```bash
mkdir -p ~/uros_ws/src && cd ~/uros_ws
git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git src/micro_ros_agent
rosdep install --from-paths src --ignore-src -y
colcon build && echo 'source ~/uros_ws/install/setup.bash' >> ~/.bashrc
```

Nothing else is needed: the web UI is pure Python standard library, no pip, no
internet at run time.

## 2. Build this workspace

```bash
# copy the repository to the robot PC, then
cd ~/gps_localize_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` matters: with it, editing `web/*.html`, `web/*.js` or the
YAML defaults takes effect on the next start without rebuilding.

## 3. Network

The ESP32 talks to the agent over Wi-Fi (UDP 8888).

```bash
hostname -I          # note this address
sudo ufw allow 8888/udp     # only if the firewall is enabled
sudo ufw allow 8080/tcp     # web UI
```

### Find the agent by name, not by IP

If the router hands out a different address after a reboot, a hard-coded IP
breaks the robot. The firmware can look the PC up by name instead:

```bash
sudo apt install -y avahi-daemon        # publishes <hostname>.local on the LAN
sudo hostnamectl set-hostname ros2-agent
systemctl restart avahi-daemon
avahi-resolve -n ros2-agent.local       # sanity check
```

Then in `config/network.h` keep `USE_AGENT_HOSTNAME 1` and set
`AGENT_HOSTNAME = "ros2-agent"`. The ESP32 tries mDNS first, then plain DNS,
then the fixed `AGENT_IP` as a fallback — and it repeats the lookup while
waiting for the agent, so it follows the PC to a new address without a reflash.

Put the fallback address in `firmware/config/network.h`:

```c
static const char* WIFI_SSID = "your-ssid";
static const char* WIFI_PASS = "your-password";
#define USE_AGENT_HOSTNAME 1
static const char*     AGENT_HOSTNAME = "ros2-agent";  // preferred: found by name
static const IPAddress AGENT_IP(192, 168, 50, 242);    // fallback only
static const uint16_t  AGENT_PORT = 8888;
#define ROS_DOMAIN_ID 10
```

`ROS_DOMAIN_ID` must be the same on both sides. The launch file sets it to 10.

## 4. Flash the firmware

```bash
sudo apt install -y python3-pip && pip install -U platformio
source /opt/ros/humble/setup.bash        # micro_ros_platformio reads $ROS_DISTRO
cd firmware
pio run -e wifi -t upload                # Wi-Fi transport (normal)
pio device monitor -b 115200             # check the boot messages
```

Expected boot output:

```
[GPS] listening at 115200 baud
```

plus a line for any INA226 or the BNO085 that is missing. A clean board prints
nothing else — that is normal, the ESP32 only talks over micro-ROS after that.

Serial transport fallback (no Wi-Fi):

```bash
pio run -e serial -t upload
ros2 launch gps_localize bringup.launch.py transport:=serial serial_device:=/dev/ttyUSB0
```

## 5. Run

```bash
source ~/gps_localize_ws/install/setup.bash
ros2 launch gps_localize bringup.launch.py
```

Then open `http://<robot-ip>:8080` from a phone or laptop on the same network.

First start creates `~/.gps_localize/` with a copy of every settings file.
That directory is the live configuration; the web UI writes there.

Check that the firmware really connected:

```bash
ros2 node list                      # /gps_localize_firmware must appear
ros2 topic hz /gps_localize/telemetry   # ~10 Hz
```

## 6. One server, many devices

The robot PC is the server. Everyone else only needs a browser — no app, no
install, no internet.

```bash
hostname -I | awk '{print $1}'     # e.g. 192.168.1.50 -> tell the operators this
sudo ufw allow 8080/tcp            # only if the firewall is on
```

`config/web.yaml` must keep `host: 0.0.0.0` (the default) so the page is
reachable from the network; `127.0.0.1` would make it visible only on the robot
PC itself. `max_clients` (default 8) limits how many devices can stream at once.

| Device | What to do |
|---|---|
| Phone / tablet | connect to the same Wi-Fi, open `http://192.168.1.50:8080` |
| Laptop | same URL in any browser |
| Windows PC with no ROS | `webapp/run_windows.bat` → view only, cannot drive |

All connected browsers see the same live data, and any of them can drive.
Keep that in mind on site: agree who is the operator, because the robot obeys
whoever presses a button. Every one of them can also hit the emergency stop.

**No router on site?** Turn the robot PC into a hotspot and let the phones join
it — the ESP32 must then join the same hotspot (put its SSID/password in
`config/network.h`):

```bash
nmcli device wifi hotspot ifname wlan0 ssid GPS_Localize password robot12345
nmcli connection show --active          # confirm, then use the PC address above
```

### Desktop viewer (no ROS)

`webapp/` contains the same interface running without ROS, for looking at the
UI and preparing settings/waypoints on any Windows or Ubuntu machine:

```bat
webapp\run_windows.bat                 :: Windows, uses exe\GPS_Localize_Viewer.exe
```
```bash
webapp/run_ubuntu.sh                   # Ubuntu, uses system python3
```

It shows a permanent "VIEW ONLY" bar, pops a warning on start, disables every
control button and answers `/api/nav`, `/api/manual` and `/api/estop` with HTTP
403. Files it saves land in `webapp/exe/settings/` — copy them to
`~/.gps_localize/` on the robot PC to use them for real. Details:
[../webapp/README.md](../webapp/README.md).

## 7. Autostart on boot

```bash
sudo cp ~/gps_localize_ws/src/gps_localize/systemd/gps_localize.service /etc/systemd/system/
sudo nano /etc/systemd/system/gps_localize.service   # set User= and the paths
sudo systemctl daemon-reload
sudo systemctl enable --now gps_localize
journalctl -u gps_localize -f
```

## Troubleshooting

| Symptom | Check |
|---|---|
| `ros2 node list` shows no firmware node | agent running? same `ROS_DOMAIN_ID`? UDP 8888 open? serial monitor shows which address the ESP32 resolved |
| ESP32 prints `name lookup failed` | `avahi-daemon` not installed/running on the PC, or the Wi-Fi AP blocks multicast — use the `AGENT_IP` fallback or a DHCP reservation |
| Web page loads but every value is `—` | `ros2 topic hz /gps_localize/telemetry`; if empty the ESP32 is not connected |
| Red LED blinks fast all the time | heartbeat or command timeout — is `safety_watchdog` running, is the browser open? |
| Red LED solid | emergency stop latched: mushroom switch, on-board button, or web e-stop (press RESET) |
| Robot refuses to start a mission | GPS gate: needs `min_satellites` and `max_hdop` from `nav.yaml`; the reason is shown on the dashboard |
| GPS never gets a fix | outdoors with sky view, wait 1–2 min for the first fix; check the antenna connector |
| Heading drifts / robot circles | see [tuning.md](tuning.md) — IMU mounting signs first, then the heading PID |
| Web UI unreachable from a phone | `web.host` must be `0.0.0.0`, port 8080 open in the firewall, same subnet |
| `Address already in use` on start | another copy is running: `systemctl stop gps_localize` or change `web.port` |

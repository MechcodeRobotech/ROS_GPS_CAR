# Safety chain

Five layers. Each one stops the robot on its own — no layer needs another one
to work. Read this before the first outdoor test.

| # | Layer | Trigger | Reaction | Recovery |
|---|---|---|---|---|
| 1 | Hardware relay | mushroom switch SW3 | motor power (EMER VCC) physically cut | release the switch |
| 2 | Firmware inputs | `E-EMER` (IO13) from the mushroom switch | PWM off, contactor opens, red LED solid | clear the input, wait 400 ms |
| 3 | Link watchdog | no `/gps_localize/heartbeat` for 600 ms | motors braked, red LED fast blink | heartbeat returns |
| 4 | Command watchdog | no `cmd_move`/`cmd_manual` for 500 ms while moving | motors braked, targets zeroed | send a new command |
| 5 | Web heartbeat | browser silent for `web_heartbeat_timeout_s` | `safety_watchdog` stops the heartbeat → layer 3 fires, navigation paused | reconnect **and** press START/RESUME |

Plus two conditions that latch inside the firmware:

* **Battery cutoff** — pack under 10.7 V for 3 s (`BATT_CUTOFF_V`), and a non-latching soft stop at 11.0 V (`BATT_SOFT_STOP_V`), red LED slow
  blink. Clears when the pack recovers above 10.3 V.
* **Startup lockout** — no motion during the first 1.5 s after boot.

## Software emergency stop

The web button publishes `/gps_localize/estop_request` → `safety_watchdog`
latches it → `/gps_localize/estop` → firmware. It is **latched**: releasing the
button does nothing, the operator must press RESET (`safety.auto_clear_estop`
is false and should stay false).

While an e-stop is latched the heartbeat keeps running on purpose, so the red
LED stays *solid* (operator stopped me) instead of *blinking* (I lost the
network). That difference tells a person standing next to the robot what
actually happened.

## Red LED patterns

| Pattern | Meaning |
|---|---|
| Solid | emergency stop latched (hardware, button or web) |
| Fast blink (~4 Hz) | link lost, command timeout, or agent gone |
| Slow blink (~1 Hz) | battery cutoff |
| Short blip every 1.5 s | healthy |

The green LED is wired straight to 5 V after the ON MODULE switch. It says
"the module has power", and because no GPIO drives it, it keeps working even
if the firmware crashes.

## Verifying the chain (do this after any wiring change)

```bash
# 1. link layer: kill the watchdog, robot must stop within ~1 s
ros2 lifecycle  # (not used) -- simply: pkill -f safety_watchdog

# 2. command layer: publish one move command and stop publishing
ros2 topic pub -1 /gps_localize/cmd_move geometry_msgs/msg/Twist '{linear: {x: 0.15}, angular: {z: 0.0}}'
#    the robot moves briefly and stops after ~0.5 s

# 3. software e-stop
ros2 topic pub -1 /gps_localize/estop std_msgs/msg/Bool '{data: true}'

# 4. web layer: close the browser tab while the robot drives
# 5. hardware layer: press the mushroom switch while the robot drives
```

Expected `stop_flags` on the dashboard for each test: `heartbeat_lost`,
`command_timeout`, `software_estop`, `heartbeat_lost`, `hardware_emergency`.

## Limits you should know about

* The robot has **no obstacle sensor**. Navigation only knows GPS position and
  heading; it will drive into anything in the way. Keep the operator in sight
  of the robot and the e-stop within reach.
* GPS accuracy without RTK is ±2–3 m. `arrive_radius_m` below ~0.6 m makes the
  robot hunt around the point instead of settling.
* Heading from `heading_source: 1` needs movement to align with the GPS course.
  Right after power-on and before the first metres are driven, the absolute
  heading may be off; the first turn of a mission corrects it.
* Wheel speed on the web UI is derived from IMU + GPS, not measured. Treat it
  as an indication, not as a diagnostic for a slipping wheel.
* The relay cuts the motor rails only. The ESP32, GPS and IMU stay powered
  during an emergency stop (that is what lets the UI keep reporting).

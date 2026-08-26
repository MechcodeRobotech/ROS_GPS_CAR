# Hardware — schematic draft_5

Reference drawing: `SCH_Schematic1_5-draft_5_2026-08-04.pdf` (page `draft_5`).
Still no wheel encoders — GPS + IMU only.

Changes from draft_2, all of them already in the firmware:

| Change | Effect on the code |
|---|---|
| **4th INA226** on the fan rail (`FAN_SENSOR`) | new address `0x45`, published as `fan_v` / `fan_a` |
| **Two LM2596 bucks**: one for the fans, one for the processor | fans can no longer brown out the ESP32 (this was the fix requested) |
| **4 fans** wired individually (`MOTOR_FAN_COOLING_A..D`) | still uncontrolled, always on |
| **Series diode moved**: it now sits *after* the INA226, right before the TB6612 `VM` | the sensor reads the rail **before** the diode, so the motor model subtracts `Vf` — per motor, they can be different parts |
| Component designators renumbered (C1…, R1…) | no effect |

ESP32 pin map, GPS and IMU are unchanged from draft_2.

## ESP32-DevKitC pin map

| Pin | Net (schematic) | Direction | Used for |
|---|---|---|---|
| IO25 | MOTOR A IN1 | out | Motor A (LEFT) direction |
| IO26 | MOTOR A IN2 | out | Motor A direction |
| IO27 | MOTOR A EN | out (PWM) | Motor A speed, TB6612 PWMA+PWMB |
| IO19 | MOTOR B IN1 | out | Motor B (RIGHT) direction |
| IO18 | MOTOR B IN2 | out | Motor B direction |
| IO4 | MOTOR B EN | out (PWM) | Motor B speed |
| IO21 | SDA | I²C | BNO085, 4× INA226, QMC5883L in the GPS module |
| IO22 | SCL | I²C | 3.3 k pull-ups on the board |
| IO16 | GPS RX (net) | in | ESP RX ← GPS TX |
| IO17 | GPS TX (net) | out | ESP TX → GPS RX |
| IO14 | E-EMER | in | emergency relay line sense |
| IO13 | ESP CHECK EMER | in | on-board e-stop button (10 k pull-up, LOW = pressed) |
| IO34 | MOTOR A ALER | in | INA226 alert (input-only pin) |
| IO35 | MOTOR B ALER | in | INA226 alert |
| IO32 | PROCESSOR ALER | in | INA226 alert |
| IO2 | LED indicator | out | **RED** LED through 330 Ω |
| IO33, IO23, IO5, IO12, IO15, IO0 | — | — | free |

Firmware copy of this table: `firmware/config/pins.h`.

## Motor drivers

Each TB6612FNG has **both half-bridges wired in parallel** (AIN1+BIN1,
AIN2+BIN2, PWMA+PWMB, A01+B01, A02+B02) so one board drives one motor at
double the current. `STBY` is tied to +3.3 V. The firmware therefore only
drives 3 pins per motor.

* Motor A = LEFT wheel, Motor B = RIGHT wheel.
* If a wheel turns the wrong way, flip `MOTOR_A_INVERT` / `MOTOR_B_INVERT`
  in `firmware/config/motor.h` (no rewiring needed).
* PWM: 20 kHz, 10 bit, ESP32 LEDC channels 0 and 1.

## Power sensing — 4× INA226

```
VCC BATT ─ switch ─ VCC ON MODULE ─┬─[FAN_SENSOR]───── VCC FAN ───── LM2596 ─ +5V FAN ─ diode ─ 4× fan
                                   └─[PROCESSOR_SENSOR] VCC PROCESSOR LM2596 ─ +5V ─ ESP32

VCC BATT ─ fuse ─ mushroom ─ relay ─ VCC EMER ─┬─[MOTOR_A_SENSOR]─ VCC MOTOR A ─▶|─ TB6612 VM
                                               └─[MOTOR_B_SENSOR]─ VCC MOTOR B ─▶|─ TB6612 VM
```

| Sensor | Measures | Default address | Shunt |
|---|---|---|---|
| PROCESSOR_SENSOR | VCC ON MODULE → processor buck (= pack voltage) | 0x40 | 0.1 Ω / 0.8 A |
| MOTOR_A_SENSOR | VCC EMER → motor A rail, **before the diode** | 0x41 | 0.01 Ω / 8 A |
| MOTOR_B_SENSOR | VCC EMER → motor B rail, **before the diode** | 0x44 | 0.01 Ω / 8 A |
| FAN_SENSOR | VCC ON MODULE → fan buck (draft_5, new) | 0x45 | 0.1 Ω / 1.5 A |

INA226 address jumpers (A1,A0): GND,GND `0x40` · GND,VS `0x41` · VS,GND `0x44` ·
VS,VS `0x45`. `FAN_SENSOR`'s ALERT pin is not wired — the firmware does not use
the alert outputs, it polls all four sensors at 5 Hz.

### The series diode (`MOTOR_A_DIODE` / `MOTOR_B_DIODE`)

Each motor rail has a diode between its sensor and the TB6612 `VM` pin, with
the 1000 µF + 0.1 µF bulk caps after it. The sensor therefore reads the voltage
**before** the diode and the motor only ever gets

```
V_driver = V_sensor − (Vf + I × R_slope)
```

The back-EMF wheel-speed model subtracts exactly that, so `Vf` must match the
part you fitted — and the two motors have separate settings because they are
separate diodes:

```c
// firmware/config/motor.h        (or Settings → มอเตอร์ on the web UI)
#define MOTOR_A_DIODE_VF_V      1.81f   // MEASURED on this board, left
#define MOTOR_A_DIODE_R_OHM     0.03f   // slope, 0 = flat drop
#define MOTOR_B_DIODE_VF_V      1.90f   // MEASURED on this board, right
#define MOTOR_B_DIODE_R_OHM     0.03f
```

**Set the A0/A1 jumpers before power-up** or change `INA226_ADDR_*` in
`power.h`. Stock CJMCU-226 boards ship with a 0.1 Ω shunt (max 0.8 A) — fine
for the processor and fan rails, too small for the motors. Fit 0.01 Ω on the
two motor sensors and keep `INA226_SHUNT_MOTOR_OHM` in sync.

Battery protection lives in `power.h`, as 3S LiPo tiers: warn at 11.4 V
(3.80 V/cell), soft stop at 11.0 V, hard cut at 10.7 V held for 3 s, and
recovery above 11.1 V. A cell is damaged below about 3.0 V/cell (9.0 V pack),
so these sit well clear of it.

The diode drops above are measured, not datasheet figures - at ~12 V a 1.8 V
drop is 15% of the rail, and the back-EMF wheel-speed model uses it, so a
guess makes the derived speed read high whenever GPS is unavailable.

Check the addresses on the bench:

```bash
# ESP32 serial monitor prints "[INA226] ... missing" for each sensor it cannot see
pio device monitor -b 115200
```

## Board notes

### 1. Two LEDs instead of one

This is the one item the drawing still shows as a single LED:

| LED | Wiring | Meaning |
|---|---|---|
| **RED** | IO2 → 330 Ω → LED → GND (as drawn) | solid = emergency stop / E-emergency latched · fast blink = link or command lost · slow blink = battery cutoff · short blip every 1.5 s = healthy |
| **GREEN** | +5 V (after the ON MODULE switch) → 330 Ω → LED → GND, **no GPIO** | module is powered / switch is ON. Deliberately not driven by firmware so it still lights if the ESP32 hangs |

### 2. Cooling fans — done in draft_5

The 4 fans now run from their own LM2596 (`FAN_BUCK`) fed through `FAN_SENSOR`,
separate from the processor buck, so fan inrush cannot brown out the ESP32.
`FAN_DIODE` sits on the buck output.

The fans run continuously — the firmware neither switches nor needs to switch
them, it only reports the rail on the dashboard. If you later add a MOSFET, set
`PIN_FAN_CTRL` in `pins.h` to that GPIO (IO33 is free).

## Emergency stop wiring

```
VCC BATT ─ 10 A fuse ─ SW3 (mushroom, latching) ─ relay coil ─┐
                                                              ├─► EMER VCC ──► motor rails
                                                E-EMER sense ─┘   (INA226 motor A/B measure here)
```

* `E-EMER` (IO14) tells the firmware whether the emergency loop is closed.
  `E_EMER_ACTIVE_LOW` in `safety.h` is set to 1 — **verify the real
  polarity on the bench** with a multimeter before the first drive test and
  change the define if the LED lights the wrong way round.
* `SW4` (IO13) is the small on-board e-stop button, active LOW.
* Because the motor rails sit behind the relay, `MOTOR_A_SENSOR`/`MOTOR_B_SENSOR`
  read ~0 V when the emergency loop is open — a second, electrical proof that
  power really was cut.

## Mechanical / calibration constants

| Constant | Where | Default | Note |
|---|---|---|---|
| Wheel diameter | `WHEEL_DIAMETER_M`, `robot.yaml` | 0.081 m | 81 mm wheels |
| Wheel track | `WHEEL_TRACK_M`, `robot.yaml`, `pid.yaml:estimator.track_m` | 0.300 m | **measure your chassis and update all three** |
| Max speed | `ROBOT_MAX_SPEED_MPS` | ≈0.42 m/s | derived from 100 rpm × Ø81 mm |
| IMU mounting | `IMU_ACC_FORWARD_AXIS`, `IMU_YAW_TO_COMPASS_SIGN` | X forward, chip up | see docs/tuning.md |

## GPS module and its compass

GEP-M10-DQ: UART for position, and the **QMC5883L magnetometer inside the
module is used** (I²C address `0x0D`, on the same bus as everything else).

That compass is what gives the robot an absolute heading while it is standing
still — the GPS course over ground only exists once it is moving, and the
BNO085 game rotation vector is relative. It sits on the antenna mast, away from
the motors, which makes it the better of the two magnetometers on the robot.
The readings are tilt-compensated with roll/pitch from the BNO085 and rejected
when the field strength jumps (motor current, steel fence): see
`firmware/lib/compass/compass_qmc5883.h` and calibrate once with the web UI
button (docs/tuning.md §2b). The calibration is stored in the ESP32 flash.

Set `USE_QMC5883L 0` in `compass.h` if your module has no compass — the
firmware then falls back to the BNO085 magnetometer and the GPS course.

The DPS310 barometer in the module is not used.

Baud rate is auto-detected at boot (115200 → 38400 → 9600 → 57600). For the
best results configure the module once with u-center: 5 Hz navigation rate,
GGA + RMC + VTG enabled, GSV/GSA/GLL disabled.

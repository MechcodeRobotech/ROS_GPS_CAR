/**
 * @file pins.h
 * @brief GPIO map - schematic "SCH_Schematic1_2-draft_2_2026-07-28".
 *
 * EDIT THIS FILE ONLY IF YOU REWIRE THE BOARD.
 * Everything else (speeds, gains, timeouts) lives in the other config files.
 *
 * Free pins on this build: IO33, IO23, IO5, IO12, IO15, IO0.
 */
#ifndef PINS_H
#define PINS_H

//------------------------- motor drivers ---------------------------------//
// Two TB6612FNG boards, both half-bridges paralleled on each board, so one
// board drives one motor with 3 GPIOs. STBY is tied to +3.3 V on the PCB.
// MOTOR A = LEFT wheel, MOTOR B = RIGHT wheel.
#define MOTOR_A_IN1             25        // IO25
#define MOTOR_A_IN2             26        // IO26
#define MOTOR_A_EN              27        // IO27, PWMA+PWMB
#define MOTOR_B_IN1             19        // IO19
#define MOTOR_B_IN2             18        // IO18
#define MOTOR_B_EN              4         // IO4,  PWMA+PWMB

//--------------------------- I2C bus -------------------------------------//
// Shared by the BNO085, the 3x INA226 and the QMC5883L inside the GPS module.
#define SDA_PIN                 21        // IO21
#define SCL_PIN                 22        // IO22
#define I2C_CLOCK_HZ            400000

//---------------------------- GPS UART -----------------------------------//
#define PIN_GPS_RX              16        // IO16 <- GPS TX
#define PIN_GPS_TX              17        // IO17 -> GPS RX

//------------------------ safety and indicators --------------------------//
// IO13 is an emergency INPUT and IO14 is the relay OUTPUT that powers the motor
// rails. These were the other way round in the firmware - IO14 was read as an
// "emergency relay sense" and nothing ever drove a relay at all. The coil was
// therefore never energized, the relay never closed, the motor rails stayed
// dead, and the pack (which sits behind the relay) read 0 V and could not be
// measured. The symptom on the bench was that the relay never clicked.
//
// Schematic net "ESP CHECK EMER": SW2 to ground with R2, a 10K pull-up to
// +3.3 V. So the line idles HIGH and the switch pulls it LOW - it is active
// LOW, not active high. Configured with the internal pull-up as well, so the
// board reads correctly whether or not R2 is populated: unpressed is HIGH
// either way, pressed is LOW either way.
#define PIN_E_EMER              13        // IO13 <- SW2 emergency switch, LOW = tripped
#define PIN_ESP_EMER_SW         -1        // no second button in this build

// Schematic net "E-EMER": IO14 to the relay module's EN pin. Active HIGH - the
// pin is driven high to close the relay, and the firmware holds it low until it
// has decided the robot is safe to power.
//
// The schematic shows R1 as a 10K PULL-UP on this line. Do not fit it. EN is
// active high, so a pull-up closes the contactor whenever the ESP32 is not
// actively holding it low - which is exactly during reset, boot and any crash,
// the three moments the motor rails must be dead. Fit a pull-DOWN instead. It
// costs nothing: the coil current is what draws power, and a MOSFET gate held
// high draws essentially none.
//
// Drive it through a MOSFET or transistor, never straight from the GPIO: a 5 V
// coil wants 70-100 mA and an ESP32 pin is good for about 20 mA.
#define PIN_RELAY_MAIN          14        // IO14 -> motor rail relay coil
#define RELAY_MAIN_ACTIVE_LOW   0         // 0 = drive HIGH to close
// Input polarity of the two emergency inputs: 1 = the pin reads LOW while the
// emergency is ASSERTED, 0 = it reads HIGH. safety_manager.h tests these with
// #if, so an undefined name would silently mean "active high" there while
// still failing to compile in the host tests.
#define E_EMER_ACTIVE_LOW       1         // R2 pull-up idles high; SW2 pulls it low
#define ESP_EMER_SW_ACTIVE_LOW  0         // button drives the pin high when pressed
#define PIN_LED_RED             2         // IO2 through 330R: e-stop / link lost
#define PIN_LED_GREEN           -1        // hardwired to 5 V, firmware never drives it
#define PIN_FAN_CTRL            -1        // fans run from their own buck; set a GPIO only
                                          // if you add a MOSFET for fan switching

//--------------------------- INA226 alerts -------------------------------//
#define INA226_ALERT_MOTOR_A    34        // IO34 (input only)
#define INA226_ALERT_MOTOR_B    35        // IO35 (input only)
#define INA226_ALERT_PROCESSOR  32        // IO32

#endif // PINS_H

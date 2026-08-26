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
#define PIN_E_EMER              14        // IO14 <- "E-EMER" emergency relay sense
#define PIN_ESP_EMER_SW         13        // IO13 <- on-board e-stop button
// Input polarity of the two emergency inputs: 1 = the pin reads LOW while the
// emergency is ASSERTED, 0 = it reads HIGH. safety_manager.h tests these with
// #if, so an undefined name would silently mean "active high" there while
// still failing to compile in the host tests.
#define E_EMER_ACTIVE_LOW       0         // relay sense pulls the line high on trip
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

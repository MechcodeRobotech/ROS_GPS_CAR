/**
 * @file power.h
 * @brief 4x INA226 voltage/current sensors and battery protection.
 *
 * Wired to schematic draft_5 (SCH_Schematic1_5-draft_5_2026-08-04):
 *
 *   VCC BATT ─ switch ─ VCC ON MODULE ─┬─ [FAN_SENSOR] ─ VCC FAN ─ LM2596 ─ +5V FAN ─ diode ─ 4x fan
 *                                      └─ [PROCESSOR_SENSOR] ─ VCC PROCESSOR ─ LM2596 ─ +5V ─ ESP32
 *
 *   VCC BATT ─ fuse ─ mushroom SW ─ relay ─ VCC EMER ─┬─ [MOTOR_A_SENSOR] ─ VCC MOTOR A ─ diode ─ TB6612 VM
 *                                                     └─ [MOTOR_B_SENSOR] ─ VCC MOTOR B ─ diode ─ TB6612 VM
 *
 * Each sensor therefore reads the rail BEFORE its diode / buck converter.
 * The motor voltage that actually reaches the driver is
 *      V_driver = V_sensor - MOTOR_DIODE_VF_V   (see motor.h)
 * which matters because the back-EMF wheel-speed model uses it.
 *
 * SET THE A0/A1 JUMPERS on each module to match the addresses below before
 * powering up. The serial monitor prints "[INA226] ... missing" for every
 * sensor it cannot find. Address table of the INA226 (A1,A0 -> address):
 *      GND,GND 0x40   GND,VS 0x41   GND,SDA 0x42   GND,SCL 0x43
 *      VS,GND  0x44   VS,VS  0x45   VS,SDA  0x46   VS,SCL  0x47
 */
#ifndef POWER_CONFIG_H
#define POWER_CONFIG_H

//---------------------------- addresses ----------------------------------//
#define INA226_ADDR_PROCESSOR   0x40      // VCC ON MODULE -> processor buck (= pack voltage)
#define INA226_ADDR_MOTOR_A     0x41      // VCC EMER -> motor A rail
#define INA226_ADDR_MOTOR_B     0x44      // VCC EMER -> motor B rail
#define INA226_ADDR_FAN         0x45      // VCC ON MODULE -> fan buck   (draft_5, new)

//------------------------------ shunts -----------------------------------//
// Stock CJMCU-226 modules ship with 0.1 ohm (max ~0.8 A): fine for the
// processor and fan rails, too small for the motors. Fit 0.01 ohm on the two
// motor sensors and keep these numbers in step with the parts you fitted.
#define INA226_SHUNT_PROCESSOR_OHM   0.100f
#define INA226_MAXA_PROCESSOR        0.8f
#define INA226_SHUNT_FAN_OHM         0.100f
#define INA226_MAXA_FAN              1.5f      // 4 fans in parallel
#define INA226_SHUNT_MOTOR_OHM       0.010f
#define INA226_MAXA_MOTOR            8.0f

//------------------------ battery protection -----------------------------//
// 3S LiPo: 12.6 V full, 11.1 V nominal, 9.9 V empty (3.3 V per cell).
// Measured on the processor rail, which sits directly on VCC ON MODULE.
#define BATT_CELLS              3
#define BATT_WARN_V             10.8f     // 3.60 V/cell -> orange on the web UI
#define BATT_CUTOFF_V           9.9f      // 3.30 V/cell -> stop driving, protect the pack
#define BATT_CUTOFF_HOLD_MS     3000      // must stay low this long (ignore load dips)

#endif // POWER_CONFIG_H

/**
 * @file gps.h
 * @brief GEP-M10-DQ (u-blox M10050) UART settings.
 *
 * The baud rate is detected automatically at boot, so normally you do not have
 * to touch anything here. For the best results configure the module once with
 * u-center: 5 Hz navigation rate, GGA + RMC + VTG on, GSV/GSA/GLL off.
 *
 * The compass inside the same module is configured in compass.h, the pins in pins.h.
 */
#ifndef GPS_CONFIG_H
#define GPS_CONFIG_H

#define GPS_SERIAL_NUM          2         // Serial2
#define GPS_BAUD_DEFAULT        115200    // module default
#define GPS_BAUD_CANDIDATES     {115200, 38400, 9600, 57600}
#define GPS_BAUD_PROBE_MS       1200      // time spent on each candidate while detecting
#define GPS_STALE_MS            3000      // no valid sentence for this long -> fix invalid

#endif // GPS_CONFIG_H

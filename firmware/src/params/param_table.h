/**
 * The tunables, declared once.
 *
 * WHAT BELONGS HERE
 *
 * Anything that is a property of THIS robot rather than of the design: a
 * measured diode drop, a wheel that wore down, where the antenna ended up. Add
 * a line and the web settings page grows a field, because the page is built
 * from whatever the board reports.
 *
 * WHAT DOES NOT
 *
 * Pin numbers. A GPIO describes how the board is soldered, not a setting, and
 * making one changeable at runtime would turn a wiring mistake into a puzzle
 * with no error message. pins.h stays compiled in, and the UI shows it
 * read-only.
 *
 * LIVE vs STOPPED
 *
 * LIVE is safe to change while driving. STOPPED is refused while the robot is
 * moving - not because the write is dangerous, but because the value describes
 * the machine the control loops are steering. Changing the wheel diameter or
 * the track width mid-drive silently redefines what every speed and turn rate
 * means, and the robot will do something surprising for a second or two.
 */
#ifndef PARAM_TABLE_H
#define PARAM_TABLE_H

#include <config.h>
#include "param_registry.h"

// Keep in step with the array below. The registry is statically sized, so this
// is what stops it silently truncating when a parameter is added.
#define PARAM_COUNT 22

static const ParamDef PARAM_TABLE[PARAM_COUNT] = {
  // ---- motor electrical model -------------------------------------------
  // These feed the back-EMF wheel-speed estimate, which is what the robot
  // falls back to whenever GPS is unavailable. The diode drops shipped at
  // 0.45 V from a datasheet and measure 1.81 V and 1.90 V - at 12 V that is
  // 15% of the rail, and the derived speed reads high by the same margin.
  {"motor.a.diode_vf", "V", "motor", MOTOR_A_DIODE_VF_V, 0.0f, 3.0f, PARAM_LIVE,
   "Measured forward drop of motor A's series diode. Measure across it with the motor running."},
  {"motor.b.diode_vf", "V", "motor", MOTOR_B_DIODE_VF_V, 0.0f, 3.0f, PARAM_LIVE,
   "Measured forward drop of motor B's series diode."},
  {"motor.a.diode_r", "ohm", "motor", MOTOR_A_DIODE_R_OHM, 0.0f, 1.0f, PARAM_LIVE,
   "Dynamic resistance of motor A's diode - the slope of its V/I curve."},
  {"motor.b.diode_r", "ohm", "motor", MOTOR_B_DIODE_R_OHM, 0.0f, 1.0f, PARAM_LIVE,
   "Dynamic resistance of motor B's diode."},
  {"motor.resistance", "ohm", "motor", MOTOR_RESISTANCE_OHM, 0.1f, 50.0f, PARAM_LIVE,
   "Winding + driver + wiring resistance. Measure across a stalled motor at a known duty."},
  {"motor.ke", "V/rpm", "motor", MOTOR_KE_V_PER_RPM, 0.0001f, 1.0f, PARAM_LIVE,
   "Back-EMF constant. Spin the motor at a known rpm and measure the open-circuit volts."},
  {"motor.stall_a", "A", "motor", MOTOR_STALL_CURRENT_A, 0.1f, 10.0f, PARAM_LIVE,
   "Current that means the motor is stalled rather than merely loaded."},

  // ---- physical robot ----------------------------------------------------
  // STOPPED: these define what a metre and a degree mean to every control loop.
  {"robot.wheel_diameter", "m", "robot", WHEEL_DIAMETER_M, 0.02f, 0.5f, PARAM_STOPPED,
   "Wheel diameter. Measure loaded, not off the shelf - a soft tyre reads smaller."},
  {"robot.track", "m", "robot", WHEEL_TRACK_M, 0.05f, 2.0f, PARAM_STOPPED,
   "Centre of the left drive wheel to centre of the right."},
  {"robot.max_speed", "m/s", "robot", ROBOT_MAX_SPEED_MPS, 0.05f, 5.0f, PARAM_STOPPED,
   "Fastest the robot may be commanded to drive."},

  // ---- where the sensors sit --------------------------------------------
  // A sensor offset from the drive axle sweeps an arc when the robot turns,
  // and the filter reads that arc as travel. Measured from the midpoint
  // between the drive wheels: x forward, y left.
  {"geometry.gps_x", "m", "geometry", GPS_OFFSET_X_M, -2.0f, 2.0f, PARAM_STOPPED,
   "GPS antenna ahead of the drive axle (+) or behind it (-)."},
  {"geometry.gps_y", "m", "geometry", GPS_OFFSET_Y_M, -2.0f, 2.0f, PARAM_STOPPED,
   "GPS antenna left of centre (+) or right (-)."},
  {"geometry.imu_x", "m", "geometry", IMU_OFFSET_X_M, -2.0f, 2.0f, PARAM_STOPPED,
   "IMU ahead of the drive axle (+) or behind it (-)."},
  {"geometry.imu_y", "m", "geometry", IMU_OFFSET_Y_M, -2.0f, 2.0f, PARAM_STOPPED,
   "IMU left of centre (+) or right (-)."},
  {"geometry.axle_x", "m", "geometry", DRIVE_AXLE_OFFSET_X_M, -2.0f, 2.0f, PARAM_STOPPED,
   "Drive axle ahead of the middle of the chassis (+)."},

  // ---- battery -----------------------------------------------------------
  // A different pack chemistry or cell count changes all of these together.
  {"battery.warn_v", "V", "battery", BATT_WARN_V, 6.0f, 30.0f, PARAM_LIVE,
   "Orange on the web UI. Keep driving."},
  {"battery.soft_stop_v", "V", "battery", BATT_SOFT_STOP_V, 6.0f, 30.0f, PARAM_LIVE,
   "Stop driving, stay powered. Recovers on its own."},
  {"battery.cutoff_v", "V", "battery", BATT_CUTOFF_V, 6.0f, 30.0f, PARAM_LIVE,
   "Hard cut. LATCHES until the robot is switched off and on again."},

  // ---- compass -----------------------------------------------------------
  {"compass.declination", "deg", "compass", COMPASS_DECLINATION_DEG, -45.0f, 45.0f, PARAM_LIVE,
   "Magnetic declination where the robot is. Look it up for your location."},

  // ---- estimator ---------------------------------------------------------
  {"estimator.gps_tau", "s", "estimator", SPEED_EST_GPS_TAU_S, 0.05f, 20.0f, PARAM_LIVE,
   "How fast the speed estimate is pulled toward GPS. Larger is smoother and slower."},
  {"estimator.model_tau", "s", "estimator", SPEED_EST_MODEL_TAU_S, 0.05f, 20.0f, PARAM_LIVE,
   "Same, for the motor model when GPS is unavailable."},
  {"estimator.align_min", "m/s", "estimator", HEADING_ALIGN_MIN_MPS, 0.01f, 2.0f, PARAM_LIVE,
   "Below this speed GPS course is noise, so heading stops trusting it."},
};

#endif  // PARAM_TABLE_H

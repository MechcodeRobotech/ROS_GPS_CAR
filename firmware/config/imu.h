/**
 * @file imu.h
 * @brief BNO085 address and mounting orientation.
 *
 * HEADING CONVENTION FOR THE WHOLE PROJECT: compass degrees.
 *   0 = north, 90 = east, positive = clockwise - the same numbers as the GPS
 *   course and as the bearing to a waypoint. The BNO085 reports yaw
 *   counter-clockwise, hence the -1 below. ROS odometry is converted back to
 *   ENU when it is published.
 *
 * How to check the signs: docs/tuning.md 2.
 */
#ifndef IMU_CONFIG_H
#define IMU_CONFIG_H

#define BNO085_I2C_ADDR         0x4A      // 0x4B if the ADR pad is bridged
#define BNO085_RESET_PIN        -1        // not wired on this board
#define IMU_STALE_MS            500       // no event for this long -> imu_ok = false

// Which body axis points to the front of the robot
#define IMU_ACC_FORWARD_AXIS    0         // 0 = X, 1 = Y, 2 = Z
#define IMU_ACC_FORWARD_SIGN    (+1.0f)

// Sensor frame -> robot heading convention
#define IMU_YAW_TO_COMPASS_SIGN (-1.0f)   // CCW sensor yaw -> CW compass sense
#define IMU_GYRO_Z_SIGN         (+1.0f)   // -1.0f only if the board is mounted upside down
#define IMU_YAW_OFFSET_DEG      0.0f      // squares the sensor to the chassis

#endif // IMU_CONFIG_H

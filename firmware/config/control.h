/**
 * @file control.h
 * @brief Control loop rates, PID gains and state estimator tuning.
 *
 * Every gain here can also be changed at runtime from the web UI
 * (topic /gps_localize/config/pid) - these are only the power-on defaults.
 */
#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

//===================== Loop / publish rates =================//
#define CTRL_PERIOD_MS          10        // 100 Hz control loop
#define PUB_ODOM_DIV            5         // 100/5  = 20 Hz odometry
#define PUB_IMU_DIV             2         // 100/2  = 50 Hz imu
#define PUB_TELEMETRY_DIV       10        // 100/10 = 10 Hz telemetry
#define INA_READ_DIV            20        // 100/20 =  5 Hz power sensors

//====================== Heading PID =========================//
// Input : heading error [deg], -180..180
// Output: differential PWM added to the left wheel and removed from the right
#define HEADING_KP              9.0f
#define HEADING_KI              0.35f
#define HEADING_KD              1.2f
#define HEADING_KF              0.0f
#define HEADING_I_MAX           250.0f
#define HEADING_I_MIN           (-HEADING_I_MAX)
#define HEADING_ERROR_TOL_DEG   1.5f      // deadband
#define HEADING_D_FILTER_HZ     8.0f
#define HEADING_OUT_MAX         600.0f    // max differential PWM
#define HEADING_SPIN_ERR_DEG    45.0f     // error above this -> turn on the spot before driving
#define HEADING_SPIN_EXIT_DEG   12.0f     // stop spinning once the error drops below this
#define HEADING_SPIN_PWM        420       // PWM used while turning on the spot
#define TURN_IN_PLACE_TOL_DEG   3.0f      // "turn to 90 deg" finishes inside this
#define TURN_IN_PLACE_SETTLE_MS 250       // and must stay inside it this long

//======================= Speed PI ===========================//
// Input : speed error [m/s] (target - fused estimate)
// Output: PWM correction added on top of the feed-forward term
#define SPEED_KP                420.0f
#define SPEED_KI                160.0f
#define SPEED_KD                0.0f
#define SPEED_KF                0.0f
#define SPEED_I_MAX             400.0f
#define SPEED_I_MIN             (-SPEED_I_MAX)
#define SPEED_ERROR_TOL_MPS     0.01f
#define SPEED_D_FILTER_HZ       5.0f
#define SPEED_FF_GAIN           1.0f      // 1.0 = full open-loop feed-forward (pwm = v/vmax * PWM_MAX)
#define SPEED_SLEW_MPS_S        0.6f      // ramp on the speed setpoint [m/s per second]

//==================== Motor acceleration ramp ===============//
// Applied to the PWM that actually reaches the TB6612, on top of the setpoint
// slew above. It limits inrush current, protects the gearbox and stops the
// robot from lurching when a command arrives as a step.
//   MOTOR_ACCEL_RAMP_S = seconds from stopped to full PWM
//   per 100 Hz control tick that is  PWM_MAX * dt / ramp = 1023 * 0.01 / 0.6 = 17 counts
//   as a fraction of full scale       0.017 per tick at 100 Hz  (= 0.17 at 10 Hz)
// An emergency stop bypasses the ramp completely (motor.disable()).
#define MOTOR_ACCEL_RAMP_S      0.6f      // soft start
#define MOTOR_DECEL_RAMP_S      0.3f      // stopping is allowed to be twice as quick

//=================== Heading estimator ======================//
// 0 = BNO085 rotation vector only (its own magnetometer, absolute but it sits
//     right next to the motors)
// 1 = BNO085 gyro, aligned to the GPS course while moving (relative until the
//     robot has driven a few metres)
// 2 = GPS course over ground only (only usable while moving)
// 3 = DEFAULT. BNO085 gyro, seeded by the QMC5883L compass in the GPS module
//     while standing still, refined by the GPS course while moving.
//     This is the one that knows which way the robot points BEFORE it starts.
#define HEADING_SOURCE          3
#define HEADING_ALIGN_MIN_MPS   0.25f     // only trust GPS course above this speed
#define HEADING_ALIGN_TAU_S     6.0f      // time constant of the IMU->GPS alignment [s]
#define HEADING_ALIGN_MAX_DPS   15.0f     // clamp on alignment correction rate [deg/s]
#define HEADING_MAG_TAU_S       4.0f      // time constant of the IMU->compass alignment [s]
#define HEADING_MAG_STILL_MPS   0.10f     // below this the robot counts as standing still
#define MAG_READ_DIV            5         // 100/5 = 20 Hz compass reads

//==================== Speed estimator =======================//
// Complementary filter: IMU longitudinal acceleration (fast) + GPS ground
// speed (slow but drift free). Falls back to the motor model when GPS is lost.
#define SPEED_EST_GPS_TAU_S     1.5f      // GPS pull-in time constant [s]
#define SPEED_EST_MODEL_TAU_S   2.5f      // motor-model pull-in when GPS is unavailable [s]
#define SPEED_EST_ACC_DEADBAND  0.08f     // ignore |accel| below this [m/s^2] (sensor noise)
#define SPEED_EST_MAX_MPS       1.5f      // hard clamp, protects against integrator runaway

//======================= Odometry ===========================//
#define ODOM_FRAME_ID           "odom"
#define ODOM_CHILD_FRAME_ID     "base_link"
#define IMU_FRAME_ID            "imu_link"
#define GPS_FRAME_ID            "gps_link"

#endif // CONTROL_CONFIG_H

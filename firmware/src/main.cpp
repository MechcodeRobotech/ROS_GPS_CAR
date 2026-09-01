/**
 * GPS_Localize - ESP32 firmware (micro-ROS)
 * ---------------------------------------------------------------------------
 * Outdoor differential-drive robot, GPS waypoints, no wheel encoders.
 *
 *  - closes the heading (BNO085 + PID) and speed loops LOCALLY at 100 Hz, so a
 *    Wi-Fi hiccup cannot make the robot swerve
 *  - stops the motors on its own if the ROS heartbeat, the command stream or
 *    the micro-ROS agent disappears (see lib/safety/safety_manager.h)
 *  - publishes GPS fix, IMU, dead-reckoned odometry and one packed telemetry
 *    array that feeds the web UI
 *
 * MEMORY POLICY (the customer runs this for hours, it must not creep):
 *  - no String, no malloc/new anywhere after setup()
 *  - every ROS message uses a statically allocated buffer, sized at compile time
 *  - the parsers (NMEA) and drivers work on fixed buffers
 *  - free heap is published in the telemetry array so a leak is visible on the web UI
 */
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <builtin_interfaces/msg/time.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <gps_localize_msgs/msg/telemetry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <nav_msgs/msg/odometry.h>

#include <Wire.h>
#include <Preferences.h>
#include <config.h>
#include <motor_tb6612.h>
#include <INA226.h>
#include <gps_nmea.h>
#include <compass_qmc5883.h>
#include <imu_bno085.h>
#include <motor_model.h>
#include <state_estimator.h>
#include <pose_ekf.h>
#include <lever_arm.h>
#include <drive_controller.h>
#include <safety_manager.h>
#if defined(MICROROS_TRANSPORT_WIFI)
#include <ota_updater.h>
#endif
#include <power_monitor.h>

//============================ helpers ======================================//
#define RCCHECK(fn)     { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }
#define EXECUTE_EVERY_N_MS(MS, X) do { static volatile int64_t t0 = -1; \
  if (t0 == -1) t0 = uxr_millis(); \
  if (uxr_millis() - t0 > (MS)) { X; t0 = uxr_millis(); } } while (0)

//========================= micro-ROS entities ==============================//
static rclc_support_t   support;
static rcl_allocator_t  allocator;
static rcl_init_options_t init_options;
static rcl_node_t       node;
static rcl_timer_t      ctrl_timer;
static rclc_executor_t  executor;

static rcl_publisher_t pub_odom;
static rcl_publisher_t pub_imu;
static rcl_publisher_t pub_fix;
static rcl_publisher_t pub_telemetry;

static rcl_subscription_t sub_cmd_move;
static rcl_subscription_t sub_cmd_manual;
static rcl_subscription_t sub_cmd_vel;
static rcl_subscription_t sub_heartbeat;
static rcl_subscription_t sub_estop;
static rcl_subscription_t sub_config;

static nav_msgs__msg__Odometry        msg_odom;
static sensor_msgs__msg__Imu          msg_imu;
static sensor_msgs__msg__NavSatFix    msg_fix;
static gps_localize_msgs__msg__Telemetry msg_telemetry;

static geometry_msgs__msg__Twist      in_cmd_move;
static geometry_msgs__msg__Twist      in_cmd_manual;
static geometry_msgs__msg__Twist      in_cmd_vel;
static std_msgs__msg__Int32           in_heartbeat;
static std_msgs__msg__Bool            in_estop;
static std_msgs__msg__Float32MultiArray in_config;

//--- static payload buffers (never freed, never grown) ---
// TELEMETRY_LEN is gone with the packed array: the message carries named
// fields now, so there is no length for the two sides to disagree about.
#define CONFIG_MAX_LEN 12
static float config_buf[CONFIG_MAX_LEN];

static char frame_odom[]  = ODOM_FRAME_ID;
static char frame_base[]  = ODOM_CHILD_FRAME_ID;
static char frame_imu[]   = IMU_FRAME_ID;
static char frame_gps[]   = GPS_FRAME_ID;

enum Conn { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static Conn conn_state = WAITING_AGENT;

//============================== hardware ===================================//
static MotorTB6612   motor_left;    // MOTOR A
static MotorTB6612   motor_right;   // MOTOR B
static PowerMonitor  power;      // the 4x INA226, and what each one means
static GpsNmea       gps;
static CompassQMC5883 compass;
static ImuBno085     imu(BNO085_RESET_PIN);
static Preferences   prefs;               // NVS: keeps the compass calibration over a reboot
static StateEstimator estimator;
static PoseEkf       pose_ekf;   // GPS-corrected position; see pose_ekf.h
static DriveController drive;
static SafetyManager  safety;
static MotorModel     model_left, model_right;   // wheel speed from volts + amps

//============================= app state ===================================//
static volatile uint32_t last_heartbeat_ms = 0;
static volatile uint32_t last_cmd_ms       = 0;
static volatile bool     sw_estop          = false;
static volatile bool     have_heartbeat    = false;


static uint32_t loop_us = 0;
static uint16_t div_odom = 0, div_imu = 0, div_tel = 0, div_ina = 0, div_mag = 0;
static uint8_t  mag_calib_state = 0;   // 0 idle, 1 capturing, 2 finished ok, 3 failed
static uint32_t last_ctrl_us = 0;
static uint32_t last_gps_pub_ms = 0;

static bool time_synced = false;
static IPAddress agent_ip = AGENT_IP;        // re-resolved by name on every reconnect
static uint8_t   agent_retries = 0;

//============================ forward decls ================================//
/** How much the fused speed is trusted, as a variance [(m/s)^2]. The estimator
 *  does not publish one, so this is a conservative constant: it only sets how
 *  fast position uncertainty grows along the direction of travel. */
#define SPEED_TRUST_VAR 0.02f

/** Heading trust as a variance [deg^2], from which source the estimator is
 *  currently locked to. A gyro-only heading is far less trustworthy than one
 *  referenced to GPS course, and that difference belongs in the covariance. */
static float headingTrustVar() {
  switch (estimator.headingRef()) {
    case StateEstimator::REF_GPS:     return 9.0f;    // 3 deg
    case StateEstimator::REF_MAG:     return 25.0f;   // 5 deg
    case StateEstimator::REF_IMU_MAG: return 64.0f;   // 8 deg
    default:                          return 400.0f;  // relative only, 20 deg
  }
}

/** Local ENU from lat/lon, anchored at the first good fix.
 *
 *  The EKF works in metres, but the receiver reports degrees. An
 *  equirectangular projection about a nearby datum is accurate to well under a
 *  centimetre over the hundreds of metres this robot covers, and costs two
 *  multiplies - a full geodetic conversion would be precision nobody can use
 *  on a 2 m fix.
 */
static double datum_lat = 0.0, datum_lon = 0.0;
static bool   datum_set = false;
static float  fix_east = 0.0f, fix_north = 0.0f;

static void fixToLocal(double lat, double lon) {
  if (!datum_set) {
    datum_lat = lat;
    datum_lon = lon;
    datum_set = true;
  }
  const double clat = cos(datum_lat * 0.017453292519943295);
  const float antenna_east  = (float)((lon - datum_lon) * 111320.0 * clat);
  const float antenna_north = (float)((lat - datum_lat) * 110540.0);

  // The receiver reports where the ANTENNA is. The robot turns about the drive
  // axle, and on a two-caster machine the antenna is nowhere near it, so every
  // turn sweeps the antenna through an arc the filter would otherwise read as
  // real movement. Move the reading back to the turning centre before anything
  // else sees it. With the offsets left at zero this is the identity.
  LeverArm gps_arm;
  gps_arm.x = GPS_OFFSET_X_M;
  gps_arm.y = GPS_OFFSET_Y_M;
  leverArmToCentre(antenna_east, antenna_north, estimator.headingDeg(),
                   gps_arm, &fix_east, &fix_north);
}

static bool createEntities();
static bool destroyEntities();
static void controlCallback(rcl_timer_t *timer, int64_t last_call_time);

//====================== compass calibration storage ========================//
static void loadCompassCalibration() {
  CompassQMC5883::Calibration cal;
  prefs.begin("gpsloc", true);                 // read only
  cal.valid = prefs.getBool("mag_ok", false);
  if (cal.valid) {
    cal.off_x = prefs.getFloat("mag_ox", 0.0f);
    cal.off_y = prefs.getFloat("mag_oy", 0.0f);
    cal.off_z = prefs.getFloat("mag_oz", 0.0f);
    cal.scale_x = prefs.getFloat("mag_sx", 1.0f);
    cal.scale_y = prefs.getFloat("mag_sy", 1.0f);
    cal.scale_z = prefs.getFloat("mag_sz", 1.0f);
    cal.field_norm = prefs.getFloat("mag_fn", 0.0f);
    compass.setCalibration(cal);
    Serial.println("[COMPASS] calibration restored from flash");
  } else {
    Serial.println("[COMPASS] not calibrated yet - run the calibration from the web UI");
  }
  prefs.end();
}

static void saveCompassCalibration() {
  const CompassQMC5883::Calibration &cal = compass.calibration();
  prefs.begin("gpsloc", false);
  prefs.putBool("mag_ok", cal.valid);
  prefs.putFloat("mag_ox", cal.off_x);
  prefs.putFloat("mag_oy", cal.off_y);
  prefs.putFloat("mag_oz", cal.off_z);
  prefs.putFloat("mag_sx", cal.scale_x);
  prefs.putFloat("mag_sy", cal.scale_y);
  prefs.putFloat("mag_sz", cal.scale_z);
  prefs.putFloat("mag_fn", cal.field_norm);
  prefs.end();
  Serial.println("[COMPASS] calibration saved to flash");
}

//=============================== time ======================================//
static void syncTime() {
  if (rmw_uros_sync_session(1000) == RMW_RET_OK) time_synced = true;
}

static void fillStamp(builtin_interfaces__msg__Time &stamp) {
  if (time_synced) {
    int64_t ns = rmw_uros_epoch_nanos();
    stamp.sec     = (int32_t)(ns / 1000000000LL);
    stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  } else {
    uint32_t ms = millis();
    stamp.sec     = (int32_t)(ms / 1000UL);
    stamp.nanosec = (uint32_t)((ms % 1000UL) * 1000000UL);
  }
}

//=========================== message plumbing ==============================//
static void bindString(rosidl_runtime_c__String &s, char *buf, size_t cap) {
  s.data     = buf;
  s.size     = strlen(buf);
  s.capacity = cap;
}

static void initMessages() {
  memset(&msg_odom, 0, sizeof(msg_odom));
  bindString(msg_odom.header.frame_id, frame_odom, sizeof(frame_odom));
  bindString(msg_odom.child_frame_id,  frame_base, sizeof(frame_base));
  msg_odom.pose.pose.orientation.w = 1.0;
  // covariance: position from GPS (~2.5 m), heading from IMU (~5 deg)
  msg_odom.pose.covariance[0]  = 6.0;    // x
  msg_odom.pose.covariance[7]  = 6.0;    // y
  msg_odom.pose.covariance[35] = 0.01;   // yaw
  msg_odom.twist.covariance[0]  = 0.05;
  msg_odom.twist.covariance[35] = 0.05;

  memset(&msg_imu, 0, sizeof(msg_imu));
  bindString(msg_imu.header.frame_id, frame_imu, sizeof(frame_imu));
  msg_imu.orientation.w = 1.0;
  msg_imu.orientation_covariance[0] = 0.01;
  msg_imu.orientation_covariance[4] = 0.01;
  msg_imu.orientation_covariance[8] = 0.05;
  msg_imu.angular_velocity_covariance[0] = 0.001;
  msg_imu.angular_velocity_covariance[4] = 0.001;
  msg_imu.angular_velocity_covariance[8] = 0.001;
  msg_imu.linear_acceleration_covariance[0] = 0.05;
  msg_imu.linear_acceleration_covariance[4] = 0.05;
  msg_imu.linear_acceleration_covariance[8] = 0.05;

  memset(&msg_fix, 0, sizeof(msg_fix));
  bindString(msg_fix.header.frame_id, frame_gps, sizeof(frame_gps));
  msg_fix.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;
  msg_fix.position_covariance_type =
      sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_APPROXIMATED;

  // A named message needs no backing buffer: every field is a plain float in
  // the struct, so there is nothing to size or capacity-check.
  memset(&msg_telemetry, 0, sizeof(msg_telemetry));

  memset(&in_config, 0, sizeof(in_config));
  in_config.data.data     = config_buf;
  in_config.data.size     = 0;
  in_config.data.capacity = CONFIG_MAX_LEN;

  memset(&in_cmd_move, 0, sizeof(in_cmd_move));
  memset(&in_cmd_manual, 0, sizeof(in_cmd_manual));
  memset(&in_cmd_vel, 0, sizeof(in_cmd_vel));
  memset(&in_heartbeat, 0, sizeof(in_heartbeat));
  memset(&in_estop, 0, sizeof(in_estop));
}

//============================== callbacks ==================================//
/**
 * Twist: linear.x = speed [m/s] (signed), angular.z = absolute heading [deg 0..360]
 *        linear.y = 1.0 -> rotate on the spot to that heading and stop
 *                          ("turn 90 degrees", "turn to north"; works indoors,
 *                           no GPS needed)
 */
static void cmd_move_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;

  if ((float)m->linear.y > 0.5f) {
    drive.setTurnTo((float)m->angular.z);
    last_cmd_ms = millis();
    return;
  }

  float v = (float)m->linear.x;
  if (v >  ROBOT_MAX_SPEED_MPS) v =  ROBOT_MAX_SPEED_MPS;
  if (v < -ROBOT_MAX_SPEED_MPS) v = -ROBOT_MAX_SPEED_MPS;
  drive.setAuto(v, (float)m->angular.z);
  last_cmd_ms = millis();
}

/**
 * Standard ROS teleop interface: linear.x [m/s], angular.z [rad/s] counter-clockwise.
 * Lets `ros2 run teleop_twist_keyboard teleop_twist_keyboard`, a joystick node or
 * any other stock tool drive the robot without knowing about this project.
 */
static void cmd_vel_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  float v = (float)m->linear.x;
  if (v >  ROBOT_MAX_SPEED_MPS) v =  ROBOT_MAX_SPEED_MPS;
  if (v < -ROBOT_MAX_SPEED_MPS) v = -ROBOT_MAX_SPEED_MPS;
  // ROS yaw rate is counter-clockwise positive, this project steers clockwise positive
  drive.setManual(v, (float)(-m->angular.z) * 57.29578f);
  last_cmd_ms = millis();
}

/** Twist: linear.x = speed [m/s], angular.z = yaw rate [deg/s] - web UI jog buttons */
static void cmd_manual_cb(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  float v = (float)m->linear.x;
  if (v >  ROBOT_MAX_SPEED_MPS) v =  ROBOT_MAX_SPEED_MPS;
  if (v < -ROBOT_MAX_SPEED_MPS) v = -ROBOT_MAX_SPEED_MPS;
  drive.setManual(v, (float)m->angular.z);
  last_cmd_ms = millis();
}

static void heartbeat_cb(const void *msgin) {
  (void)msgin;
  last_heartbeat_ms = millis();
  have_heartbeat = true;
}

static void estop_cb(const void *msgin) {
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
  sw_estop = m->data;
  if (sw_estop) drive.stop();
}

/**
 * Runtime configuration, data[0] selects the block:
 *   0, Kp, Ki, Kd, Kf, tol, i_min, i_max   -> heading PID
 *   1, Kp, Ki, Kd, Kf, tol, i_min, i_max   -> speed PID
 *   2, heading_source, align_min, align_tau, gps_tau, model_tau, track_m,
 *      mag_enable, mag_tau, mag_still, declination_deg, mount_offset_deg
 *   3                                       -> reset odometry + estimator
 *   4, 1|0                                  -> start / finish compass calibration
 *   5, accel_ramp_s, decel_ramp_s           -> motor acceleration ramp
 *   6, ke, resistance, stall_a, dA_vf, dA_r, dB_vf, dB_r -> motor model
 *   7, heading_deg                          -> set the heading by hand
 *                                              ("set start pose" from the web map)
 */
static void config_cb(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *m = (const std_msgs__msg__Float32MultiArray *)msgin;
  if (m->data.size < 1) return;
  const int sel = (int)m->data.data[0];
  const size_t n = m->data.size;

  if (sel == 0 || sel == 1) {
    if (n < 6) return;
    PIDF &pid = (sel == 0) ? drive.headingPid() : drive.speedPid();
    pid.setPIDF(m->data.data[1], m->data.data[2], m->data.data[3], m->data.data[4], m->data.data[5]);
    if (n >= 8) pid.setIClamp(m->data.data[6], m->data.data[7]);
    pid.reset();
  } else if (sel == 2) {
    StateEstimator::Config &c = estimator.config();
    if (n >= 2) c.heading_source = (uint8_t)m->data.data[1];
    if (n >= 3) c.align_min_mps  = m->data.data[2];
    if (n >= 4) c.align_tau_s    = m->data.data[3];
    if (n >= 5) c.gps_tau_s      = m->data.data[4];
    if (n >= 6) c.model_tau_s    = m->data.data[5];
    if (n >= 7) c.track_m        = m->data.data[6];
    if (n >= 8) c.mag_enable     = m->data.data[7] > 0.5f;
    if (n >= 9) c.mag_tau_s      = m->data.data[8];
    if (n >= 10) c.mag_still_mps = m->data.data[9];
    if (n >= 11) compass.setDeclination(m->data.data[10]);
    if (n >= 12) compass.setMounting(m->data.data[11], COMPASS_INVERT);
  } else if (sel == 3) {
    estimator.reset();
    drive.reset();
  } else if (sel == 4) {
    // compass calibration: [4, 1] starts capturing, [4, 0] finishes and saves
    const bool start = (n >= 2) && (m->data.data[1] > 0.5f);
    if (start) {
      compass.startCalibration();
      mag_calib_state = 1;
    } else if (compass.calibrating()) {
      const bool good = compass.finishCalibration();
      mag_calib_state = good ? 2 : 3;
      if (good) saveCompassCalibration();
    }
  } else if (sel == 5) {
    const float accel = (n >= 2) ? m->data.data[1] : MOTOR_ACCEL_RAMP_S;
    const float decel = (n >= 3) ? m->data.data[2] : MOTOR_DECEL_RAMP_S;
    drive.setRamp(accel, decel);
  } else if (sel == 6) {
    // electrical motor model:
    // [6, ke, resistance, stall_a, diodeA_vf, diodeA_r, diodeB_vf, diodeB_r]
    if (n >= 2) { model_left.config().ke_v_per_rpm = m->data.data[1];
                  model_right.config().ke_v_per_rpm = m->data.data[1]; }
    if (n >= 3) { model_left.config().resistance_ohm = m->data.data[2];
                  model_right.config().resistance_ohm = m->data.data[2]; }
    if (n >= 4) { model_left.config().stall_current_a = m->data.data[3];
                  model_right.config().stall_current_a = m->data.data[3]; }
    if (n >= 5) model_left.config().diode_vf_v   = m->data.data[4];
    if (n >= 6) model_left.config().diode_r_ohm  = m->data.data[5];
    if (n >= 7) model_right.config().diode_vf_v  = m->data.data[6];
    if (n >= 8) model_right.config().diode_r_ohm = m->data.data[7];
  } else if (sel == 7) {
    // operator set the pose by hand on the map: pin the heading, restart odometry
    if (n < 2) return;
    const float imu_yaw = wrap360f(IMU_YAW_TO_COMPASS_SIGN * imu.yawDeg() + IMU_YAW_OFFSET_DEG);
    estimator.setHeading(m->data.data[1], imu_yaw);
    estimator.resetOdom();
    drive.reset();
  }
}

//============================== publishing =================================//
static void publishOdom() {
  fillStamp(msg_odom.header.stamp);
  // Prefer the GPS-corrected position; fall back to dead reckoning until the
  // first fix anchors the filter.
  msg_odom.pose.pose.position.x = (double)(pose_ekf.hasFix() ? pose_ekf.x() : estimator.x());
  msg_odom.pose.pose.position.y = (double)(pose_ekf.hasFix() ? pose_ekf.y() : estimator.y());
  msg_odom.pose.pose.position.z = 0.0;

  const double half = 0.5 * (double)estimator.headingEnuRad();
  msg_odom.pose.pose.orientation.x = 0.0;
  msg_odom.pose.pose.orientation.y = 0.0;
  msg_odom.pose.pose.orientation.z = sin(half);
  msg_odom.pose.pose.orientation.w = cos(half);

  msg_odom.twist.twist.linear.x  = (double)estimator.speed();
  msg_odom.twist.twist.angular.z = (double)estimator.yawRate();
  RCSOFTCHECK(rcl_publish(&pub_odom, &msg_odom, NULL));
}

static void publishImu() {
  fillStamp(msg_imu.header.stamp);
  msg_imu.orientation.w = (double)imu.qw();
  msg_imu.orientation.x = (double)imu.qx();
  msg_imu.orientation.y = (double)imu.qy();
  msg_imu.orientation.z = (double)imu.qz();
  msg_imu.angular_velocity.x = (double)imu.gyroX();
  msg_imu.angular_velocity.y = (double)imu.gyroY();
  msg_imu.angular_velocity.z = (double)imu.gyroZ();
  msg_imu.linear_acceleration.x = (double)imu.accX();
  msg_imu.linear_acceleration.y = (double)imu.accY();
  msg_imu.linear_acceleration.z = (double)imu.accZ();
  RCSOFTCHECK(rcl_publish(&pub_imu, &msg_imu, NULL));
}

static void publishFix() {
  fillStamp(msg_fix.header.stamp);
  const uint8_t q = gps.fixQuality();
  msg_fix.status.status = (q == 0) ? sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX
                        : (q >= 2) ? sensor_msgs__msg__NavSatStatus__STATUS_SBAS_FIX
                                   : sensor_msgs__msg__NavSatStatus__STATUS_FIX;
  msg_fix.latitude  = gps.latitude();
  msg_fix.longitude = gps.longitude();
  msg_fix.altitude  = (double)gps.altitude();

  // rough covariance from HDOP (2.5 m base accuracy for a consumer M10)
  double sigma = 2.5 * (double)(gps.hdop() > 0.5f ? gps.hdop() : 1.0f);
  msg_fix.position_covariance[0] = sigma * sigma;
  msg_fix.position_covariance[4] = sigma * sigma;
  msg_fix.position_covariance[8] = (sigma * 2.0) * (sigma * 2.0);
  RCSOFTCHECK(rcl_publish(&pub_fix, &msg_fix, NULL));
}

static void publishTelemetry(RunState state) {
  // Named fields, not indices. Adding a signal is one line in
  // gps_localize_msgs/msg/Telemetry.msg plus one line here - there is no
  // position to keep in step across three files any more.
  gps_localize_msgs__msg__Telemetry &m = msg_telemetry;
  m.state = (float)state;
  m.heading_deg = estimator.headingDeg();
  m.target_heading_deg = drive.targetHeading();
  m.heading_error_deg = drive.headingError();
  m.speed_mps = estimator.speed();
  m.target_speed_mps = drive.targetSpeed();
  m.wheel_rpm_left = estimator.wheelLeftRpm();
  m.wheel_rpm_right = estimator.wheelRightRpm();
  m.wheel_mps_left = estimator.wheelLeftMps();
  m.wheel_mps_right = estimator.wheelRightMps();
  m.yaw_rate_dps = estimator.yawRate() * 57.29578f;
  m.pwm_left = (float)drive.pwmLeft();
  m.pwm_right = (float)drive.pwmRight();
  m.battery_v = power.packVoltage();                      // pack, from the motor rail
  m.battery_a = power.moduleCurrent();                    // module draw, battery side
  m.motor_a_v = power.motorAVoltage();
  m.motor_a_a = power.motorACurrent();
  m.motor_b_v = power.motorBVoltage();
  m.motor_b_a = power.motorBCurrent();
  m.gps_fix_quality = (float)gps.fixQuality();
  m.gps_satellites = (float)gps.satellites();
  m.gps_hdop = gps.hdop();
  m.gps_speed_mps = gps.speedMps();
  m.gps_course_deg = gps.courseDeg();
  m.stop_flags = (float)safety.flags();
  m.free_heap_kb = (float)(ESP.getFreeHeap() / 1024);
  m.loop_us = (float)loop_us;
  m.odom_x = pose_ekf.hasFix() ? pose_ekf.x() : estimator.x();
  m.odom_y = pose_ekf.hasFix() ? pose_ekf.y() : estimator.y();
  m.mag_heading_deg = compass.headingDeg();
  m.heading_ref = (float)estimator.headingRef();          // 0 relative, 1 compass, 2 GPS, 3 IMU mag
  m.mag_status = compass.ok() ? 1.0f : (compass.disturbed() ? 2.0f : 0.0f);
  m.mag_calib_state = (float)mag_calib_state;
  m.sensor_health = (float)estimator.health();            // which sensors are missing, bitfield
  m.speed_source = (float)estimator.speedSource();
  m.motor_rpm_left = model_left.valid()  ? model_left.rpm()  : 0.0f;// back-EMF wheel speed
  m.motor_rpm_right = model_right.valid() ? model_right.rpm() : 0.0f;
  // Motor fault bitfield. Built here rather than assigned from an outer
  // variable: the old packed-array version computed `f` inside its own scope
  // block, and lifting the assignment out of that block left `f` undeclared.
  {
    uint8_t faults = 0;
    if (model_left.stalled())      faults |= 1 << 0;
    if (model_right.stalled())     faults |= 1 << 1;
    if (model_left.openCircuit())  faults |= 1 << 2;
    if (model_right.openCircuit()) faults |= 1 << 3;
    m.motor_faults = (float)faults;
  }
  m.fan_v = power.fanVoltage();                           // draft_5: fan rail sensor
  m.fan_a = power.fanCurrent();
  m.esp32_rail_v = power.esp32RailVoltage();              // ESP32 +5 V rail health
  // So the host can say when the board is behind it. Four bytes, no new
  // publisher, no string handling, no extra type support in the image.
  m.firmware_version = (float)FIRMWARE_VERSION;
  // How the board last restarted, and how long it has been up. A brownout,
  // a panic and a person pressing reset are indistinguishable in the logs
  // without this - and a 3.3 V rail sagging under Wi-Fi bursts shows up as
  // ESP_RST_BROWNOUT here and nowhere else.
  m.reset_reason = (float)esp_reset_reason();
  m.uptime_s     = (float)(millis() / 1000.0f);
  m.pack_valid = power.packValid() ? 1.0f : 0.0f;         // 0 = rail unpowered, pack unknown
  m.pos_sigma_m = pose_ekf.positionSigma();               // 1-sigma position uncertainty [m]
  m.pos_anchored = pose_ekf.hasFix() ? 1.0f : 0.0f;       // has the EKF ever been anchored
  RCSOFTCHECK(rcl_publish(&pub_telemetry, &msg_telemetry, NULL));
}

//============================ control loop =================================//
static float forwardAccel() {
#if IMU_ACC_FORWARD_AXIS == 0
  return IMU_ACC_FORWARD_SIGN * imu.accX();
#elif IMU_ACC_FORWARD_AXIS == 1
  return IMU_ACC_FORWARD_SIGN * imu.accY();
#else
  return IMU_ACC_FORWARD_SIGN * imu.accZ();
#endif
}

static void controlCallback(rcl_timer_t *timer, int64_t) {
  if (!timer) return;
  const uint32_t t_start = micros();
  const uint32_t now_ms  = millis();

  float dt = (last_ctrl_us == 0) ? (CTRL_PERIOD_MS / 1000.0f)
                                 : (t_start - last_ctrl_us) * 1e-6f;
  last_ctrl_us = t_start;
  if (dt <= 0.0f || dt > 0.5f) dt = CTRL_PERIOD_MS / 1000.0f;

  //------------------------------ sensors ------------------------------//
  imu.update();
  gps.update();

  if (++div_mag >= MAG_READ_DIV) {
    div_mag = 0;
    compass.update(imu.rollDeg(), imu.pitchDeg());   // tilt compensated with the IMU
  }

  if (++div_ina >= INA_READ_DIV) {
    div_ina = 0;
    power.update();
  }

  // wheel speed from volts + amps (back-EMF), one estimate per motor
  model_left.update(dt, power.motorAVoltage(), power.motorACurrent(),
                    (float)drive.pwmLeft() / (float)PWM_MAX, power.motorAOk());
  model_right.update(dt, power.motorBVoltage(), power.motorBCurrent(),
                     (float)drive.pwmRight() / (float)PWM_MAX, power.motorBOk());

  //----------------------------- estimation ----------------------------//
  const float imu_yaw   = wrap360f(IMU_YAW_TO_COMPASS_SIGN * imu.yawDeg() + IMU_YAW_OFFSET_DEG);
  const float yaw_rate  = IMU_GYRO_Z_SIGN * imu.gyroZ();   // CCW positive, ROS convention
  // A receiver with NO FIX still streams NMEA continuously, and the speed and
  // course in those sentences are meaningless - often stale, sometimes garbage.
  // Freshness alone is therefore not enough to trust them: it says the receiver
  // is talking, not that it knows where it is.
  //
  // Measured on the bench with the antenna indoors: 0 satellites, HDOP 99.99,
  // fix quality 0 - and a reported ground speed of 2.53 m/s. The estimator
  // believed it, so a board sitting still on a desk reported drifting at up to
  // 1.5 m/s. With a motor driver attached the controller would have acted on
  // that, and OTA refused every update because it thought the robot was moving.
  const bool  gps_ok    = gps.fresh(GPS_STALE_MS)
                          && gps.fixQuality() > 0
                          && gps.satellites() > 0;
  const bool  course_ok = gps.courseValid() && gps.speedMps() > estimator.config().align_min_mps;

  StateEstimator::Inputs est_in;
  est_in.dt              = dt;
  est_in.imu_ok          = imu.ok();
  est_in.imu_yaw         = imu_yaw;
  est_in.yaw_rate        = yaw_rate;
  est_in.acc_fwd         = forwardAccel();
  est_in.gps_ok          = gps_ok;
  est_in.gps_speed       = gps.speedMps();
  est_in.gps_course      = gps.courseDeg();
  est_in.gps_course_ok   = course_ok;
  est_in.mag_ok          = compass.ok() && !compass.calibrating();
  est_in.mag_heading     = compass.headingDeg();
  est_in.imu_mag_ok      = imu.magYawOk();
  est_in.imu_mag_heading = wrap360f(IMU_YAW_TO_COMPASS_SIGN * imu.magYawDeg() + IMU_YAW_OFFSET_DEG);
  // Fallback speed reference. The back-EMF estimate knows about load (slope,
  // grass, a dragging wheel); the open-loop duty model does not. Use it when
  // both motor sensors answered, otherwise fall back to the duty model.
  if (model_left.valid() && model_right.valid()) {
    est_in.model_mps = 0.5f * (model_left.mps() + model_right.mps());
  } else {
    est_in.model_mps = drive.modelSpeedMps();
  }
  estimator.update(est_in);

  // Position: predict at the control rate from the fused heading and speed,
  // then correct whenever a fix arrives. Without this, x/y is open-loop dead
  // reckoning and a small heading error becomes metres of drift.
  pose_ekf.predict(dt, estimator.speed(), estimator.headingDeg(),
                   SPEED_TRUST_VAR, headingTrustVar());

  // Correct on each NEW fix. gps.fresh() alone would re-apply the same fix a
  // hundred times a second and make the filter far more confident than the
  // data justifies, so only a changed timestamp counts as a measurement.
  if (gps_ok && gps.fixQuality() > 0) {
    static uint32_t last_fix_ms = 0;
    const uint32_t fix_ms = gps.lastFixMs();
    if (fix_ms != last_fix_ms) {
      last_fix_ms = fix_ms;
      fixToLocal(gps.latitude(), gps.longitude());
      pose_ekf.updateGps(fix_east, fix_north, gps.hdop(), gps.fixQuality());
    }
  }

  //------------------------------- safety ------------------------------//
  SafetyManager::Inputs si;
  si.sw_estop         = sw_estop;
  si.heartbeat_age_ms = have_heartbeat ? (now_ms - last_heartbeat_ms) : 0xFFFFFFFF;
  si.cmd_age_ms       = (last_cmd_ms == 0) ? 0xFFFFFFFF : (now_ms - last_cmd_ms);
  si.agent_ok         = (conn_state == AGENT_CONNECTED);
  si.battery_v        = power.packVoltage();
  si.battery_valid    = power.packValid();
  si.motion_requested = fabsf(drive.targetSpeed()) > 1e-3f ||
                        fabsf(drive.targetYawRate()) > 1e-3f;   // pure rotation counts too
  safety.update(si);

  //------------------------------- drive -------------------------------//
  if (!safety.allowMotion()) {
    drive.stop();
    motor_left.disable();
    motor_right.disable();
  } else {
    drive.update(dt, estimator);
    motor_left.spin(drive.pwmLeft());
    motor_right.spin(drive.pwmRight());
  }

  //------------------------------ publish ------------------------------//
  const bool moving = si.motion_requested;
  const RunState state = safety.runState(!drive.manual() && moving, drive.manual() && moving);

  if (++div_odom >= PUB_ODOM_DIV) { div_odom = 0; publishOdom(); }
  if (++div_imu  >= PUB_IMU_DIV)  { div_imu  = 0; if (imu.ok()) publishImu(); }
  if (++div_tel  >= PUB_TELEMETRY_DIV) { div_tel = 0; publishTelemetry(state); }

  // GPS is published when a new solution arrives, capped at 5 Hz
  if (gps.lastFixMs() != last_gps_pub_ms && now_ms - last_gps_pub_ms >= 200) {
    last_gps_pub_ms = gps.lastFixMs();
    publishFix();
  }

  loop_us = micros() - t_start;
}

//========================= entities create/destroy =========================//
static bool createEntities() {
  allocator    = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
  rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);
  if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) return false;

  RCCHECK(rclc_node_init_default(&node, ROS_NODE_NAME, "", &support));

  RCCHECK(rclc_publisher_init_default(&pub_odom, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), ROS_NS "/odom"));
  RCCHECK(rclc_publisher_init_default(&pub_imu, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), ROS_NS "/imu/data"));
  RCCHECK(rclc_publisher_init_default(&pub_fix, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), ROS_NS "/gps/fix"));
  RCCHECK(rclc_publisher_init_default(&pub_telemetry, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(gps_localize_msgs, msg, Telemetry), ROS_NS "/telemetry"));

  RCCHECK(rclc_subscription_init_default(&sub_cmd_move, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), ROS_NS "/cmd_move"));
  RCCHECK(rclc_subscription_init_default(&sub_cmd_manual, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), ROS_NS "/cmd_manual"));
  RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), ROS_NS "/cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&sub_heartbeat, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), ROS_NS "/heartbeat"));
  RCCHECK(rclc_subscription_init_default(&sub_estop, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), ROS_NS "/estop"));
  RCCHECK(rclc_subscription_init_default(&sub_config, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), ROS_NS "/config/pid"));

  RCCHECK(rclc_timer_init_default(&ctrl_timer, &support, RCL_MS_TO_NS(CTRL_PERIOD_MS), controlCallback));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &ctrl_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_move,   &in_cmd_move,   &cmd_move_cb,   ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_manual, &in_cmd_manual, &cmd_manual_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel,    &in_cmd_vel,    &cmd_vel_cb,    ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_heartbeat,  &in_heartbeat,  &heartbeat_cb,  ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_estop,      &in_estop,      &estop_cb,      ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_config,     &in_config,     &config_cb,     ON_NEW_DATA));

  syncTime();
  return true;
}

static bool destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);

  rcl_publisher_fini(&pub_odom, &node);
  rcl_publisher_fini(&pub_imu, &node);
  rcl_publisher_fini(&pub_fix, &node);
  rcl_publisher_fini(&pub_telemetry, &node);

  rcl_subscription_fini(&sub_cmd_move, &node);
  rcl_subscription_fini(&sub_cmd_manual, &node);
  rcl_subscription_fini(&sub_cmd_vel, &node);
  rcl_subscription_fini(&sub_heartbeat, &node);
  rcl_subscription_fini(&sub_estop, &node);
  rcl_subscription_fini(&sub_config, &node);

  rcl_timer_fini(&ctrl_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);

  time_synced = false;
  return true;
}

//================================ setup ====================================//
#if defined(MICROROS_TRANSPORT_WIFI)
#include <ESPmDNS.h>

/**
 * Find the micro-ROS agent by NAME, so a new DHCP lease on the Ubuntu PC does
 * not brick the robot (the mor_luam problem).
 *
 *   1. mDNS   "<AGENT_HOSTNAME>.local"  (Ubuntu: sudo apt install avahi-daemon)
 *   2. DNS    plain lookup, works when the router registers DHCP names
 *   3. AGENT_IP from network.h
 *
 * Called again on every reconnect, so the robot follows the PC to a new address
 * while it is running.
 */
/**
 * Accept whatever the user pasted into AGENT_HOSTNAME and return the bare
 * hostname: "mannaja", "mannaja.local" and "mannaja@mannaja.local" (copied
 * straight from the Ubuntu prompt) all become "mannaja".
 */
static const char *agentHostname() {
  static char name[40];
  const char *src = AGENT_HOSTNAME;

  const char *at = strchr(src, '@');       // drop the "user@" part
  if (at) src = at + 1;

  size_t len = strlen(src);
  const char *suffix = ".local";           // strip it: mDNS wants the bare name
  const size_t suffix_len = 6;
  if (len > suffix_len) {
    bool match = true;
    for (size_t i = 0; i < suffix_len; ++i) {
      char a = src[len - suffix_len + i];
      if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');   // no strcasecmp: not
      if (a != suffix[i]) { match = false; break; }          // portable everywhere
    }
    if (match) len -= suffix_len;
  }
  if (len >= sizeof(name)) len = sizeof(name) - 1;

  memcpy(name, src, len);
  name[len] = '\0';
  return name;
}

static IPAddress resolveAgentIp() {
#if USE_AGENT_HOSTNAME
  static bool mdns_started = false;
  const char *host = agentHostname();
  if (!mdns_started) {
    mdns_started = MDNS.begin(WIFI_HOSTNAME);
    if (!mdns_started) Serial.println("[mDNS] could not start responder");
  }

  if (mdns_started) {
    IPAddress found = MDNS.queryHost(host, AGENT_RESOLVE_TIMEOUT_MS);
    if (found != IPAddress((uint32_t)0)) {
      Serial.printf("[agent] %s.local -> %s (mDNS)\n", host, found.toString().c_str());
      return found;
    }
  }

  // plain DNS, first the bare name (router DHCP names), then name.local
  IPAddress dns_found;
  char dotted[48];
  snprintf(dotted, sizeof(dotted), "%s.local", host);
  const char *candidates[2] = { host, dotted };
  for (uint8_t i = 0; i < 2; ++i) {
    if (WiFi.hostByName(candidates[i], dns_found) == 1 && dns_found != IPAddress((uint32_t)0)) {
      Serial.printf("[agent] %s -> %s (DNS)\n", candidates[i], dns_found.toString().c_str());
      return dns_found;
    }
  }
  Serial.printf("[agent] '%s' not found, using fallback %s\n",
                host, AGENT_IP.toString().c_str());
#endif
  return AGENT_IP;
}

/**
 * Connect as fast as the radio allows:
 *   - persistent(false)  : no flash write on every connect
 *   - setSleep(false)    : no power-save latency (matters for a 100 Hz link)
 *   - static IP (opt)    : skips DHCP, typically 1-2 s quicker
 *   - autoReconnect      : the stack re-joins on its own if the AP blinks
 * Target: associated in well under 30 s even on a busy 2.4 GHz band.
 */
// Upper bound on the WIFI_NETWORKS table, so the "already tried" flags live on
// the stack instead of the heap. Raise it if you ever list more than this.
#define WIFI_MAX_NETWORKS 8
// How long to wait on ONE network before moving to the next candidate. The
// overall budget is still the timeout_ms argument.
#define WIFI_ATTEMPT_TIMEOUT_MS 8000

// Which entry of WIFI_NETWORKS actually associated. set_microros_wifi_transports
// must be handed these, not the first row of the table.
static const char *wifi_ssid_active = WIFI_NETWORKS[0].ssid;
static const char *wifi_pass_active = WIFI_NETWORKS[0].pass;

static bool wifiConnect(uint32_t timeout_ms = 30000) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(WIFI_HOSTNAME);

#if USE_STATIC_IP
  if (!WiFi.config(STATIC_IP, STATIC_GATEWAY, STATIC_SUBNET, STATIC_DNS)) {
    Serial.println("[WiFi] static IP rejected, falling back to DHCP");
  }
#endif

  const uint32_t t0 = millis();

  // Scan once, then try the listed networks strongest first. Everything below
  // works on fixed buffers: the only allocation is inside the SDK's scan, and
  // that is released with scanDelete() before this function returns. Nothing
  // here runs again after setup().
  int16_t found = WiFi.scanNetworks(false /*async*/, false /*show hidden*/);
  if (found < 0) found = 0;
  Serial.printf("[WiFi] scan found %d network(s), %u configured\n",
                (int)found, (unsigned)WIFI_NETWORK_COUNT);
  // List what is actually in range. Without this, a board that cannot join
  // tells you nothing about what it COULD join, and setup becomes guesswork.
  for (int16_t s_i = 0; s_i < found && s_i < 20; ++s_i) {
    bool known = false;
    for (size_t k = 0; k < WIFI_NETWORK_COUNT; ++k) {
      if (WiFi.SSID(s_i) == WIFI_NETWORKS[k].ssid) { known = true; break; }
    }
    Serial.printf("       %-32s %4ld dBm  %s\n",
                  WiFi.SSID(s_i).c_str(), (long)WiFi.RSSI(s_i),
                  known ? "<- configured" : "");
  }

  bool tried[WIFI_MAX_NETWORKS] = { false };
  const size_t table_n = (WIFI_NETWORK_COUNT < WIFI_MAX_NETWORKS)
                       ? WIFI_NETWORK_COUNT : WIFI_MAX_NETWORKS;

  for (size_t attempt = 0; attempt < table_n; ++attempt) {
    // pick the strongest configured network that is on the air and untried
    int best_idx = -1;
    int32_t best_rssi = -127;
    for (int16_t s = 0; s < found; ++s) {
      for (size_t k = 0; k < table_n; ++k) {
        if (tried[k]) continue;
        if (WiFi.SSID(s) != WIFI_NETWORKS[k].ssid) continue;
        if (WiFi.RSSI(s) > best_rssi) { best_rssi = WiFi.RSSI(s); best_idx = (int)k; }
      }
    }
    if (best_idx < 0) break;                 // nothing configured is in range
    tried[best_idx] = true;

    const WifiNetwork &net = WIFI_NETWORKS[best_idx];
    Serial.printf("[WiFi] trying \"%s\" (%ld dBm)\n", net.ssid, (long)best_rssi);
    WiFi.begin(net.ssid, net.pass);

    const uint32_t attempt_start = millis();
    while (WiFi.status() != WL_CONNECTED &&
           millis() - attempt_start < WIFI_ATTEMPT_TIMEOUT_MS &&
           millis() - t0 < timeout_ms) {
      delay(50);
    }

    if (WiFi.status() == WL_CONNECTED) {
      wifi_ssid_active = net.ssid;           // micro-ROS transport needs these
      wifi_pass_active = net.pass;
      WiFi.scanDelete();
      Serial.printf("[WiFi] connected to \"%s\" in %lu ms, ip %s, rssi %d dBm\n",
                    net.ssid, (unsigned long)(millis() - t0),
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
      Serial.printf("[WiFi] this robot answers as %s.local\n", WIFI_HOSTNAME);
      return true;
    }
    WiFi.disconnect(false, true);
    if (millis() - t0 >= timeout_ms) break;
  }

  WiFi.scanDelete();
  Serial.println("[WiFi] no configured network could be joined");
  return false;
}
#endif

#if defined(MICROROS_TRANSPORT_WIFI)
OtaUpdater ota;

/**
 * May an update start right now?
 *
 * The only thing that must be true beforehand is that the robot is STANDING
 * STILL. It does not have to be e-stopped first: the firmware owns the
 * contactor, so it drops the motor rails itself in otaOnBegin() below. An
 * earlier version demanded the contactor already be open, which meant every
 * wireless flash began with a walk over to the robot to press the mushroom -
 * defeating the point of flashing wirelessly at all.
 *
 * Motion is the line that is not crossed automatically. Cutting power to a
 * machine that is moving, remotely, because someone somewhere started an
 * upload, is not a decision to take quietly - it stops being an update and
 * becomes an unannounced e-stop. So a moving robot simply does not answer.
 */
static bool otaIsSafe() {
  return fabsf(estimator.speed()) < 0.05f;
}

/**
 * Make the robot safe, then let the flash proceed.
 *
 * Ordered deliberately: commands first so nothing re-applies a duty cycle, then
 * the motors, then the contactor - which removes the power those motors would
 * have used even if everything above somehow failed.
 */
static void otaOnBegin() {
  drive.stop();
  motor_left.stop();
  motor_right.stop();
  safety.beginUpdateLockout();      // rails dead for the rest of this boot
  Serial.println("[OTA] motor rails cut for the update");
}
#endif  // MICROROS_TRANSPORT_WIFI

void setup() {
  Serial.begin(115200);
  delay(50);

  // motors first: make sure nothing spins while the rest boots
  motor_left.begin(MOTOR_A_EN, MOTOR_A_IN1, MOTOR_A_IN2, LEDC_CH_MOTOR_A,
                   PWM_FREQUENCY, PWM_BITS, MOTOR_A_INVERT, MOTOR_BRAKE_ON_STOP);
  motor_right.begin(MOTOR_B_EN, MOTOR_B_IN1, MOTOR_B_IN2, LEDC_CH_MOTOR_B,
                    PWM_FREQUENCY, PWM_BITS, MOTOR_B_INVERT, MOTOR_BRAKE_ON_STOP);
  motor_left.disable();
  motor_right.disable();

  safety.begin();

  pinMode(INA226_ALERT_MOTOR_A, INPUT);
  pinMode(INA226_ALERT_MOTOR_B, INPUT);
  pinMode(INA226_ALERT_PROCESSOR, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN, I2C_CLOCK_HZ);

  // Which devices are actually on the bus, by address. Worth printing at every
  // boot: "the sensor initialised" and "the sensor is answering" are different
  // claims, and only this distinguishes a wiring fault from a driver one.
  {
    Serial.print("[I2C] devices:");
    uint8_t found = 0;
    for (uint8_t a = 0x08; a < 0x78; ++a) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); ++found; }
    }
    if (!found) Serial.print(" NONE - check SDA/SCL and power");
    Serial.println();
  }

  // MODE_BOTH gives the game rotation vector (smooth, relative) *and* the
  // magnetometer-referenced one, so the fallback chain has a spare absolute source
  const ImuBno085::Mode imu_mode = (HEADING_SOURCE == 0) ? ImuBno085::MODE_MAG
                                 : (HEADING_SOURCE == 1) ? ImuBno085::MODE_GAME
                                                         : ImuBno085::MODE_BOTH;
  if (!imu.begin(&Wire, BNO085_I2C_ADDR, imu_mode)) {
    Serial.println("[IMU] BNO085 not found");
  }
  imu.setStaleMs(IMU_STALE_MS);

  if (!power.begin(&Wire))
    Serial.println("[INA226] one or more sensors missing - see the rail readings");

#if USE_QMC5883L
  if (compass.begin(&Wire, QMC5883L_ADDR)) {
    compass.setDeclination(COMPASS_DECLINATION_DEG);
    compass.setMounting(COMPASS_MOUNT_OFFSET_DEG, COMPASS_INVERT);
    compass.setFieldTolerance(COMPASS_FIELD_TOL);
    loadCompassCalibration();
    Serial.println("[COMPASS] QMC5883L ready");
  } else {
    Serial.println("[COMPASS] QMC5883L not found - heading stays relative until the robot moves");
  }
#endif

  static const uint32_t gps_bauds[] = GPS_BAUD_CANDIDATES;
  gps.begin(&Serial2, PIN_GPS_RX, PIN_GPS_TX, GPS_BAUD_DEFAULT, true,
            gps_bauds, sizeof(gps_bauds) / sizeof(gps_bauds[0]), GPS_BAUD_PROBE_MS);
  Serial.printf("[GPS] listening at %lu baud\n", (unsigned long)gps.baud());

  StateEstimator::Config ecfg;
  ecfg.heading_source   = HEADING_SOURCE;
  ecfg.align_min_mps    = HEADING_ALIGN_MIN_MPS;
  ecfg.align_tau_s      = HEADING_ALIGN_TAU_S;
  ecfg.align_max_dps    = HEADING_ALIGN_MAX_DPS;
  ecfg.gps_tau_s        = SPEED_EST_GPS_TAU_S;
  ecfg.model_tau_s      = SPEED_EST_MODEL_TAU_S;
  ecfg.acc_deadband     = SPEED_EST_ACC_DEADBAND;
  ecfg.max_speed_mps    = SPEED_EST_MAX_MPS;
  ecfg.track_m          = WHEEL_TRACK_M;
  ecfg.wheel_diameter_m = WHEEL_DIAMETER_M;
  ecfg.mag_tau_s        = HEADING_MAG_TAU_S;
  ecfg.mag_still_mps    = HEADING_MAG_STILL_MPS;
  ecfg.mag_enable       = USE_QMC5883L && compass.present();
  estimator.begin(ecfg);
  drive.begin();
  drive.setRamp(MOTOR_ACCEL_RAMP_S, MOTOR_DECEL_RAMP_S);

  MotorModel::Config mcfg;
  mcfg.ke_v_per_rpm    = MOTOR_KE_V_PER_RPM;
  mcfg.resistance_ohm  = MOTOR_RESISTANCE_OHM;
  mcfg.wheel_diameter_m = WHEEL_DIAMETER_M;
  mcfg.min_duty        = MOTOR_MIN_DUTY_FOR_EST;
  mcfg.stall_current_a = MOTOR_STALL_CURRENT_A;
  mcfg.open_current_a  = MOTOR_OPEN_CURRENT_A;

  mcfg.diode_vf_v  = MOTOR_A_DIODE_VF_V;      // each motor has its own diode
  mcfg.diode_r_ohm = MOTOR_A_DIODE_R_OHM;
  model_left.begin(mcfg);

  mcfg.diode_vf_v  = MOTOR_B_DIODE_VF_V;
  mcfg.diode_r_ohm = MOTOR_B_DIODE_R_OHM;
  model_right.begin(mcfg);

  initMessages();

#if defined(MICROROS_TRANSPORT_WIFI)
  if (!wifiConnect()) {
    // Do NOT reboot. Rebooting on Wi-Fi failure makes a board that knows no
    // network in range loop forever, and a looping board cannot be configured
    // - the serial console never stays up long enough to talk to it. Proven on
    // the bench: with only an out-of-range SSID configured, the board restarted
    // every few seconds and was unreachable by any means.
    //
    // Stay up instead. The console keeps working, the scan result above says
    // what IS in range, and the retry below picks the network up the moment it
    // appears. Nothing can move: with no agent the safety manager holds
    // STOP_AGENT_LOST, so the motors stay disabled the whole time.
    Serial.println("[WiFi] no network joined - staying up so this board can");
    Serial.println("       still be reached over USB. Retrying in the background.");
    Serial.println("       Add a network to firmware/config/network_secrets.h,");
    Serial.println("       or pick one from the scan list above.");
  }
  agent_ip = resolveAgentIp();
  set_microros_wifi_transports((char *)wifi_ssid_active, (char *)wifi_pass_active, agent_ip, AGENT_PORT);

  // Wireless flashing, so a board bolted into the robot never needs a cable
  // again. Only useful once there is a network, hence here rather than earlier.
  if (WiFi.status() == WL_CONNECTED) {
    ota.begin(WIFI_HOSTNAME, otaIsSafe, otaOnBegin);
  }
#else
  set_microros_serial_transports(Serial);
#endif

  conn_state = WAITING_AGENT;
}

//================================= loop ====================================//
void loop() {
#if defined(MICROROS_TRANSPORT_WIFI)
  // Serviced first and every pass: an update must be able to land even when the
  // agent is unreachable, which is exactly when a board most needs reflashing.
  ota.handle();
#endif

  // Keep trying to join a network if we have none. Backed off so a board with
  // nothing in range does not spend its whole life scanning.
#if defined(MICROROS_TRANSPORT_WIFI)
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t next_wifi_try_ms = 0;
    const uint32_t now_wifi = millis();
    if (now_wifi >= next_wifi_try_ms) {
      next_wifi_try_ms = now_wifi + 30000;      // every 30 s, not continuously
      Serial.println("[WiFi] retrying...");
      if (wifiConnect(15000)) {
        agent_ip = resolveAgentIp();
        set_microros_wifi_transports((char *)wifi_ssid_active, (char *)wifi_pass_active,
                                     agent_ip, AGENT_PORT);
        Serial.println("[WiFi] back on the air");
      }
    }
  }
#endif

  switch (conn_state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(1000, {
        conn_state = (RMW_RET_OK == rmw_uros_ping_agent(500, 5)) ? AGENT_AVAILABLE : WAITING_AGENT;
#if defined(MICROROS_TRANSPORT_WIFI)
        // after a few failed pings, look the agent up by name again: the PC may
        // have come back on a different IP address
        if (conn_state == WAITING_AGENT && ++agent_retries >= 5) {
          agent_retries = 0;
          if (WiFi.status() == WL_CONNECTED) {
            IPAddress fresh = resolveAgentIp();
            if (fresh != agent_ip) {
              agent_ip = fresh;
              set_microros_wifi_transports((char *)wifi_ssid_active, (char *)wifi_pass_active, agent_ip, AGENT_PORT);
              Serial.printf("[agent] now trying %s\n", agent_ip.toString().c_str());
            }
          }
        }
#endif
      });
      break;

    case AGENT_AVAILABLE:
      conn_state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (conn_state == WAITING_AGENT) destroyEntities();
      break;

    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(AGENT_PING_PERIOD_MS,
        conn_state = (RMW_RET_OK == rmw_uros_ping_agent(300, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (conn_state == AGENT_CONNECTED) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
      break;

    case AGENT_DISCONNECTED:
      motor_left.disable();
      motor_right.disable();
      drive.stop();
      have_heartbeat = false;
      destroyEntities();
      conn_state = WAITING_AGENT;
      break;
  }

  // Motors must also stop while the agent is gone (the control timer is not running then)
  if (conn_state != AGENT_CONNECTED) {
    SafetyManager::Inputs si;
    si.sw_estop         = sw_estop;
    si.heartbeat_age_ms = 0xFFFFFFFF;
    si.cmd_age_ms       = 0xFFFFFFFF;
    si.agent_ok         = false;
    si.battery_valid    = false;
    si.motion_requested = false;
    safety.update(si);
    motor_left.disable();
    motor_right.disable();
  }
}

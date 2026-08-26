/**
 * StateEstimator tests: heading arbitration, sensor fallbacks, speed fusion,
 * wheel-speed derivation and odometry.
 */
#include <unity.h>
#include "Arduino.h"
#include "state_estimator.h"

static StateEstimator make(uint8_t source = 4) {
    StateEstimator est;
    StateEstimator::Config cfg;
    cfg.heading_source = source;
    cfg.align_min_mps = 0.25f;
    cfg.align_tau_s = 1.0f;
    cfg.align_max_dps = 180.0f;      // no rate limit in the tests
    cfg.mag_tau_s = 1.0f;
    cfg.mag_still_mps = 0.10f;
    cfg.gps_tau_s = 0.5f;
    cfg.model_tau_s = 0.5f;
    cfg.track_m = 0.30f;
    cfg.wheel_diameter_m = 0.081f;
    est.begin(cfg);
    return est;
}

static StateEstimator::Inputs base_inputs() {
    StateEstimator::Inputs in;
    in.dt = 0.01f;
    in.imu_ok = true;
    in.imu_yaw = 0.0f;
    in.yaw_rate = 0.0f;
    in.acc_fwd = 0.0f;
    return in;
}

static void run(StateEstimator &est, StateEstimator::Inputs in, int cycles) {
    for (int i = 0; i < cycles; ++i) est.update(in);
}

// ------------------------------------------------------------ angle helpers
void test_wrap360_positive(void) { TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, wrap360f(370.0f)); }
void test_wrap360_negative(void) { TEST_ASSERT_FLOAT_WITHIN(0.001f, 350.0f, wrap360f(-10.0f)); }
void test_ang_err_wraps_short_way(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -20.0f, angErrDeg(350.0f, 10.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, angErrDeg(10.0f, 350.0f));
}
void test_wrap_pi_negative(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -(float)M_PI / 2.0f, wrapPif(3.0f * (float)M_PI / 2.0f));
}

// --------------------------------------------------------------- heading ref
void test_compass_gives_heading_while_standing_still(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true;
    in.mag_heading = 90.0f;
    run(est, in, 500);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 90.0f, est.headingDeg());
    TEST_ASSERT_EQUAL(StateEstimator::REF_MAG, est.headingRef());
}

void test_gps_course_wins_while_moving(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true;  in.mag_heading = 90.0f;
    in.gps_ok = true;  in.gps_course = 270.0f; in.gps_course_ok = true; in.gps_speed = 0.5f;
    run(est, in, 500);
    TEST_ASSERT_EQUAL(StateEstimator::REF_GPS, est.headingRef());
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 270.0f, est.headingDeg());
}

void test_falls_back_to_imu_magnetometer_when_compass_dies(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = false;                       // QMC5883L gone
    in.imu_mag_ok = true; in.imu_mag_heading = 45.0f;
    run(est, in, 800);
    TEST_ASSERT_EQUAL(StateEstimator::REF_IMU_MAG, est.headingRef());
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 45.0f, est.headingDeg());
}

void test_no_absolute_source_is_reported(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    run(est, in, 10);
    TEST_ASSERT_EQUAL(StateEstimator::REF_NONE, est.headingRef());
    TEST_ASSERT_TRUE(est.health() & StateEstimator::HEALTH_NO_ABS_HEADING);
}

void test_health_flags_report_missing_sensors(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.imu_ok = false;
    run(est, in, 5);
    TEST_ASSERT_TRUE(est.health() & StateEstimator::HEALTH_NO_IMU);
    TEST_ASSERT_TRUE(est.health() & StateEstimator::HEALTH_NO_GPS);
    TEST_ASSERT_TRUE(est.health() & StateEstimator::HEALTH_NO_COMPASS);
}

void test_gyro_carries_heading_when_everything_absolute_is_gone(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 0.0f;
    run(est, in, 500);                       // learn 0 deg from the compass
    in.mag_ok = false;
    in.yaw_rate = -0.5f;                     // CCW negative -> compass heading grows
    in.imu_yaw = 0.0f;
    for (int i = 0; i < 100; ++i) { in.imu_yaw += 0.5f * 57.29578f * 0.01f; est.update(in); }
    TEST_ASSERT_TRUE(est.headingDeg() > 20.0f);   // it kept turning with the gyro
}

void test_imu_dead_uses_gps_course_directly(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.imu_ok = false;
    in.gps_ok = true; in.gps_course = 123.0f; in.gps_course_ok = true; in.gps_speed = 0.6f;
    run(est, in, 300);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 123.0f, est.headingDeg());
    TEST_ASSERT_EQUAL(StateEstimator::REF_GPS, est.headingRef());
}

void test_imu_and_gps_dead_uses_compass_directly(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.imu_ok = false;
    in.mag_ok = true; in.mag_heading = 200.0f;
    run(est, in, 50);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 200.0f, est.headingDeg());
}

void test_mode0_uses_imu_yaw_as_absolute(void) {
    StateEstimator est = make(0);
    StateEstimator::Inputs in = base_inputs();
    in.imu_yaw = 137.0f;
    run(est, in, 5);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 137.0f, est.headingDeg());
}

void test_mode2_gps_only(void) {
    StateEstimator est = make(2);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_course = 80.0f; in.gps_course_ok = true; in.gps_speed = 0.8f;
    run(est, in, 100);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 80.0f, est.headingDeg());
}

void test_compass_ignored_while_driving_fast(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_speed = 0.6f; in.gps_course = 10.0f; in.gps_course_ok = true;
    in.mag_ok = true; in.mag_heading = 300.0f;      // disagrees on purpose
    run(est, in, 600);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 10.0f, est.headingDeg());
}

// ------------------------------------------------------------------- speed
void test_speed_follows_gps(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_speed = 0.40f;
    run(est, in, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.40f, est.speed());
    TEST_ASSERT_EQUAL(StateEstimator::SPEED_IMU_GPS, est.speedSource());
}

void test_speed_falls_back_to_model_without_gps(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = false; in.model_mps = 0.30f;
    // model_tau_s is 2.5 s, so 500 cycles at dt=0.01 is only 2 time constants
    // and a first-order lag has reached 86.5% - 0.259, outside the +-0.03 the
    // assertion allows. Run 4 time constants, where it is at 98.2%.
    run(est, in, 1000);
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.30f, est.speed());
    TEST_ASSERT_EQUAL(StateEstimator::SPEED_IMU_MODEL, est.speedSource());
}

void test_speed_source_without_imu(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.imu_ok = false; in.gps_ok = true; in.gps_speed = 0.2f;
    run(est, in, 100);
    TEST_ASSERT_EQUAL(StateEstimator::SPEED_GPS, est.speedSource());
}

void test_speed_is_clamped(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.acc_fwd = 50.0f;                      // absurd acceleration
    run(est, in, 1000);
    TEST_ASSERT_TRUE(est.speed() <= 1.51f);
}

void test_small_speed_is_squelched(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_speed = 0.001f;
    run(est, in, 200);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, est.speed());
}

void test_acceleration_deadband(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.acc_fwd = 0.05f;                      // below the 0.08 deadband
    run(est, in, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.speed());
}

// ------------------------------------------------------- wheels and odometry
void test_wheel_speeds_equal_when_driving_straight(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_speed = 0.3f;
    run(est, in, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, est.wheelLeftMps(), est.wheelRightMps());
}

void test_wheel_speeds_split_when_turning(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.gps_ok = true; in.gps_speed = 0.3f;
    in.yaw_rate = 1.0f;                      // 1 rad/s counter-clockwise = turning left
    run(est, in, 500);
    TEST_ASSERT_TRUE(est.wheelRightMps() > est.wheelLeftMps());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.30f, est.wheelRightMps() - est.wheelLeftMps());  // w * track
}

void test_rpm_conversion(void) {
    StateEstimator est = make(4);
    // 0.2545 m per revolution (81 mm wheel) -> 1 m/s is about 236 rpm
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 235.8f, est.mpsToRpm(1.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.mpsToRpm(0.0f));
}

void test_odometry_moves_north_when_heading_north(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 0.0f;         // north
    in.gps_ok = true; in.gps_speed = 0.0f;
    run(est, in, 500);
    in.gps_speed = 1.0f;
    run(est, in, 500);                               // ~5 s at up to 1 m/s
    TEST_ASSERT_TRUE(est.y() > 1.0f);                // north = +y in ENU
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, est.x());
}

void test_odometry_moves_east_when_heading_east(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 90.0f;        // east
    in.gps_ok = true; in.gps_speed = 0.0f;
    run(est, in, 500);
    in.gps_speed = 1.0f;
    run(est, in, 500);
    TEST_ASSERT_TRUE(est.x() > 1.0f);                // east = +x
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, est.y());
}

void test_heading_enu_conversion(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 90.0f;
    run(est, in, 500);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, est.headingEnuRad());   // east = 0 rad in ENU
}

void test_reset_clears_everything(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 90.0f; in.gps_ok = true; in.gps_speed = 0.5f;
    run(est, in, 500);
    est.reset();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.speed());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.x());
    TEST_ASSERT_EQUAL(StateEstimator::REF_NONE, est.headingRef());
}

void test_reset_odom_keeps_heading(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.mag_ok = true; in.mag_heading = 90.0f; in.gps_ok = true; in.gps_speed = 0.5f;
    run(est, in, 500);
    float heading = est.headingDeg();
    est.resetOdom();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.x());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, heading, est.headingDeg());
}

void test_zero_dt_is_ignored(void) {
    StateEstimator est = make(4);
    StateEstimator::Inputs in = base_inputs();
    in.dt = 0.0f;
    in.gps_ok = true; in.gps_speed = 1.0f;
    run(est, in, 10);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, est.speed());
}

int run_estimator_tests(void) {
    RUN_TEST(test_wrap360_positive);
    RUN_TEST(test_wrap360_negative);
    RUN_TEST(test_ang_err_wraps_short_way);
    RUN_TEST(test_wrap_pi_negative);
    RUN_TEST(test_compass_gives_heading_while_standing_still);
    RUN_TEST(test_gps_course_wins_while_moving);
    RUN_TEST(test_falls_back_to_imu_magnetometer_when_compass_dies);
    RUN_TEST(test_no_absolute_source_is_reported);
    RUN_TEST(test_health_flags_report_missing_sensors);
    RUN_TEST(test_gyro_carries_heading_when_everything_absolute_is_gone);
    RUN_TEST(test_imu_dead_uses_gps_course_directly);
    RUN_TEST(test_imu_and_gps_dead_uses_compass_directly);
    RUN_TEST(test_mode0_uses_imu_yaw_as_absolute);
    RUN_TEST(test_mode2_gps_only);
    RUN_TEST(test_compass_ignored_while_driving_fast);
    RUN_TEST(test_speed_follows_gps);
    RUN_TEST(test_speed_falls_back_to_model_without_gps);
    RUN_TEST(test_speed_source_without_imu);
    RUN_TEST(test_speed_is_clamped);
    RUN_TEST(test_small_speed_is_squelched);
    RUN_TEST(test_acceleration_deadband);
    RUN_TEST(test_wheel_speeds_equal_when_driving_straight);
    RUN_TEST(test_wheel_speeds_split_when_turning);
    RUN_TEST(test_rpm_conversion);
    RUN_TEST(test_odometry_moves_north_when_heading_north);
    RUN_TEST(test_odometry_moves_east_when_heading_east);
    RUN_TEST(test_heading_enu_conversion);
    RUN_TEST(test_reset_clears_everything);
    RUN_TEST(test_reset_odom_keeps_heading);
    RUN_TEST(test_zero_dt_is_ignored);
    return 0;
}

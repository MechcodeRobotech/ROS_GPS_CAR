/**
 * DriveController tests: heading hold, spin on the spot, turn-to-heading,
 * manual jog, PWM saturation and the motor acceleration ramp.
 */
#include <unity.h>
#include "Arduino.h"
#include "config.h"
#include "drive_controller.h"
#include "state_estimator.h"

/** Estimator pinned to a fixed heading/speed, so the controller is tested alone. */
static StateEstimator fixed_state(float heading_deg, float speed_mps) {
    StateEstimator est;
    StateEstimator::Config cfg;
    cfg.heading_source = 4;
    cfg.mag_tau_s = 0.001f;
    cfg.align_max_dps = 100000.0f;
    cfg.gps_tau_s = 0.001f;
    est.begin(cfg);

    StateEstimator::Inputs in;
    in.dt = 0.01f;
    in.imu_ok = true;
    in.mag_ok = true;
    in.mag_heading = heading_deg;
    in.gps_ok = true;
    in.gps_speed = speed_mps;
    for (int i = 0; i < 400; ++i) est.update(in);
    return est;
}

static DriveController make_drive() {
    DriveController drive;
    drive.begin();
    return drive;
}

static void spin_cycles(DriveController &drive, const StateEstimator &est, int n, float dt = 0.01f) {
    for (int i = 0; i < n; ++i) { mock_advance_ms((unsigned long)(dt * 1000.0f)); drive.update(dt, est); }
}

// ------------------------------------------------------------------- basics
void test_starts_stopped(void) {
    DriveController drive = make_drive();
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmRight());
}

void test_zero_speed_keeps_motors_off(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setAuto(0.0f, 0.0f);
    spin_cycles(drive, est, 50);
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmRight());
}

void test_drives_forward_on_target_heading(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.2f);
    drive.setAuto(0.25f, 0.0f);
    spin_cycles(drive, est, 300);
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    TEST_ASSERT_TRUE(drive.pwmRight() > 0);
    TEST_ASSERT_INT_WITHIN(120, drive.pwmLeft(), drive.pwmRight());   // roughly straight
}

void test_heading_error_steers_right(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.2f);       // pointing north
    drive.setAuto(0.25f, 20.0f);                        // want north-east, small error
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmLeft() > drive.pwmRight());   // left faster = turn clockwise
}

void test_heading_error_steers_left(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.2f);
    drive.setAuto(0.25f, 340.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmRight() > drive.pwmLeft());
}

void test_large_error_spins_on_the_spot(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setAuto(0.25f, 120.0f);                        // 120 deg away
    spin_cycles(drive, est, 100);
    TEST_ASSERT_TRUE(drive.spinning());
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    TEST_ASSERT_TRUE(drive.pwmRight() < 0);              // counter rotating
}

void test_spin_direction_is_shortest_way(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(10.0f, 0.0f);
    drive.setAuto(0.25f, 300.0f);                        // shorter to go anticlockwise
    spin_cycles(drive, est, 100);
    TEST_ASSERT_TRUE(drive.pwmRight() > 0);
    TEST_ASSERT_TRUE(drive.pwmLeft() < 0);
}

void test_pwm_never_exceeds_limit(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);        // measured speed 0, huge error
    drive.setAuto(0.42f, 0.0f);
    spin_cycles(drive, est, 500);
    TEST_ASSERT_TRUE(drive.pwmLeft() <= PWM_MAX);
    TEST_ASSERT_TRUE(drive.pwmRight() <= PWM_MAX);
    TEST_ASSERT_TRUE(drive.pwmLeft() >= -PWM_MAX);
}

// ------------------------------------------------------------ turn in place
void test_turn_to_heading_rotates(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setTurnTo(90.0f);
    TEST_ASSERT_TRUE(drive.turningInPlace());
    spin_cycles(drive, est, 50);
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    TEST_ASSERT_TRUE(drive.pwmRight() < 0);
}

void test_turn_to_heading_finishes_inside_tolerance(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(89.0f, 0.0f);       // already almost there
    drive.setTurnTo(90.0f);
    spin_cycles(drive, est, 100);                        // longer than the settle time
    TEST_ASSERT_FALSE(drive.turningInPlace());
    TEST_ASSERT_TRUE(drive.turnDone());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
}

void test_turn_to_heading_needs_no_speed_command(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setTurnTo(180.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, drive.targetSpeed());
    spin_cycles(drive, est, 50);
    TEST_ASSERT_TRUE(drive.pwmLeft() != 0);              // still rotates
}

void test_stop_cancels_turn(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setTurnTo(180.0f);
    drive.stop();
    TEST_ASSERT_FALSE(drive.turningInPlace());
    spin_cycles(drive, est, 50);
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
}

// ------------------------------------------------------------------ manual
void test_manual_forward(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.manual());
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    TEST_ASSERT_EQUAL_INT(drive.pwmLeft(), drive.pwmRight());
}

void test_manual_backward(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(-0.2f, 0.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmLeft() < 0);
    TEST_ASSERT_TRUE(drive.pwmRight() < 0);
}

void test_manual_turn_right(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.0f, 45.0f);                        // clockwise
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmLeft() > drive.pwmRight());
}

void test_manual_turn_left(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.0f, -45.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmRight() > drive.pwmLeft());
}

void test_manual_zero_stops(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.2f, 0.0f);
    spin_cycles(drive, est, 100);
    drive.setManual(0.0f, 0.0f);
    spin_cycles(drive, est, 300);
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmRight());
}

// -------------------------------------------------------------- accel ramp
void test_ramp_does_not_jump_to_full_power(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.6f, 0.3f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 1);                          // one 10 ms tick
    TEST_ASSERT_TRUE(drive.pwmLeft() <= PWM_MIN_MOVE + 30);
}

void test_ramp_reaches_target_after_the_ramp_time(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.6f, 0.3f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 200);                        // 2 s > 0.6 s ramp
    TEST_ASSERT_TRUE(drive.pwmLeft() > PWM_MAX / 2);
}

void test_ramp_breakaway_avoids_stiction(void) {
    DriveController drive = make_drive();
    drive.setRamp(2.0f, 1.0f);                           // very slow ramp
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 1);
    TEST_ASSERT_TRUE(drive.pwmLeft() >= PWM_MIN_MOVE);   // jumped straight to breakaway
}

void test_ramp_down_to_zero(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.6f, 0.3f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 200);
    drive.setManual(0.0f, 0.0f);
    spin_cycles(drive, est, 5);
    int mid = drive.pwmLeft();
    TEST_ASSERT_TRUE(mid > 0);                           // not an instant cut
    spin_cycles(drive, est, 200);
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
}

void test_ramp_zero_time_is_instant(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.0f, 0.0f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmLeft() > PWM_MAX / 2);
}

void test_direction_reversal_passes_through_zero(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.6f, 0.3f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 200);
    TEST_ASSERT_TRUE(drive.pwmLeft() > 0);
    drive.setManual(-0.42f, 0.0f);
    bool saw_zero_or_less = false;
    for (int i = 0; i < 200; ++i) {
        spin_cycles(drive, est, 1);
        if (drive.pwmLeft() <= 0) { saw_zero_or_less = true; break; }
    }
    TEST_ASSERT_TRUE(saw_zero_or_less);
}

void test_model_speed_follows_pwm(void) {
    DriveController drive = make_drive();
    drive.setRamp(0.0f, 0.0f);
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.0f, 0.0f);
    spin_cycles(drive, est, 10);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, drive.modelSpeedMps());
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 300);
    TEST_ASSERT_TRUE(drive.modelSpeedMps() > 0.1f);
}

void test_reset_clears_outputs(void) {
    DriveController drive = make_drive();
    StateEstimator est = fixed_state(0.0f, 0.0f);
    drive.setManual(0.42f, 0.0f);
    spin_cycles(drive, est, 100);
    drive.reset();
    TEST_ASSERT_EQUAL_INT(0, drive.pwmLeft());
    TEST_ASSERT_EQUAL_INT(0, drive.pwmRight());
}

void test_target_accessors(void) {
    DriveController drive = make_drive();
    drive.setAuto(0.3f, 123.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.3f, drive.targetSpeed());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 123.0f, drive.targetHeading());
    drive.setManual(0.1f, 30.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, drive.targetYawRate());
}

void test_setauto_leaves_manual_mode(void) {
    DriveController drive = make_drive();
    drive.setManual(0.1f, 0.0f);
    TEST_ASSERT_TRUE(drive.manual());
    drive.setAuto(0.1f, 0.0f);
    TEST_ASSERT_FALSE(drive.manual());
}

int run_drive_tests(void) {
    RUN_TEST(test_starts_stopped);
    RUN_TEST(test_zero_speed_keeps_motors_off);
    RUN_TEST(test_drives_forward_on_target_heading);
    RUN_TEST(test_heading_error_steers_right);
    RUN_TEST(test_heading_error_steers_left);
    RUN_TEST(test_large_error_spins_on_the_spot);
    RUN_TEST(test_spin_direction_is_shortest_way);
    RUN_TEST(test_pwm_never_exceeds_limit);
    RUN_TEST(test_turn_to_heading_rotates);
    RUN_TEST(test_turn_to_heading_finishes_inside_tolerance);
    RUN_TEST(test_turn_to_heading_needs_no_speed_command);
    RUN_TEST(test_stop_cancels_turn);
    RUN_TEST(test_manual_forward);
    RUN_TEST(test_manual_backward);
    RUN_TEST(test_manual_turn_right);
    RUN_TEST(test_manual_turn_left);
    RUN_TEST(test_manual_zero_stops);
    RUN_TEST(test_ramp_does_not_jump_to_full_power);
    RUN_TEST(test_ramp_reaches_target_after_the_ramp_time);
    RUN_TEST(test_ramp_breakaway_avoids_stiction);
    RUN_TEST(test_ramp_down_to_zero);
    RUN_TEST(test_ramp_zero_time_is_instant);
    RUN_TEST(test_direction_reversal_passes_through_zero);
    RUN_TEST(test_model_speed_follows_pwm);
    RUN_TEST(test_reset_clears_outputs);
    RUN_TEST(test_target_accessors);
    RUN_TEST(test_setauto_leaves_manual_mode);
    return 0;
}

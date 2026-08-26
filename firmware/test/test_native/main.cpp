/**
 * Native test runner:  pio test -e native
 *
 * Covers the logic that decides how the robot behaves - controller, estimator,
 * safety chain and the NMEA parser. Anything that needs real I2C/UART/PWM
 * silicon is not tested here; that is what the bench checklist in
 * docs/acceptance.md is for.
 */
#include <unity.h>

int run_pidf_tests(void);
int run_estimator_tests(void);
int run_drive_tests(void);
int run_motor_model_tests(void);
int run_safety_gps_tests(void);
int run_fusion_tests(void);
int run_pose_ekf_tests(void);

int main(int, char **) {
    UNITY_BEGIN();
    run_pidf_tests();
    run_estimator_tests();
    run_drive_tests();
    run_motor_model_tests();
    run_safety_gps_tests();
    run_fusion_tests();
    run_pose_ekf_tests();
    return UNITY_END();
}

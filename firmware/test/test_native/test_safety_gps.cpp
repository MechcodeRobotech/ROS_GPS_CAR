/**
 * SafetyManager and GPS NMEA parser tests - the two pieces where a bug is
 * either dangerous (safety) or silent (parser).
 */
#include <unity.h>
#include <cstring>
#include "Arduino.h"
#include "config.h"
#include "safety_manager.h"

// ============================== SafetyManager ==============================
static SafetyManager::Inputs healthy() {
    SafetyManager::Inputs in;
    in.sw_estop = false;
    in.heartbeat_age_ms = 50;
    in.cmd_age_ms = 50;
    in.agent_ok = true;
    in.battery_v = 12.0f;
    in.battery_valid = true;
    in.motion_requested = true;
    return in;
}

/** Boot the manager and let the start-up lockout expire. */
static void arm(SafetyManager &safety) {
    safety.begin();
    for (int i = 0; i < 40; ++i) { mock_advance_ms(100); safety.update(healthy()); }
}

static void set_hardware_clear() {
    mock_set_pin(PIN_E_EMER, E_EMER_ACTIVE_LOW ? HIGH : LOW);
    mock_set_pin(PIN_ESP_EMER_SW, ESP_EMER_SW_ACTIVE_LOW ? HIGH : LOW);
}

void test_safety_blocks_during_startup_lockout(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    safety.begin();
    mock_advance_ms(100);
    safety.update(healthy());
    TEST_ASSERT_FALSE(safety.allowMotion());
    TEST_ASSERT_TRUE(safety.flags() & STOP_STARTUP);
}

void test_safety_allows_motion_when_everything_is_fine(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    TEST_ASSERT_TRUE(safety.allowMotion());
    TEST_ASSERT_EQUAL_UINT32(0, safety.flags());
}

void test_safety_stops_on_heartbeat_timeout(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    SafetyManager::Inputs in = healthy();
    in.heartbeat_age_ms = HEARTBEAT_TIMEOUT_MS + 100;
    mock_advance_ms(10);
    safety.update(in);
    TEST_ASSERT_FALSE(safety.allowMotion());
    TEST_ASSERT_TRUE(safety.flags() & STOP_HEARTBEAT_LOST);
    TEST_ASSERT_TRUE(safety.linkLost());
}

void test_safety_stops_on_command_timeout_only_when_moving(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);

    SafetyManager::Inputs idle = healthy();
    idle.cmd_age_ms = CMD_TIMEOUT_MS + 500;
    idle.motion_requested = false;
    mock_advance_ms(10);
    safety.update(idle);
    TEST_ASSERT_FALSE(safety.flags() & STOP_CMD_TIMEOUT);   // parked, nothing to stop

    SafetyManager::Inputs moving = idle;
    moving.motion_requested = true;
    mock_advance_ms(10);
    safety.update(moving);
    TEST_ASSERT_TRUE(safety.flags() & STOP_CMD_TIMEOUT);
}

void test_safety_stops_when_agent_is_lost(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    SafetyManager::Inputs in = healthy();
    in.agent_ok = false;
    mock_advance_ms(10);
    safety.update(in);
    TEST_ASSERT_TRUE(safety.flags() & STOP_AGENT_LOST);
    TEST_ASSERT_FALSE(safety.allowMotion());
}

void test_safety_software_estop(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    SafetyManager::Inputs in = healthy();
    in.sw_estop = true;
    mock_advance_ms(10);
    safety.update(in);
    TEST_ASSERT_TRUE(safety.flags() & STOP_SW_ESTOP);
    TEST_ASSERT_TRUE(safety.emergencyLatched());
    TEST_ASSERT_FALSE(safety.allowMotion());
}

void test_safety_hardware_emergency_input(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    mock_set_pin(PIN_E_EMER, E_EMER_ACTIVE_LOW ? LOW : HIGH);
    for (int i = 0; i < 5; ++i) { mock_advance_ms(20); safety.update(healthy()); }
    TEST_ASSERT_TRUE(safety.flags() & STOP_HW_EMERGENCY);
    TEST_ASSERT_TRUE(safety.hwEmergency());
}

void test_safety_button_is_debounced(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    mock_set_pin(PIN_ESP_EMER_SW, ESP_EMER_SW_ACTIVE_LOW ? LOW : HIGH);
    mock_advance_ms(5);
    safety.update(healthy());
    TEST_ASSERT_FALSE(safety.espButton());          // too short, bounced
    for (int i = 0; i < 5; ++i) { mock_advance_ms(20); safety.update(healthy()); }
    TEST_ASSERT_TRUE(safety.espButton());
}

void test_safety_release_needs_hold_time(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    SafetyManager::Inputs in = healthy();
    in.sw_estop = true;
    mock_advance_ms(10);
    safety.update(in);
    TEST_ASSERT_FALSE(safety.allowMotion());

    in.sw_estop = false;
    mock_advance_ms(10);
    safety.update(in);
    TEST_ASSERT_FALSE(safety.allowMotion());        // must stay clear for a while
    for (int i = 0; i < 10; ++i) { mock_advance_ms(100); safety.update(in); }
    TEST_ASSERT_TRUE(safety.allowMotion());
}

void test_safety_battery_cutoff_needs_to_persist(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    SafetyManager::Inputs in = healthy();
    in.battery_v = BATT_CUTOFF_V - 0.5f;

    mock_advance_ms(100);
    safety.update(in);
    TEST_ASSERT_FALSE(safety.flags() & STOP_BATTERY_LOW);      // a load dip is ignored

    for (int i = 0; i < 60; ++i) { mock_advance_ms(100); safety.update(in); }
    TEST_ASSERT_TRUE(safety.flags() & STOP_BATTERY_LOW);
    TEST_ASSERT_TRUE(safety.batteryLatched());
}

void test_safety_battery_recovers_above_hysteresis(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    SafetyManager::Inputs in = healthy();
    in.battery_v = BATT_CUTOFF_V - 0.5f;
    for (int i = 0; i < 60; ++i) { mock_advance_ms(100); safety.update(in); }
    TEST_ASSERT_TRUE(safety.batteryLatched());

    in.battery_v = BATT_CUTOFF_V + 1.0f;
    for (int i = 0; i < 10; ++i) { mock_advance_ms(100); safety.update(in); }
    TEST_ASSERT_FALSE(safety.batteryLatched());
}

void test_safety_invalid_battery_reading_is_ignored(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    SafetyManager::Inputs in = healthy();
    in.battery_valid = false;
    in.battery_v = 0.0f;                                        // sensor missing
    for (int i = 0; i < 60; ++i) { mock_advance_ms(100); safety.update(in); }
    TEST_ASSERT_FALSE(safety.flags() & STOP_BATTERY_LOW);
    TEST_ASSERT_TRUE(safety.allowMotion());
}

void test_safety_run_state_mapping(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    TEST_ASSERT_EQUAL(RUN_IDLE, safety.runState(false, false));
    TEST_ASSERT_EQUAL(RUN_AUTO, safety.runState(true, false));
    TEST_ASSERT_EQUAL(RUN_MANUAL, safety.runState(false, true));

    SafetyManager::Inputs in = healthy();
    in.sw_estop = true;
    mock_advance_ms(10);
    safety.update(in);
    TEST_ASSERT_EQUAL(RUN_ESTOP, safety.runState(true, false));

    in.sw_estop = false;
    in.heartbeat_age_ms = HEARTBEAT_TIMEOUT_MS + 100;
    mock_advance_ms(10);
    safety.update(in);
    TEST_ASSERT_EQUAL(RUN_COMM_LOSS, safety.runState(true, false));
}

void test_safety_multiple_flags_at_once(void) {
    mock_reset_clock();
    set_hardware_clear();
    SafetyManager safety;
    arm(safety);
    SafetyManager::Inputs in = healthy();
    in.sw_estop = true;
    in.agent_ok = false;
    in.heartbeat_age_ms = 5000;
    mock_advance_ms(10);
    safety.update(in);
    TEST_ASSERT_TRUE(safety.flags() & STOP_SW_ESTOP);
    TEST_ASSERT_TRUE(safety.flags() & STOP_AGENT_LOST);
    TEST_ASSERT_TRUE(safety.flags() & STOP_HEARTBEAT_LOST);
}

// ================================ NMEA parser ==============================
// The parser reads from a HardwareSerial; for the native build we feed it a
// fake serial that replays a canned sentence stream.
#include "fake_serial.h"
#define HardwareSerial FakeSerial
#include "gps_nmea.h"
#undef HardwareSerial

static void feed(GpsNmea &gps, FakeSerial &port, const char *sentence) {
    port.push(sentence);
    gps.update();
}

void test_nmea_parses_gga_position(void) {
    FakeSerial port;
    GpsNmea gps;
    gps.begin(&port, 16, 17, 115200, false);
    feed(gps, port, "$GPGGA,123519,1344.2030,N,10031.3911,E,1,08,0.9,545.4,M,46.9,M,,*4F\r\n");
    TEST_ASSERT_TRUE(gps.valid());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 13.737383, gps.latitude());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 100.523185, gps.longitude());
    TEST_ASSERT_EQUAL_UINT8(8, gps.satellites());
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.9f, gps.hdop());
}

void test_nmea_rejects_bad_checksum(void) {
    FakeSerial port;
    GpsNmea gps;
    gps.begin(&port, 16, 17, 115200, false);
    feed(gps, port, "$GPGGA,123519,1344.2030,N,10031.3911,E,1,08,0.9,545.4,M,46.9,M,,*FF\r\n");
    TEST_ASSERT_FALSE(gps.valid());
    TEST_ASSERT_EQUAL_UINT32(1, gps.sentencesBad());
}

void test_nmea_no_fix_is_not_valid(void) {
    FakeSerial port;
    GpsNmea gps;
    gps.begin(&port, 16, 17, 115200, false);
    feed(gps, port, "$GPGGA,123519,,,,,0,00,99.9,,M,,M,,*7C\r\n");
    TEST_ASSERT_FALSE(gps.valid());
}

void test_nmea_parses_rmc_speed_and_course(void) {
    FakeSerial port;
    GpsNmea gps;
    gps.begin(&port, 16, 17, 115200, false);
    feed(gps, port, "$GPRMC,123519,A,1344.2030,N,10031.3911,E,1.94,84.4,230394,,*11\r\n");
    TEST_ASSERT_TRUE(gps.valid());
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, gps.speedMps());      // 1.94 knots ~ 1 m/s
    TEST_ASSERT_TRUE(gps.courseValid());
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 84.4f, gps.courseDeg());
}

void test_nmea_rmc_void_clears_fix(void) {
    FakeSerial port;
    GpsNmea gps;
    gps.begin(&port, 16, 17, 115200, false);
    feed(gps, port, "$GPGGA,123519,1344.2030,N,10031.3911,E,1,08,0.9,545.4,M,46.9,M,,*4F\r\n");
    TEST_ASSERT_TRUE(gps.valid());
    feed(gps, port, "$GPRMC,123520,V,,,,,,,230394,,*39\r\n");
    TEST_ASSERT_FALSE(gps.valid());
}

void test_nmea_southern_and_western_hemisphere(void) {
    FakeSerial port;
    GpsNmea gps;
    gps.begin(&port, 16, 17, 115200, false);
    feed(gps, port, "$GPGGA,123519,3350.0000,S,15112.0000,W,1,08,0.9,10.0,M,0.0,M,,*43\r\n");
    TEST_ASSERT_TRUE(gps.latitude() < 0.0);
    TEST_ASSERT_TRUE(gps.longitude() < 0.0);
}

void test_nmea_ignores_garbage(void) {
    FakeSerial port;
    GpsNmea gps;
    gps.begin(&port, 16, 17, 115200, false);
    feed(gps, port, "random noise without a dollar sign\r\n");
    TEST_ASSERT_FALSE(gps.valid());
    TEST_ASSERT_EQUAL_UINT32(0, gps.sentencesOk());
}

void test_nmea_survives_overlong_line(void) {
    FakeSerial port;
    GpsNmea gps;
    gps.begin(&port, 16, 17, 115200, false);
    char big[400];
    big[0] = '$';
    memset(big + 1, 'A', sizeof(big) - 3);
    big[sizeof(big) - 2] = '\n';
    big[sizeof(big) - 1] = '\0';
    port.push(big);
    for (int i = 0; i < 10; ++i) gps.update();               // must not crash or hang
    feed(gps, port, "$GPGGA,123519,1344.2030,N,10031.3911,E,1,08,0.9,545.4,M,46.9,M,,*4F\r\n");
    TEST_ASSERT_TRUE(gps.valid());                           // recovers on the next sentence
}

void test_nmea_freshness_expires(void) {
    FakeSerial port;
    GpsNmea gps;
    mock_reset_clock();
    gps.begin(&port, 16, 17, 115200, false);
    feed(gps, port, "$GPGGA,123519,1344.2030,N,10031.3911,E,1,08,0.9,545.4,M,46.9,M,,*4F\r\n");
    TEST_ASSERT_TRUE(gps.fresh(3000));
    mock_advance_ms(4000);
    TEST_ASSERT_FALSE(gps.fresh(3000));
}

void test_nmea_gnss_talker_ids_accepted(void) {
    FakeSerial port;
    GpsNmea gps;
    gps.begin(&port, 16, 17, 115200, false);
    feed(gps, port, "$GNGGA,123519,1344.2030,N,10031.3911,E,1,11,0.8,545.4,M,46.9,M,,*58\r\n");
    TEST_ASSERT_TRUE(gps.valid());
    TEST_ASSERT_EQUAL_UINT8(11, gps.satellites());
}

void test_nmea_vtg_updates_speed(void) {
    FakeSerial port;
    GpsNmea gps;
    gps.begin(&port, 16, 17, 115200, false);
    feed(gps, port, "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48\r\n");
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.83f, gps.speedMps());     // 10.2 km/h
    TEST_ASSERT_TRUE(gps.courseValid());
}

void test_nmea_partial_sentence_then_rest(void) {
    FakeSerial port;
    GpsNmea gps;
    gps.begin(&port, 16, 17, 115200, false);
    port.push("$GPGGA,123519,1344.2030,N,100");
    gps.update();
    TEST_ASSERT_FALSE(gps.valid());
    // checksum is over the whole sentence, both halves together:
    // GPGGA,123519,1344.2030,N,10031.3911,E,1,08,0.9,545.4,M,46.9,M,, -> 4F
    port.push("31.3911,E,1,08,0.9,545.4,M,46.9,M,,*4F\r\n");
    gps.update();
    TEST_ASSERT_TRUE(gps.valid());
}

int run_safety_gps_tests(void) {
    RUN_TEST(test_safety_blocks_during_startup_lockout);
    RUN_TEST(test_safety_allows_motion_when_everything_is_fine);
    RUN_TEST(test_safety_stops_on_heartbeat_timeout);
    RUN_TEST(test_safety_stops_on_command_timeout_only_when_moving);
    RUN_TEST(test_safety_stops_when_agent_is_lost);
    RUN_TEST(test_safety_software_estop);
    RUN_TEST(test_safety_hardware_emergency_input);
    RUN_TEST(test_safety_button_is_debounced);
    RUN_TEST(test_safety_release_needs_hold_time);
    RUN_TEST(test_safety_battery_cutoff_needs_to_persist);
    RUN_TEST(test_safety_battery_recovers_above_hysteresis);
    RUN_TEST(test_safety_invalid_battery_reading_is_ignored);
    RUN_TEST(test_safety_run_state_mapping);
    RUN_TEST(test_safety_multiple_flags_at_once);

    RUN_TEST(test_nmea_parses_gga_position);
    RUN_TEST(test_nmea_rejects_bad_checksum);
    RUN_TEST(test_nmea_no_fix_is_not_valid);
    RUN_TEST(test_nmea_parses_rmc_speed_and_course);
    RUN_TEST(test_nmea_rmc_void_clears_fix);
    RUN_TEST(test_nmea_southern_and_western_hemisphere);
    RUN_TEST(test_nmea_ignores_garbage);
    RUN_TEST(test_nmea_survives_overlong_line);
    RUN_TEST(test_nmea_freshness_expires);
    RUN_TEST(test_nmea_gnss_talker_ids_accepted);
    RUN_TEST(test_nmea_vtg_updates_speed);
    RUN_TEST(test_nmea_partial_sentence_then_rest);
    return 0;
}

/**
 * @file safety_manager.h
 * @brief Collects every stop condition into one bitfield and drives the red LED.
 *
 * The manager never touches the motors itself - main.cpp asks allowMotion()
 * on every control cycle. Keeping the decision in one place means a new stop
 * source only has to be added here.
 *
 * Red LED (IO2)   : solid = emergency latched, fast blink = link/command lost,
 *                   slow blink = battery cutoff, short blip = healthy.
 * Green LED       : wired straight to the 5 V rail after the ON MODULE switch.
 *                   It shows "module powered" and is intentionally NOT driven
 *                   by firmware, so it still lights if the ESP32 hangs.
 */
#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include <Arduino.h>
#include <config.h>

class SafetyManager {
public:
  struct Inputs {
    bool     sw_estop        = false;   // software e-stop latched by the web UI
    uint32_t heartbeat_age_ms = 0xFFFFFFFF;
    uint32_t cmd_age_ms       = 0xFFFFFFFF;
    bool     agent_ok        = false;
    float    battery_v       = 0.0f;
    bool     battery_valid   = false;
    bool     motion_requested = false;  // a non-zero command is pending
  };

  void begin() {
    pinMode(PIN_E_EMER, INPUT);
    pinMode(PIN_ESP_EMER_SW, INPUT_PULLUP);
    if (PIN_LED_RED >= 0) { pinMode(PIN_LED_RED, OUTPUT); digitalWrite(PIN_LED_RED, LOW); }
    if (PIN_LED_GREEN >= 0) { pinMode(PIN_LED_GREEN, OUTPUT); digitalWrite(PIN_LED_GREEN, HIGH); }
    if (PIN_FAN_CTRL >= 0) { pinMode(PIN_FAN_CTRL, OUTPUT); digitalWrite(PIN_FAN_CTRL, HIGH); }
    boot_ms_ = millis();
    last_clear_ms_ = 0;
  }

  void update(const Inputs &in) {
    uint32_t now = millis();
    uint32_t f = 0;

    // --- hardware inputs (debounced) ---
    bool hw_raw  = readEmergencyLine();
    bool btn_raw = readEspButton();
    hw_emer_  = debounce(hw_raw,  hw_state_,  hw_change_ms_,  now);
    esp_btn_  = debounce(btn_raw, btn_state_, btn_change_ms_, now);

    if (hw_emer_)  f |= STOP_HW_EMERGENCY;
    if (esp_btn_)  f |= STOP_ESP_BUTTON;
    if (in.sw_estop) f |= STOP_SW_ESTOP;

    // --- link / command supervision ---
    if (!in.agent_ok)                                    f |= STOP_AGENT_LOST;
    if (in.heartbeat_age_ms > HEARTBEAT_TIMEOUT_MS)      f |= STOP_HEARTBEAT_LOST;
    if (in.motion_requested && in.cmd_age_ms > CMD_TIMEOUT_MS) f |= STOP_CMD_TIMEOUT;

    // --- battery protection (must stay low, ignore short load dips) ---
    //
    // battery_valid false means the pack voltage could not be MEASURED - the
    // motor rail is unpowered because the mushroom switch or the relay is open.
    // That is not a flat battery, so nothing latches. Reading the ESP32 +5 V
    // rail here by mistake is exactly what latched the robot into e-stop three
    // seconds after every boot; see power.h.
    if (in.battery_valid && in.battery_v < BATT_CUTOFF_V) {
      if (batt_low_since_ == 0) batt_low_since_ = now;
      if (now - batt_low_since_ > BATT_CUTOFF_HOLD_MS) batt_latched_ = true;
    } else {
      batt_low_since_ = 0;
      if (in.battery_valid && in.battery_v > BATT_CUTOFF_CLEAR_V) batt_latched_ = false;
    }
    if (batt_latched_) f |= STOP_BATTERY_LOW;

    // Soft stop: stop driving but stay powered, so the pack is not run down to
    // the hard cut on every outing. Not latched - it follows the voltage back
    // up once the load comes off.
    batt_soft_stop_ = in.battery_valid && in.battery_v < BATT_SOFT_STOP_V;
    if (batt_soft_stop_) f |= STOP_BATTERY_SOFT;

    // --- boot lockout ---
    if (now - boot_ms_ < STARTUP_LOCKOUT_MS) f |= STOP_STARTUP;

    // --- release needs the inputs to stay clear for a moment ---
    if (f == 0) {
      if (last_clear_ms_ == 0) last_clear_ms_ = now;
      allow_ = (now - last_clear_ms_) >= ESTOP_RELEASE_HOLD_MS;
    } else {
      last_clear_ms_ = 0;
      allow_ = false;
    }
    flags_ = f;
    serviceLed(now);
  }

  uint32_t flags() const { return flags_; }
  bool allowMotion() const { return allow_; }
  bool emergencyLatched() const {
    return (flags_ & (STOP_HW_EMERGENCY | STOP_ESP_BUTTON | STOP_SW_ESTOP)) != 0;
  }
  bool linkLost() const {
    return (flags_ & (STOP_HEARTBEAT_LOST | STOP_CMD_TIMEOUT | STOP_AGENT_LOST)) != 0;
  }
  bool batteryLatched() const { return batt_latched_; }
  bool batterySoftStop() const { return batt_soft_stop_; }
  bool hwEmergency() const { return hw_emer_; }
  bool espButton() const { return esp_btn_; }

  RunState runState(bool auto_mode, bool manual_mode) const {
    if (emergencyLatched() || batt_latched_) return RUN_ESTOP;
    if (linkLost())                          return RUN_COMM_LOSS;
    if (auto_mode)                           return RUN_AUTO;
    if (manual_mode)                         return RUN_MANUAL;
    return RUN_IDLE;
  }

private:
  static bool readEmergencyLine() {
    int v = digitalRead(PIN_E_EMER);
#if E_EMER_ACTIVE_LOW
    return v == LOW;
#else
    return v == HIGH;
#endif
  }

  static bool readEspButton() {
    int v = digitalRead(PIN_ESP_EMER_SW);
#if ESP_EMER_SW_ACTIVE_LOW
    return v == LOW;
#else
    return v == HIGH;
#endif
  }

  static bool debounce(bool raw, bool &state, uint32_t &change_ms, uint32_t now) {
    const uint32_t DEBOUNCE_MS = 25;
    if (raw != state) {
      if (change_ms == 0) change_ms = now;
      else if (now - change_ms >= DEBOUNCE_MS) { state = raw; change_ms = 0; }
    } else {
      change_ms = 0;
    }
    return state;
  }

  void serviceLed(uint32_t now) {
    if (PIN_LED_RED < 0) return;
    bool on;
    if (emergencyLatched()) {
      on = true;                                                    // solid
    } else if (linkLost()) {
      on = ((now / LED_BLINK_FAST_MS) & 1) != 0;                    // fast blink
    } else if (batt_latched_) {
      on = ((now / LED_BLINK_SLOW_MS) & 1) != 0;                    // slow blink
    } else {
      on = (now % LED_HEARTBEAT_MS) < 60;                           // healthy blip
    }
    if (on != led_on_) { led_on_ = on; digitalWrite(PIN_LED_RED, on ? HIGH : LOW); }
  }

  uint32_t flags_ = STOP_STARTUP;
  bool     allow_ = false;
  bool     hw_emer_ = false, esp_btn_ = false;
  bool     hw_state_ = false, btn_state_ = false;
  uint32_t hw_change_ms_ = 0, btn_change_ms_ = 0;
  uint32_t boot_ms_ = 0;
  uint32_t last_clear_ms_ = 0;
  uint32_t batt_low_since_ = 0;
  bool     batt_latched_ = false;
  bool     batt_soft_stop_ = false;
  bool     led_on_ = false;
};

#endif // SAFETY_MANAGER_H

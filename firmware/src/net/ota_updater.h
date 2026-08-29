/**
 * Firmware update over Wi-Fi, so a board that is bolted into a robot and
 * running on battery can be reflashed without finding a cable.
 *
 * This exists because of a real dead end: the relay was mis-wired in firmware,
 * the fix was one line, and it could not be applied. The robot was on battery
 * with no USB attached, and the running image had no way to accept an update.
 * A cable was the only route, which is exactly the situation OTA removes.
 *
 * WHAT IT WILL NOT DO
 *
 * It refuses to start an update unless the robot is standing still with its
 * motor rails dead. Rewriting the program while wheels are turning means the
 * motors keep their last duty cycle for the seconds it takes to flash and
 * reboot, with nothing running that could stop them. So the contactor must be
 * open before a single byte is accepted - and because the safety manager owns
 * the contactor, asking it is how we know.
 *
 * The password is not decoration. Anything on the network can reach port 3232,
 * and an unauthenticated OTA port is a way to put arbitrary code on a machine
 * that drives itself around. It comes from network_secrets.h, which is
 * gitignored, and the build fails if it is missing.
 *
 * ROLLBACK
 *
 * ArduinoOTA writes to the inactive OTA partition and only switches over once
 * the whole image has arrived and its MD5 matches. A partial or corrupt upload
 * leaves the running image untouched, so a dropped Wi-Fi link mid-flash costs
 * nothing but a retry. That is why the update lands as a normal reboot rather
 * than a bricked board.
 */
#ifndef OTA_UPDATER_H
#define OTA_UPDATER_H

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#include "config.h"

class OtaUpdater {
 public:
  /**
   * Advertise the board for wireless flashing.
   *
   * @param hostname   what the board answers to, e.g. "esp32-gps-localize"
   * @param is_safe    called before an update is allowed to begin. Return false
   *                   whenever the robot could move - the update is then
   *                   refused rather than deferred, so the person flashing gets
   *                   an error instead of a silent wait.
   * @param on_begin   called once an update has been accepted, to shut down
   *                   anything that must not run while flash is being rewritten.
   */
  void begin(const char *hostname, bool (*is_safe)(), void (*on_begin)()) {
    is_safe_  = is_safe;
    on_begin_ = on_begin;

    ArduinoOTA.setHostname(hostname);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([this]() {
      // Last gate before the flash is rewritten. Refusing here is the whole
      // point: an update that starts while the robot can move leaves the motors
      // holding their last command with no code running to change it.
      if (is_safe_ && !is_safe_()) {
        Serial.println("[OTA] refused - the robot is not in a safe state "
                       "(motor rails must be dead and the robot still)");
        // No clean "reject" exists in the callback, so make it fail loudly and
        // leave the running image alone rather than flash in an unsafe state.
        ArduinoOTA.end();
        ESP.restart();
        return;
      }
      if (on_begin_) on_begin_();
      updating_ = true;
      Serial.printf("[OTA] update starting (%s)\n",
                    ArduinoOTA.getCommand() == U_FLASH ? "firmware"
                                                       : "filesystem");
    });

    ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
      static int last_pct = -1;
      const int pct = total ? (int)((done * 100) / total) : 0;
      if (pct != last_pct && pct % 10 == 0) {
        last_pct = pct;
        Serial.printf("[OTA] %d%%\n", pct);
      }
    });

    ArduinoOTA.onEnd([this]() {
      updating_ = false;
      Serial.println("[OTA] image verified, rebooting into it");
    });

    ArduinoOTA.onError([this](ota_error_t error) {
      updating_ = false;
      // The running image is untouched: ArduinoOTA writes the inactive
      // partition and only switches over on a verified image.
      const char *why = "unknown";
      switch (error) {
        case OTA_AUTH_ERROR:    why = "wrong password"; break;
        case OTA_BEGIN_ERROR:   why = "could not begin - no space or bad partition"; break;
        case OTA_CONNECT_ERROR: why = "connection lost"; break;
        case OTA_RECEIVE_ERROR: why = "receive failed"; break;
        case OTA_END_ERROR:     why = "image failed verification"; break;
      }
      Serial.printf("[OTA] failed: %s - the running firmware is unchanged\n", why);
    });

    ArduinoOTA.begin();
    ready_ = true;
    Serial.printf("[OTA] ready - flash over Wi-Fi with:\n"
                  "      pio run -e wifi_ota -t upload   (board: %s / %s)\n",
                  hostname, WiFi.localIP().toString().c_str());
  }

  /** Service the OTA server. Cheap when no update is in progress. */
  void handle() {
    if (ready_) ArduinoOTA.handle();
  }

  /** True while an image is being received, so the caller can stay out of the way. */
  bool updating() const { return updating_; }

 private:
  bool (*is_safe_)()  = nullptr;
  void (*on_begin_)() = nullptr;
  bool ready_    = false;
  bool updating_ = false;
};

#endif  // OTA_UPDATER_H

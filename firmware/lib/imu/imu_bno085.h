/**
 * @file imu_bno085.h
 * @brief BNO085 wrapper: fused orientation, yaw rate and gravity-free acceleration.
 *
 * Three reports are enabled:
 *   - rotation vector      : quaternion + heading (game RV = no magnetometer)
 *   - gyroscope calibrated : yaw rate for the heading PID and wheel-speed split
 *   - linear acceleration  : forward acceleration for the speed estimator
 *
 * The BNO085 can silently reset (brown-out, ESD). wasReset() is checked on
 * every update and the reports are re-enabled automatically; ok() goes false
 * while no event has arrived for stale_ms so the safety layer can react.
 */
#ifndef IMU_BNO085_H
#define IMU_BNO085_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

class ImuBno085 {
public:
  ImuBno085(int reset_pin = -1) : bno_(reset_pin) {}

  /** Which orientation reports to enable. BOTH costs a little I2C bandwidth and
   *  buys a second, independent absolute heading source for the fallback chain. */
  enum Mode { MODE_GAME = 0, MODE_MAG = 1, MODE_BOTH = 2 };

  bool begin(TwoWire *wire, uint8_t addr, Mode mode,
             uint32_t report_interval_us = 10000) {
    wire_        = wire;
    addr_        = addr;
    mode_        = mode;
    use_mag_     = (mode == MODE_MAG);
    interval_us_ = report_interval_us;

    if (!bno_.begin_I2C(addr_, wire_)) { begun_ = false; ok_ = false; return false; }
    enableReports();
    begun_ = true;
    // NOT ok_ yet. begun_ means the sensor answered and the driver is
    // initialised; ok_ means the data is FRESH. Only the first event can make
    // the second true, and the BNO085 takes a while to produce one.
    ok_ = false;
    last_event_ms_ = millis();
    return true;
  }

  /** Service the sensor. Bounded: at most max_events per call. */
  bool update(uint8_t max_events = 8) {
    // Never touch the driver if begin() failed. The Adafruit BNO08x library
    // leaves its SHTP state uninitialised in that case, so getSensorEvent()
    // dereferences a null pointer and the ESP32 dies with
    //     Guru Meditation Error: LoadProhibited
    // inside shtp_service.
    //
    // Found on real hardware with no IMU attached: the board crashed and
    // rebooted roughly every 15 s, forever. It matters with an IMU fitted
    // too - a loose connector or an I2C fault at boot turns a robot that
    // should degrade into one that crash-loops.
    // Guard on begun_, NOT on ok_. Conflating the two cost the IMU entirely:
    // ok_ means "data is fresh", so the first gap longer than stale_ms_ made it
    // false, and this early return then stopped the driver polling the sensor
    // ever again. One late report at start-up latched the IMU dead until reboot.
    //
    // The BNO085 takes well over 500 ms to start reporting after
    // enableReport(), so that gap happened on EVERY boot. Seen on hardware: the
    // sensor answering at 0x4A, begin() succeeding, and sensor_health still
    // saying imu_missing forever afterwards.
    //
    // begun_ is what the crash guard actually needs anyway - it only has to
    // know begin() succeeded, so the driver's SHTP state is initialised.
    if (!begun_) return false;
    if (bno_.wasReset()) { resets_++; enableReports(); }

    bool got = false;
    while (max_events--) {
      if (!bno_.getSensorEvent(&evt_)) break;
      switch (evt_.sensorId) {
        case SH2_ROTATION_VECTOR: {
          // magnetometer-referenced: absolute, but the BNO085 sits near the motors
          float yaw = yawFromQuat(evt_.un.rotationVector.real, evt_.un.rotationVector.i,
                                  evt_.un.rotationVector.j, evt_.un.rotationVector.k);
          mag_yaw_deg_ = yaw;
          accuracy_ = evt_.un.rotationVector.accuracy;
          mag_yaw_ms_ = millis();
          if (use_mag_) setQuat(evt_.un.rotationVector.real, evt_.un.rotationVector.i,
                                evt_.un.rotationVector.j, evt_.un.rotationVector.k);
          got = true;
        } break;
        case SH2_GAME_ROTATION_VECTOR:
          // gyro/accel only: smooth and immune to magnets, but relative
          if (!use_mag_) {
            setQuat(evt_.un.gameRotationVector.real, evt_.un.gameRotationVector.i,
                    evt_.un.gameRotationVector.j, evt_.un.gameRotationVector.k);
            got = true;
          }
          break;
        case SH2_GYROSCOPE_CALIBRATED:
          gyro_x_ = evt_.un.gyroscope.x;
          gyro_y_ = evt_.un.gyroscope.y;
          gyro_z_ = evt_.un.gyroscope.z;
          got = true;
          break;
        case SH2_LINEAR_ACCELERATION:
          acc_x_ = evt_.un.linearAcceleration.x;
          acc_y_ = evt_.un.linearAcceleration.y;
          acc_z_ = evt_.un.linearAcceleration.z;
          got = true;
          break;
        default:
          break;
      }
    }
    if (got) last_event_ms_ = millis();
    ok_ = (millis() - last_event_ms_) < stale_ms_;
    return got;
  }

  bool  ok()        const { return ok_; }
  float yawDeg()    const { return yaw_deg_; }     // 0..360, CCW positive (ENU convention)
  float pitchDeg()  const { return pitch_deg_; }
  float rollDeg()   const { return roll_deg_; }
  float yawRateDps() const { return gyro_z_ * 57.29578f; }
  float gyroX()     const { return gyro_x_; }      // [rad/s]
  float gyroY()     const { return gyro_y_; }
  float gyroZ()     const { return gyro_z_; }
  float accX()      const { return acc_x_; }       // [m/s^2], gravity removed
  float accY()      const { return acc_y_; }
  float accZ()      const { return acc_z_; }
  float qw()        const { return qw_; }
  float qx()        const { return qx_; }
  float qy()        const { return qy_; }
  float qz()        const { return qz_; }
  uint8_t accuracy() const { return accuracy_; }   // 0 unreliable .. 3 high (rotation vector only)
  uint16_t resets()  const { return resets_; }
  void setStaleMs(uint32_t ms) { stale_ms_ = ms; }

  /** Absolute heading from the BNO085's own magnetometer - third source in the
   *  fallback chain, used when the QMC5883L is missing or disturbed. */
  float magYawDeg() const { return mag_yaw_deg_; }
  bool  magYawOk() const {
    return mag_yaw_ms_ != 0 && (millis() - mag_yaw_ms_) < stale_ms_ && accuracy_ >= 2;
  }

private:
  void enableReports() {
    if (mode_ != MODE_MAG)  bno_.enableReport(SH2_GAME_ROTATION_VECTOR, interval_us_);
    if (mode_ != MODE_GAME) bno_.enableReport(SH2_ROTATION_VECTOR, interval_us_ * 2);
    bno_.enableReport(SH2_GYROSCOPE_CALIBRATED, interval_us_);
    bno_.enableReport(SH2_LINEAR_ACCELERATION, interval_us_ * 2);
  }

  static float yawFromQuat(float w, float x, float y, float z) {
    float sqw = w * w, sqx = x * x, sqy = y * y, sqz = z * z;
    float yaw = atan2f(2.0f * (x * y + z * w), (sqx - sqy - sqz + sqw));
    return fmodf(yaw * 57.29578f + 360.0f, 360.0f);
  }

  void setQuat(float w, float x, float y, float z) {
    qw_ = w; qx_ = x; qy_ = y; qz_ = z;

    float sqw = w * w, sqx = x * x, sqy = y * y, sqz = z * z;
    float yaw   = atan2f(2.0f * (x * y + z * w), (sqx - sqy - sqz + sqw));
    float pitch = asinf(constrain(-2.0f * (x * z - y * w) / (sqx + sqy + sqz + sqw), -1.0f, 1.0f));
    float roll  = atan2f(2.0f * (y * z + x * w), (-sqx - sqy + sqz + sqw));

    yaw_deg_   = fmodf(yaw * 57.29578f + 360.0f, 360.0f);
    pitch_deg_ = pitch * 57.29578f;
    roll_deg_  = roll * 57.29578f;
  }

  Adafruit_BNO08x   bno_;
  sh2_SensorValue_t evt_;
  TwoWire *wire_ = nullptr;
  uint8_t  addr_ = 0x4A;
  Mode     mode_ = MODE_BOTH;
  bool     use_mag_ = false;
  float    mag_yaw_deg_ = 0.0f;
  uint32_t mag_yaw_ms_ = 0;
  uint32_t interval_us_ = 10000;
  uint32_t stale_ms_ = 500;
  uint32_t last_event_ms_ = 0;
  bool     begun_ = false;      // begin() succeeded; never cleared after
  uint16_t resets_ = 0;
  bool     ok_ = false;

  float qw_ = 1.0f, qx_ = 0.0f, qy_ = 0.0f, qz_ = 0.0f;
  float yaw_deg_ = 0.0f, pitch_deg_ = 0.0f, roll_deg_ = 0.0f;
  float gyro_x_ = 0.0f, gyro_y_ = 0.0f, gyro_z_ = 0.0f;
  float acc_x_ = 0.0f, acc_y_ = 0.0f, acc_z_ = 0.0f;
  uint8_t accuracy_ = 0;
};

#endif // IMU_BNO085_H

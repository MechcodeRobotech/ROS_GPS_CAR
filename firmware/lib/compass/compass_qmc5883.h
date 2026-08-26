/**
 * @file compass_qmc5883.h
 * @brief QMC5883L magnetometer (the one built into the GEP-M10-DQ GPS module).
 *
 * WHY IT IS HERE
 *   The GPS course over ground only exists while the robot is moving, and the
 *   BNO085 game rotation vector is relative. Without a magnetometer the robot
 *   does not know which way it is pointing until it has driven a few metres.
 *   This chip gives an ABSOLUTE heading while standing still, which is what
 *   the estimator uses to seed itself before the first move.
 *
 * WHY THIS ONE AND NOT THE BNO085 MAGNETOMETER
 *   It sits on the GPS antenna mast, far from the motors and the battery
 *   wires. The BNO085 lives next to both. You can still switch to the BNO085
 *   rotation vector with heading_source = 0.
 *
 * TILT COMPENSATION
 *   A flat compass reading is only correct when the robot is level. Roll and
 *   pitch from the BNO085 are used to project the field onto the horizontal
 *   plane, so a slope or a bumpy lawn does not swing the heading.
 *
 * INTERFERENCE CHECK
 *   The field strength is compared with the value learned during calibration.
 *   If it changes by more than the tolerance (motor current, a steel fence,
 *   a phone on the chassis) the reading is marked invalid instead of lying.
 *
 * No dynamic allocation, one I2C burst read per update.
 */
#ifndef COMPASS_QMC5883_H
#define COMPASS_QMC5883_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

class CompassQMC5883 {
public:
  static const uint8_t REG_DATA    = 0x00;   // X L/H, Y L/H, Z L/H
  static const uint8_t REG_STATUS  = 0x06;
  static const uint8_t REG_CTRL1   = 0x09;
  static const uint8_t REG_CTRL2   = 0x0A;
  static const uint8_t REG_SETRESET = 0x0B;
  static const uint8_t REG_CHIP_ID = 0x0D;   // reads 0xFF on a QMC5883L

  struct Calibration {
    float off_x = 0.0f, off_y = 0.0f, off_z = 0.0f;   // hard iron [counts]
    float scale_x = 1.0f, scale_y = 1.0f, scale_z = 1.0f;  // soft iron
    float field_norm = 0.0f;                          // expected |B| after correction
    bool  valid = false;
  };

  bool begin(TwoWire *wire, uint8_t addr = 0x0D) {
    wire_ = wire;
    addr_ = addr;

    uint8_t id = 0;
    if (!readReg(REG_CHIP_ID, id) || id != 0xFF) { present_ = false; return false; }

    writeReg(REG_CTRL2, 0x80);        // soft reset
    delay(10);
    writeReg(REG_SETRESET, 0x01);     // recommended by the datasheet
    // OSR 512 | range 8 G | ODR 100 Hz | continuous
    writeReg(REG_CTRL1, (0x00 << 6) | (0x01 << 4) | (0x02 << 2) | 0x01);
    delay(10);

    present_ = true;
    return true;
  }

  void setCalibration(const Calibration &cal) { cal_ = cal; }
  const Calibration &calibration() const { return cal_; }

  /** Mounting: angle added after the maths, and direction flip if the module is upside down. */
  void setMounting(float offset_deg, bool invert) { mount_offset_ = offset_deg; invert_ = invert; }
  void setDeclination(float deg) { declination_ = deg; }
  void setFieldTolerance(float fraction) { field_tol_ = fraction; }

  /**
   * Read the sensor and compute the tilt compensated heading.
   * @param roll_deg  roll from the IMU
   * @param pitch_deg pitch from the IMU
   * @return true when a fresh, trustworthy sample was produced
   */
  bool update(float roll_deg, float pitch_deg) {
    if (!present_) return false;

    int16_t rx, ry, rz;
    if (!readRaw(rx, ry, rz)) { fail(); return false; }

    raw_x_ = rx; raw_y_ = ry; raw_z_ = rz;
    if (calibrating_) captureMinMax(rx, ry, rz);

    const float mx = ((float)rx - cal_.off_x) * cal_.scale_x;
    const float my = ((float)ry - cal_.off_y) * cal_.scale_y;
    const float mz = ((float)rz - cal_.off_z) * cal_.scale_z;

    field_ = sqrtf(mx * mx + my * my + mz * mz);
    if (cal_.valid && cal_.field_norm > 1.0f) {
      const float ratio = field_ / cal_.field_norm;
      if (ratio < (1.0f - field_tol_) || ratio > (1.0f + field_tol_)) {
        disturbed_ = true;
        ok_ = false;
        return false;              // magnetic disturbance: better no reading than a wrong one
      }
    }
    disturbed_ = false;

    const float roll  = roll_deg  * 0.01745329f;
    const float pitch = pitch_deg * 0.01745329f;
    const float sr = sinf(roll),  cr = cosf(roll);
    const float sp = sinf(pitch), cp = cosf(pitch);

    const float xh = mx * cp + my * sr * sp + mz * cr * sp;
    const float yh = my * cr - mz * sr;

    float deg = atan2f(-yh, xh) * 57.29578f;          // compass sense: 0 = north, CW
    if (invert_) deg = -deg;
    deg += declination_ + mount_offset_;
    heading_deg_ = wrap(deg);

    last_ok_ms_ = millis();
    ok_ = true;
    errors_ = 0;
    return true;
  }

  bool  present() const { return present_; }
  bool  ok() const { return ok_ && (millis() - last_ok_ms_) < 500; }
  bool  disturbed() const { return disturbed_; }
  float headingDeg() const { return heading_deg_; }   // compass degrees, tilt compensated
  float field() const { return field_; }
  int16_t rawX() const { return raw_x_; }
  int16_t rawY() const { return raw_y_; }
  int16_t rawZ() const { return raw_z_; }

  //--------------------------- calibration ---------------------------------//
  /** Start capturing min/max. Turn the robot slowly through at least one full circle. */
  void startCalibration() {
    calibrating_ = true;
    min_x_ = min_y_ = min_z_ = 32767;
    max_x_ = max_y_ = max_z_ = -32768;
    samples_ = 0;
  }

  bool calibrating() const { return calibrating_; }
  uint16_t calibrationSamples() const { return samples_; }

  /** Stop capturing and apply the result. Returns false if the turn was too small. */
  bool finishCalibration() {
    calibrating_ = false;
    if (samples_ < 100) return false;

    const float span_x = (float)(max_x_ - min_x_);
    const float span_y = (float)(max_y_ - min_y_);
    const float span_z = (float)(max_z_ - min_z_);
    if (span_x < 200.0f || span_y < 200.0f) return false;   // barely turned

    Calibration cal;
    cal.off_x = (max_x_ + min_x_) * 0.5f;
    cal.off_y = (max_y_ + min_y_) * 0.5f;
    cal.off_z = (max_z_ + min_z_) * 0.5f;

    const float avg = (span_x + span_y + span_z) / 3.0f;
    cal.scale_x = (span_x > 1.0f) ? avg / span_x : 1.0f;
    cal.scale_y = (span_y > 1.0f) ? avg / span_y : 1.0f;
    cal.scale_z = (span_z > 1.0f) ? avg / span_z : 1.0f;
    cal.field_norm = avg * 0.5f;
    cal.valid = true;

    cal_ = cal;
    return true;
  }

private:
  void captureMinMax(int16_t x, int16_t y, int16_t z) {
    if (x < min_x_) min_x_ = x;
    if (x > max_x_) max_x_ = x;
    if (y < min_y_) min_y_ = y;
    if (y > max_y_) max_y_ = y;
    if (z < min_z_) min_z_ = z;
    if (z > max_z_) max_z_ = z;
    if (samples_ < 65535) samples_++;
  }

  static float wrap(float a) { float x = fmodf(a, 360.0f); return x < 0.0f ? x + 360.0f : x; }

  void fail() { if (++errors_ > 10) { ok_ = false; } }

  bool writeReg(uint8_t reg, uint8_t value) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
  }

  bool readReg(uint8_t reg, uint8_t &out) {
    wire_->beginTransmission(addr_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) return false;
    if (wire_->requestFrom((int)addr_, 1) != 1) return false;
    out = wire_->read();
    return true;
  }

  bool readRaw(int16_t &x, int16_t &y, int16_t &z) {
    wire_->beginTransmission(addr_);
    wire_->write(REG_DATA);
    if (wire_->endTransmission(false) != 0) return false;
    if (wire_->requestFrom((int)addr_, 6) != 6) return false;
    uint8_t b[6];
    for (uint8_t i = 0; i < 6; ++i) b[i] = wire_->read();
    x = (int16_t)((uint16_t)b[1] << 8 | b[0]);        // little endian
    y = (int16_t)((uint16_t)b[3] << 8 | b[2]);
    z = (int16_t)((uint16_t)b[5] << 8 | b[4]);
    return !(x == 0 && y == 0 && z == 0);
  }

  TwoWire *wire_ = nullptr;
  uint8_t  addr_ = 0x0D;
  bool     present_ = false;
  bool     ok_ = false;
  bool     disturbed_ = false;
  uint8_t  errors_ = 0;
  uint32_t last_ok_ms_ = 0;

  Calibration cal_;
  float declination_  = 0.0f;
  float mount_offset_ = 0.0f;
  bool  invert_ = false;
  float field_tol_ = 0.35f;

  float heading_deg_ = 0.0f;
  float field_ = 0.0f;
  int16_t raw_x_ = 0, raw_y_ = 0, raw_z_ = 0;

  bool     calibrating_ = false;
  int16_t  min_x_ = 32767, min_y_ = 32767, min_z_ = 32767;
  int16_t  max_x_ = -32768, max_y_ = -32768, max_z_ = -32768;
  uint16_t samples_ = 0;
};

#endif // COMPASS_QMC5883_H

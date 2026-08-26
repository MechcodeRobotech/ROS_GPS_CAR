/**
 * @file drive_controller.h
 * @brief Heading-hold differential drive controller (no encoders).
 *
 * AUTO mode  : the ROS side sends "go to heading H at speed V".
 *              - heading PID (IMU) produces a differential PWM term
 *              - speed PI + feed-forward produces the common PWM term
 *              - if the heading error is large the robot first turns on the
 *                spot (counter-rotating wheels), then drives straight
 * MANUAL mode: the web UI sends forward speed + yaw rate, open loop with the
 *              same feed-forward mapping. Used for jogging the robot around.
 *
 * Everything is fixed point-free float math on stack variables; the two PIDF
 * objects are the only state and they are created once.
 */
#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

#include <Arduino.h>
#include <PIDF.h>
#include <config.h>
#include <state_estimator.h>

class DriveController {
public:
  DriveController()
  : heading_(-HEADING_OUT_MAX, HEADING_OUT_MAX,
             HEADING_KP, HEADING_KI, HEADING_I_MIN, HEADING_I_MAX,
             HEADING_KD, HEADING_KF, HEADING_ERROR_TOL_DEG),
    speed_(-(float)PWM_MAX, (float)PWM_MAX,
           SPEED_KP, SPEED_KI, SPEED_I_MIN, SPEED_I_MAX,
           SPEED_KD, SPEED_KF, SPEED_ERROR_TOL_MPS)
  {}

  void begin() {
    heading_.setDFilterCutoffHz(HEADING_D_FILTER_HZ);
    speed_.setDFilterCutoffHz(SPEED_D_FILTER_HZ);
    reset();
  }

  void reset() {
    heading_.reset();
    speed_.reset();
    speed_ramp_ = 0.0f;
    spinning_   = false;
    pwm_l_ = pwm_r_ = 0;
    out_l_ = out_r_ = 0.0f;
  }

  /**
   * Motor acceleration ramp.
   * @param accel_s time to go from stopped to full PWM  (0 = instant)
   * @param decel_s time to come back down to zero
   * The ramp is applied to the PWM that actually reaches the driver, so it
   * limits current draw and gearbox shock whatever the control loop asks for.
   * An emergency stop bypasses it: main.cpp calls motor.disable() directly.
   */
  void setRamp(float accel_s, float decel_s) {
    accel_per_s_ = (accel_s > 0.01f) ? ((float)PWM_MAX / accel_s) : 0.0f;
    decel_per_s_ = (decel_s > 0.01f) ? ((float)PWM_MAX / decel_s) : 0.0f;
  }

  //---------------------------- commands ----------------------------//
  void setAuto(float speed_mps, float heading_deg) {
    target_speed_   = speed_mps;
    target_heading_ = wrap360f(heading_deg);
    manual_ = false;
    turn_in_place_ = false;
  }

  void setManual(float speed_mps, float yaw_rate_dps) {
    target_speed_    = speed_mps;
    target_yaw_rate_ = yaw_rate_dps;
    manual_ = true;
    turn_in_place_ = false;
  }

  /**
   * "Turn to 90 degrees" / "turn to north" - rotate on the spot and stop.
   * Works without GPS (indoor test mode): the heading comes from the compass
   * and the gyro, no ground speed needed.
   */
  void setTurnTo(float heading_deg) {
    target_heading_  = wrap360f(heading_deg);
    target_speed_    = 0.0f;
    target_yaw_rate_ = 0.0f;
    speed_ramp_      = 0.0f;
    manual_          = false;
    turn_in_place_   = true;
    turn_done_       = false;
    turn_ok_ms_      = 0;
    heading_.reset();
  }

  bool turningInPlace() const { return turn_in_place_; }
  bool turnDone() const { return turn_done_; }

  void stop() {
    target_speed_ = 0.0f;
    target_yaw_rate_ = 0.0f;
    speed_ramp_ = 0.0f;
    spinning_ = false;
    turn_in_place_ = false;
    heading_.reset();
    speed_.reset();
    pwm_l_ = pwm_r_ = 0;
    out_l_ = out_r_ = 0.0f;
  }

  //----------------------------- update -----------------------------//
  /** Two stages: work out what the wheels should do, then ramp the PWM there. */
  void update(float dt, const StateEstimator &est) {
    computeTargets(dt, est);
    applyRamp(dt);
  }

private:
  void computeTargets(float dt, const StateEstimator &est) {
    // speed setpoint slew, so a step command does not slam the gearbox
    float max_step = SPEED_SLEW_MPS_S * dt;
    float d = target_speed_ - speed_ramp_;
    if (d >  max_step) d =  max_step;
    if (d < -max_step) d = -max_step;
    speed_ramp_ += d;

    if (manual_) { updateManual(dt); return; }

    heading_error_ = angErrDeg(target_heading_, est.headingDeg());

    if (turn_in_place_) { updateTurnInPlace(); return; }

    // large error -> turn on the spot first
    if (!spinning_ && fabsf(heading_error_) > HEADING_SPIN_ERR_DEG && fabsf(target_speed_) > 1e-3f) {
      spinning_ = true;
      speed_.reset();
      speed_ramp_ = 0.0f;
    } else if (spinning_ && fabsf(heading_error_) < HEADING_SPIN_EXIT_DEG) {
      spinning_ = false;
      heading_.reset();
    }

    if (spinning_) {
      // headings are compass degrees: a positive error means "turn clockwise",
      // i.e. the left wheel drives forward and the right wheel backwards
      int turn = (heading_error_ > 0.0f) ? HEADING_SPIN_PWM : -HEADING_SPIN_PWM;
      tgt_l_ =  turn;
      tgt_r_ = -turn;
      return;
    }

    if (fabsf(target_speed_) < 1e-3f) { stopOutputs(); return; }

    // common term: feed-forward from the motor model + PI on the fused speed
    float ff   = SPEED_FF_GAIN * (speed_ramp_ / ROBOT_MAX_SPEED_MPS) * (float)PWM_MAX;
    float corr = speed_.compute(speed_ramp_, est.speed());
    float base = ff + corr;
    base = clampf(base, -(float)PWM_MAX, (float)PWM_MAX);

    // differential term from the heading PID (compass error, + = turn right)
    float diff = heading_.compute_with_error(heading_error_);
    if (target_speed_ < 0.0f) diff = -diff;   // steering flips when reversing

    float left  = base + diff;
    float right = base - diff;

    // keep the ratio if one side saturates, so the robot does not veer
    float peak = fmaxf(fabsf(left), fabsf(right));
    if (peak > (float)PWM_MAX) {
      float k = (float)PWM_MAX / peak;
      left  *= k;
      right *= k;
    }

    tgt_l_ = applyDeadband((int)lroundf(left));
    tgt_r_ = applyDeadband((int)lroundf(right));
  }

public:
  int   pwmLeft()  const { return pwm_l_; }
  int   pwmRight() const { return pwm_r_; }
  float headingError() const { return heading_error_; }
  float targetHeading() const { return target_heading_; }
  float targetSpeed()   const { return target_speed_; }
  float targetYawRate() const { return target_yaw_rate_; }
  float rampedSpeed()   const { return speed_ramp_; }
  bool  spinning()      const { return spinning_; }
  bool  manual()        const { return manual_; }

  /** Open-loop speed implied by the current PWM - the estimator uses it when GPS is lost. */
  float modelSpeedMps() const {
    float duty = 0.5f * ((float)pwm_l_ + (float)pwm_r_) / (float)PWM_MAX;
    return duty * ROBOT_MAX_SPEED_MPS;
  }

  PIDF &headingPid() { return heading_; }
  PIDF &speedPid()   { return speed_; }

private:
  static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  void stopOutputs() {
    tgt_l_ = tgt_r_ = 0;
    speed_.reset();
    heading_.reset();
  }

  /**
   * Slew the applied PWM towards the target.
   *  - starting from rest the output jumps to PWM_MIN_MOVE first (breakaway
   *    torque) and ramps from there, otherwise the ramp is spent below the
   *    stiction point and the wheels just buzz
   *  - a direction reversal always ramps down through zero first
   *  - stopping uses the (faster) decel rate
   */
  void applyRamp(float dt) {
    pwm_l_ = rampAxis(out_l_, (float)tgt_l_, dt);
    pwm_r_ = rampAxis(out_r_, (float)tgt_r_, dt);
  }

  int rampAxis(float &current, float target, float dt) {
    if (accel_per_s_ <= 0.0f) { current = target; return (int)lroundf(current); }

    const bool reversing = (current > 1.0f && target < -1.0f) || (current < -1.0f && target > 1.0f);
    const float aim = reversing ? 0.0f : target;
    const bool slowing = fabsf(aim) < fabsf(current);
    const float rate = (slowing && decel_per_s_ > 0.0f) ? decel_per_s_ : accel_per_s_;

    if (fabsf(current) < 1.0f && fabsf(aim) > 1.0f) {
      current = (aim > 0.0f) ? (float)PWM_MIN_MOVE : -(float)PWM_MIN_MOVE;  // breakaway
      if (fabsf(current) > fabsf(aim)) current = aim;
    }

    const float step = rate * dt;
    const float diff = aim - current;
    current += (diff > step) ? step : (diff < -step ? -step : diff);

    if (fabsf(current) < 1.0f) current = 0.0f;
    return (int)lroundf(current);
  }

  static int applyDeadband(int pwm) {
    if (pwm == 0) return 0;
    if (abs(pwm) < PWM_MIN_MOVE) return (pwm > 0) ? PWM_MIN_MOVE : -PWM_MIN_MOVE;
    if (pwm >  PWM_MAX) return  PWM_MAX;
    if (pwm < -PWM_MAX) return -PWM_MAX;
    return pwm;
  }

  /** Rotate on the spot until the heading error is inside the tolerance, then stop. */
  void updateTurnInPlace() {
    const uint32_t now = millis();
    if (fabsf(heading_error_) <= TURN_IN_PLACE_TOL_DEG) {
      if (turn_ok_ms_ == 0) turn_ok_ms_ = now;
      tgt_l_ = tgt_r_ = 0;
      if (now - turn_ok_ms_ >= TURN_IN_PLACE_SETTLE_MS) {
        turn_in_place_ = false;
        turn_done_ = true;
        heading_.reset();
      }
      return;
    }
    turn_ok_ms_ = 0;

    // proportional so it eases into the target instead of banging into it
    float mag = fabsf(heading_.compute_with_error(heading_error_));
    if (mag < (float)PWM_MIN_MOVE)   mag = (float)PWM_MIN_MOVE;
    if (mag > (float)HEADING_SPIN_PWM) mag = (float)HEADING_SPIN_PWM;

    const int turn = (heading_error_ > 0.0f) ? (int)lroundf(mag) : -(int)lroundf(mag);
    tgt_l_ =  turn;      // compass error > 0 = turn clockwise
    tgt_r_ = -turn;
  }

  void updateManual(float dt) {
    (void)dt;
    if (fabsf(speed_ramp_) < 1e-3f && fabsf(target_yaw_rate_) < 1e-3f) { stopOutputs(); return; }
    float base = (speed_ramp_ / ROBOT_MAX_SPEED_MPS) * (float)PWM_MAX;
    // yaw rate [deg/s, clockwise positive] -> wheel speed difference -> PWM
    float dv   = (target_yaw_rate_ * 0.01745329f) * WHEEL_TRACK_M * 0.5f;
    float diff = (dv / ROBOT_MAX_SPEED_MPS) * (float)PWM_MAX;

    float left  = base + diff;
    float right = base - diff;
    float peak  = fmaxf(fabsf(left), fabsf(right));
    if (peak > (float)PWM_MAX) { float k = (float)PWM_MAX / peak; left *= k; right *= k; }

    tgt_l_ = applyDeadband((int)lroundf(left));
    tgt_r_ = applyDeadband((int)lroundf(right));
  }

  PIDF  heading_;
  PIDF  speed_;
  float target_speed_    = 0.0f;
  float target_heading_  = 0.0f;
  float target_yaw_rate_ = 0.0f;
  float speed_ramp_      = 0.0f;
  float heading_error_   = 0.0f;
  bool  spinning_ = false;
  bool  manual_   = false;
  bool  turn_in_place_ = false;
  bool  turn_done_ = false;
  uint32_t turn_ok_ms_ = 0;
  int   tgt_l_ = 0, tgt_r_ = 0;       // what the controller asks for
  float out_l_ = 0.0f, out_r_ = 0.0f; // what the ramp has actually applied
  int   pwm_l_ = 0, pwm_r_ = 0;       // rounded copy of out_*, sent to the driver
  float accel_per_s_ = (float)PWM_MAX / MOTOR_ACCEL_RAMP_S;
  float decel_per_s_ = (float)PWM_MAX / MOTOR_DECEL_RAMP_S;
};

#endif // DRIVE_CONTROLLER_H

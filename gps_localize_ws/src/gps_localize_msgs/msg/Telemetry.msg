# Telemetry from the ESP32. Nominally 10 Hz (PUB_TELEMETRY_DIV).
#
# Replaces a packed Float32MultiArray. Named fields mean adding a signal is
# one line here and both sides pick it up - no index to keep in step across
# three files, which is exactly what the old array required and what QC
# check 11 existed to police.

float32 state    # RunState: 0 idle 1 auto 2 manual 3 estop 4 comm_loss
float32 heading_deg    # fused heading, compass degrees (0 = north, CW)
float32 target_heading_deg
float32 heading_error_deg
float32 speed_mps    # fused body speed
float32 target_speed_mps
float32 wheel_rpm_left    # derived from speed + yaw rate (no encoder on this robot)
float32 wheel_rpm_right
float32 wheel_mps_left
float32 wheel_mps_right
float32 yaw_rate_dps    # counter-clockwise positive
float32 pwm_left
float32 pwm_right
float32 battery_v    # pack voltage, read from the motor rail (VCC EMER).
float32 battery_a    # whole-module draw, measured on the battery side
float32 motor_a_v
float32 motor_a_a
float32 motor_b_v
float32 motor_b_a
float32 gps_fix_quality    # 0 none, 1 GPS, 2 DGPS/SBAS
float32 gps_satellites
float32 gps_hdop
float32 gps_speed_mps
float32 gps_course_deg
float32 stop_flags    # bitfield, see STOP_FLAGS
float32 free_heap_kb    # watch this on the web UI: it must stay flat
float32 loop_us    # control loop duration
float32 odom_x    # ESP dead-reckoning, ENU metres
float32 odom_y
float32 mag_heading_deg    # QMC5883L compass in the GPS module, tilt compensated
float32 heading_ref    # what the heading is locked to right now
float32 mag_status    # 0 no compass, 1 ok, 2 magnetic disturbance
float32 mag_calib_state    # 0 idle, 1 capturing, 2 saved, 3 failed (turn was too small)
float32 sensor_health    # bitfield of missing sensors, see SENSOR_HEALTH
float32 speed_source    # what the speed estimate is built from
float32 motor_rpm_left    # back-EMF wheel speed from the INA226 volts + amps
float32 motor_rpm_right
float32 motor_faults    # bitfield: stall / open circuit per motor
float32 fan_v    # fan rail INA226 (schematic draft_5)
float32 fan_a
float32 esp32_rail_v    # ESP32 +5 V rail, health of the processor buck
float32 pack_valid    # 1 = pack voltage is measurable; 0 = motor rail
float32 pos_sigma_m    # 1-sigma position uncertainty from the EKF [m].
float32 pos_anchored    # 1 = the EKF has had at least one fix; 0 = the

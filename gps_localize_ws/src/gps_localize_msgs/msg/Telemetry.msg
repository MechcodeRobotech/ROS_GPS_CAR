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

# major * 100 + minor, from firmware/config/version.h. Sent in the ordinary
# frame because it is four bytes and needs no new publisher, no string handling
# and no extra type support in the firmware image. 0 from a board too old to
# send it - which is itself the answer to "how old is this board".
float32 firmware_version

# Why the board last restarted, from esp_reset_reason(). This is how a brownout
# is told apart from a crash and from someone pressing reset - they look
# identical in the logs otherwise, and a 3.3 V rail that sags under Wi-Fi bursts
# shows up here as ESP_RST_BROWNOUT and nowhere else.
#   1 POWERON  3 SW  4 PANIC  5 INT_WDT  6 TASK_WDT  7 WDT  8 DEEPSLEEP
#   9 BROWNOUT  10 SDIO
float32 reset_reason

# Seconds since boot. A board that is quietly resetting has a short uptime that
# keeps starting over, which is invisible if you only ever look at live values.
float32 uptime_s

# ESP32 die temperature, degrees C, or NaN when the chip has no usable sensor.
#
# The original ESP32 (WROOM-32) carries a temperature sensor that Espressif
# disabled after early silicon: temperatureRead() still answers, but on most
# boards it returns a fixed value near 53 C no matter how hot the chip is. A
# constant is worse than nothing, because it looks like a reading - so the
# firmware checks whether it MOVES before believing it, and reports NaN when it
# does not.
#
# Note what this is not: the die is not the regulator. The AMS1117 dropping
# 5 V to 3.3 V is the part that actually runs hot, and nothing on the board
# measures it.
float32 esp32_temp_c

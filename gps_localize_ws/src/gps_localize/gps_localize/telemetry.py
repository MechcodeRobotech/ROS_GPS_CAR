# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Field names and derived values for /gps_localize/telemetry.

The robot publishes gps_localize_msgs/Telemetry - named fields, no
indices. FIELDS is kept as the canonical name list, and the .msg file is
checked against it by QC so the two cannot drift.

One packed array instead of a dozen small topics: it keeps the micro-ROS
memory pool small on the ESP32 and gives the web UI a single consistent
snapshot per tick. The firmware fills it in src/main.cpp::publishTelemetry -
KEEP BOTH SIDES IN SYNC.
"""


# index -> key used everywhere on the ROS/web side
FIELDS = (
    'state',              # 0  RunState: 0 idle 1 auto 2 manual 3 estop 4 comm_loss
    'heading_deg',        # 1  fused heading, compass degrees (0 = north, CW)
    'target_heading_deg',  # 2
    'heading_error_deg',  # 3
    'speed_mps',          # 4  fused body speed
    'target_speed_mps',   # 5
    'wheel_rpm_left',     # 6  derived from speed + yaw rate (no encoder on this robot)
    'wheel_rpm_right',    # 7
    'wheel_mps_left',     # 8
    'wheel_mps_right',    # 9
    'roll_rate_dps',
    'pitch_rate_dps',
    'yaw_rate_dps',       # 10 counter-clockwise positive
    'imu_roll_deg',       # 11
    'imu_pitch_deg',      # 12
    'imu_yaw_deg',        # 13
    'pwm_left',           # 14
    'pwm_right',          # 15
    'battery_v',          # 16 pack voltage, read from the motor rail (VCC EMER).
                          #    NOT the processor sensor: that one measures the
                          #    ESP32 +5 V rail. See firmware/config/power.h.
    'battery_a',          # 14 whole-module draw, measured on the battery side
    'motor_a_v',          # 15
    'motor_a_a',          # 16
    'motor_b_v',          # 17
    'motor_b_a',          # 18
    'gps_fix_quality',    # 19 0 none, 1 GPS, 2 DGPS/SBAS
    'gps_satellites',     # 20
    'gps_hdop',           # 21
    'gps_speed_mps',      # 22
    'gps_course_deg',     # 23
    'stop_flags',         # 24 bitfield, see STOP_FLAGS
    'free_heap_kb',       # 25 watch this on the web UI: it must stay flat
    'loop_us',            # 26 control loop duration
    'odom_x',             # 27 ESP dead-reckoning, ENU metres
    'odom_y',             # 28
    'mag_heading_deg',    # 29 QMC5883L compass in the GPS module, tilt compensated
    'heading_ref',        # 30 what the heading is locked to right now
    'mag_status',         # 31 0 no compass, 1 ok, 2 magnetic disturbance
    'mag_calib_state',    # 32 0 idle, 1 capturing, 2 saved, 3 failed (turn was too small)
    'sensor_health',      # 33 bitfield of missing sensors, see SENSOR_HEALTH
    'speed_source',       # 34 what the speed estimate is built from
    'motor_rpm_left',     # 35 back-EMF wheel speed from the INA226 volts + amps
    'motor_rpm_right',    # 36
    'motor_faults',       # 37 bitfield: stall / open circuit per motor
    'fan_v',              # 38 fan rail INA226 (schematic draft_5)
    'fan_a',              # 39
    'esp32_rail_v',       # 40 ESP32 +5 V rail, health of the processor buck
    'pack_valid',         # 41 1 = pack voltage is measurable; 0 = motor rail
                          #    unpowered (mushroom switch or relay open), so
                          #    the pack voltage is unknown, not low
    'pos_sigma_m',        # 42 1-sigma position uncertainty from the EKF [m].
                          #    Grows while coasting without GPS, shrinks on a
                          #    fix - show it, so the map dot is honest.
    'pos_anchored',       # 43 1 = the EKF has had at least one fix; 0 = the
                          #    position is still open-loop dead reckoning
    'firmware_version',   # 44 major*100+minor, 0 from a board too old to send it
    'reset_reason',       # 45 esp_reset_reason(): 9 = brownout, 4 = panic
    'uptime_s',           # 46 seconds since boot
    'esp32_temp_c',       # 47 die temperature, NaN if the chip has no usable sensor
    'mag_calib_progress',  # 48 how far round the circle the compass sweep got,
                           #    0..1, as the fraction of 16 sectors visited.
                           #    Sample count is not progress: standing still
                           #    piles up samples and calibrates nothing.
    'imu_accuracy',        # 49 BNO085's own confidence in its magnetometer,
                           #    0 unreliable .. 3 high. The heading chain will
                           #    not use the sensor below 2.
    'imu_calib_state',     # 50 0 idle, 1 learning, 2 saved, 3 refused
    'imu_calib_result',    # 51 what the sensor said: 0 SH2_OK, otherwise a
                           #    negative sh2_err.h code (-100 = IMU never began)
    'mag_turned_deg',      # 52 degrees actually turned during the compass sweep,
                           #    from the IMU's fused heading. Signed, so rocking
                           #    cancels. The independent check on a measurement
                           #    that would otherwise judge itself.
    'imu_acc_x',           # 53 linear acceleration, m/s^2, GRAVITY REMOVED - a
    'imu_acc_y',           # 54 robot standing still reads ~0 on all three, not
    'imu_acc_z',           # 55 9.81 on one. This is what the speed estimator eats.
    'imu_resets',          # 56 times the BNO085 has reset itself since boot.
                           #    Each one is a gap in the stream, and a gap over
                           #    IMU_STALE_MS reads as the IMU going missing.
    'imu_hz_rv',           # 57 delivered rate of each BNO085 report, Hz. Asked
    'imu_hz_game',         # 58 for: 10, 100, 100, 50. Four reports share one
    'imu_hz_gyro',         # 59 I2C bus and the slowest starves first, so a low
    'imu_hz_accel',        # 60 rv beside healthy fast rates is a full bus, not
                           #    a broken magnetometer. They looked identical on
                           #    the dashboard, and one was mistaken for the other.
    'heading_innov_deg',   # 61 how far the absolute heading reference sits from
                           #    the IMU's own fused yaw, signed degrees. Near
                           #    zero = two independent sources agree, which is
                           #    worth more than either alone. Large = one of
                           #    them is disturbed.
    'heading_gated',       # 62 1 while a reference is being refused for
                           #    disagreeing. Not a fault - it is the cross-check
                           #    working, and it clears by itself.
    'wifi_rssi_dbm',       # 63 signal strength at the ROBOT, dBm, negative. 0 =
                           #    not associated. The number that separates "the
                           #    firmware is stuck" from "it drove out of range".
)

# Derived, never hand-written. This was a literal 44 while FIELDS had grown to
# 47, and only the port gate noticed - a constant maintained by hand beside
# the list it describes will eventually disagree with it.
TELEMETRY_LEN = len(FIELDS)

# must match publishTelemetry() in firmware/src/main.cpp
MOTOR_FAULTS = (
    (1 << 0, 'left_stalled'),
    (1 << 1, 'right_stalled'),
    (1 << 2, 'left_open_circuit'),
    (1 << 3, 'right_open_circuit'),
)

# What to actually DO about each one. "open circuit" reads like a broken wire,
# and on this robot it was not: the right channel drew 0.001 A against the
# left's 0.6 A for 24 consecutive moves, then worked perfectly after the robot
# was fully unpowered for ten seconds. A wire does not repair itself. A motor
# driver latching into thermal or overcurrent protection behaves exactly like
# that - it stays off until VM is removed, whatever the logic inputs say.
MOTOR_FAULT_ADVICE = {
    'left_open_circuit':
        'no current at all on the left rail while it is being driven. If a full '
        'power-down (10 s, battery AND USB) brings it back, the driver latched '
        'into protection rather than a wire breaking',
    'right_open_circuit':
        'no current at all on the right rail while it is being driven. If a full '
        'power-down (10 s, battery AND USB) brings it back, the driver latched '
        'into protection rather than a wire breaking',
    'left_stalled':
        'current above motor.stall_a with the wheel not turning. Often the '
        'threshold rather than the motor - check it on the Calibration tab',
    'right_stalled':
        'current above motor.stall_a with the wheel not turning. Often the '
        'threshold rather than the motor - check it on the Calibration tab',
}

STATE_NAMES = {0: 'IDLE', 1: 'AUTO', 2: 'MANUAL', 3: 'ESTOP', 4: 'COMM_LOSS'}
HEADING_REF_NAMES = {0: 'relative (no absolute reference)', 1: 'compass (QMC5883L)',
                     2: 'GPS course', 3: 'IMU magnetometer'}
MAG_STATUS_NAMES = {0: 'none', 1: 'ok', 2: 'disturbed'}
MAG_CALIB_NAMES = {0: 'idle', 1: 'turning', 2: 'saved', 3: 'failed'}
IMU_CALIB_NAMES = {0: 'idle', 1: 'learning', 2: 'saved', 3: 'refused'}
# What the BNO085's accuracy number means to a person deciding whether to keep
# waving the robot about. 2 is the line the heading chain draws.
IMU_ACCURACY_NAMES = {0: 'unreliable', 1: 'low', 2: 'medium (usable)',
                      3: 'high'}

# sh2_err.h, so a refusal can say what actually happened rather than only that
# it happened. Only the codes these three commands can produce are named.
IMU_CALIB_RESULTS = {
    0: 'ok',
    -1: 'general sensor error',
    -2: 'bad parameter',
    -3: 'another command is still running',
    -4: 'I2C error talking to the sensor',
    -5: 'the sensor rejected the command',
    -6: 'timed out waiting for the sensor',
    # Our own codes from imu_bno085.h, for the failures sh2 never sees. Three
    # different actions, which is why they are three different codes.
    # Proved on hardware: this state survives every software reset - a USB
    # flash, an over-the-air update, ESP.restart() - and clears the moment the
    # board is fully unpowered for ten seconds. Retrying, an SHTP soft reset, a
    # longer I2C timeout, draining the backlog and a burst of handshakes were
    # all tried and none of them help.
    -100: 'the IMU never started - it clears only on a FULL power-down: '
          'unplug BOTH the USB cable and the battery for 10 seconds',
    -101: 'nothing answers at the IMU address - check the I2C wiring',
    -102: 'the IMU answers but is not running - power-cycle the robot '
          '(its reset line is not wired, so a reboot does not reset it)',
    -103: 'not sent: the robot is being driven. Stop it and try again',
    # The handshake worked and then every report was refused. Measured on the
    # robot at firmware 1.39: rv, game, gyro and accel all at 0.0 Hz against 10,
    # 100, 100 and 50 asked for, with a clean open and zero resets. Nothing used
    # to look at what enableReport() returned, so this failure reported 'ok' for
    # its last operation and 'missing' for its data - two true statements that
    # together named nothing at all.
    -104: 'the IMU opened but refused to turn any report on - a FULL power-down '
          'clears it: unplug BOTH the USB cable and the battery for 10 seconds',
}
SPEED_SOURCE_NAMES = {0: 'none', 1: 'IMU + GPS', 2: 'GPS only',
                      3: 'IMU + motor model', 4: 'motor model only'}

# must match StateEstimator::HEALTH_* in firmware/lib/estimator/state_estimator.h
SENSOR_HEALTH = (
    (1 << 0, 'imu_missing'),
    (1 << 1, 'gps_missing'),
    (1 << 2, 'compass_missing'),
    (1 << 3, 'imu_magnetometer_missing'),
    (1 << 4, 'heading_relative_only'),
)

# must match firmware/config/safety_config.h
STOP_FLAGS = (
    (1 << 0, 'hardware_emergency'),
    (1 << 1, 'esp_button'),
    (1 << 2, 'software_estop'),
    (1 << 3, 'heartbeat_lost'),
    (1 << 4, 'command_timeout'),
    (1 << 5, 'agent_lost'),
    (1 << 6, 'battery_low'),
    (1 << 7, 'startup_lockout'),
    # Bit 8 existed in the firmware and was missing here, so a soft battery stop
    # arrived as an unnamed bit and showed as no reason at all.
    (1 << 8, 'battery_soft'),
    # We opened the contactor and the motor rails are still live. The one fault
    # where pressing emergency does NOT remove power, and only the INAs can see
    # it - every other check would report a safely stopped robot.
    (1 << 9, 'contactor_stuck'),
)


def unpack(data):
    """Telemetry message -> dict, with the derived names the UI uses.

    Accepts either a gps_localize_msgs/Telemetry (named fields, what the robot
    publishes now) or the old Float32MultiArray data list. The list form is
    kept so a recorded bag from before the change still replays - it costs four
    lines and makes old data readable rather than dead.
    """
    out = {}
    if hasattr(data, 'state'):                      # named message
        for key in FIELDS:
            out[key] = float(getattr(data, key, 0.0))
    else:                                           # legacy packed array
        for index, key in enumerate(FIELDS):
            out[key] = float(data[index]) if index < len(data) else 0.0
    out['state_name'] = STATE_NAMES.get(int(out['state']), 'UNKNOWN')
    out['stop_reasons'] = decode_flags(int(out['stop_flags']))
    out['heading_ref_name'] = HEADING_REF_NAMES.get(int(out['heading_ref']), 'unknown')
    out['mag_status_name'] = MAG_STATUS_NAMES.get(int(out['mag_status']), 'unknown')
    out['mag_calib_name'] = MAG_CALIB_NAMES.get(int(out['mag_calib_state']), 'unknown')
    out['imu_calib_name'] = IMU_CALIB_NAMES.get(int(out['imu_calib_state']), 'unknown')
    out['imu_accuracy_name'] = IMU_ACCURACY_NAMES.get(int(out['imu_accuracy']), 'unknown')
    out['imu_calib_result_name'] = IMU_CALIB_RESULTS.get(
        int(out['imu_calib_result']), 'sh2 error %d' % int(out['imu_calib_result']))
    out['speed_source_name'] = SPEED_SOURCE_NAMES.get(int(out['speed_source']), 'unknown')
    out['missing_sensors'] = [name for bit, name in SENSOR_HEALTH
                              if int(out['sensor_health']) & bit]
    out['motor_fault_names'] = [name for bit, name in MOTOR_FAULTS
                                if int(out['motor_faults']) & bit]
    out['motor_fault_advice'] = [MOTOR_FAULT_ADVICE[n]
                                 for n in out['motor_fault_names']
                                 if n in MOTOR_FAULT_ADVICE]
    return out


def decode_flags(flags):
    return [name for bit, name in STOP_FLAGS if flags & bit]


# ---------------------------------------------------------------- config topic
# /gps_localize/config/pid selectors, see firmware config_cb()
CONFIG_HEADING_PID = 0.0
CONFIG_SPEED_PID = 1.0
CONFIG_ESTIMATOR = 2.0
CONFIG_RESET_ODOM = 3.0
CONFIG_MAG_CALIB = 4.0      # [4, 1] start turning, [4, 0] finish and save to
                            # flash, [4, 2] clear - forget it and go back to raw.
                            # Clear is reachable from the web, not only over USB:
                            # a bad calibration kills the compass outright, and
                            # needing a cable to undo it strands the robot.
CONFIG_MOTOR_RAMP = 5.0     # [5, accel_ramp_s, decel_ramp_s]
CONFIG_MOTOR_MODEL = 6.0    # [6, ke, resistance, stall_a, dA_vf, dA_r, dB_vf, dB_r]
CONFIG_SET_HEADING = 7.0    # [7, heading_deg] - "set start pose" from the map
# [10] - tell the board to FETCH new firmware from this host over HTTP.
#
# The opposite direction from ArduinoOTA, and deliberately so: a push needs the
# board to be directly addressable, which it is not when it sits behind a
# Windows hotspot's NAT. An outbound fetch traverses that happily - it is the
# same path the micro-ROS link already uses.
CONFIG_PULL_UPDATE = 10.0
CONFIG_PARAM_SET = 8.0      # [8, index, value] - one parameter, addressed by
                            # index because this topic is a float array and
                            # cannot carry a name. The index comes from
                            # config/params.json, generated from the firmware
                            # table and checked by qc.py.
CONFIG_PARAM_RESET = 9.0    # [9] every parameter back to its default
CONFIG_IMU_CALIB = 11.0     # [11, 1] start learning, [11, 0] save into the
                            # BNO085's own flash, [11, 2] clear and restart it

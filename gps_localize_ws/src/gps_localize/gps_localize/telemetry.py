"""Field names and derived values for /gps_localize/telemetry.

The robot publishes gps_localize_msgs/Telemetry - named fields, no
indices. FIELDS is kept as the canonical name list, and the .msg file is
checked against it by QC so the two cannot drift.

One packed array instead of a dozen small topics: it keeps the micro-ROS
memory pool small on the ESP32 and gives the web UI a single consistent
snapshot per tick. The firmware fills it in src/main.cpp::publishTelemetry -
KEEP BOTH SIDES IN SYNC.
"""

TELEMETRY_LEN = 44

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
    'yaw_rate_dps',       # 10 counter-clockwise positive
    'pwm_left',           # 11
    'pwm_right',          # 12
    'battery_v',          # 13 pack voltage, read from the motor rail (VCC EMER).
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
)

# must match publishTelemetry() in firmware/src/main.cpp
MOTOR_FAULTS = (
    (1 << 0, 'left_stalled'),
    (1 << 1, 'right_stalled'),
    (1 << 2, 'left_open_circuit'),
    (1 << 3, 'right_open_circuit'),
)

STATE_NAMES = {0: 'IDLE', 1: 'AUTO', 2: 'MANUAL', 3: 'ESTOP', 4: 'COMM_LOSS'}
HEADING_REF_NAMES = {0: 'relative (no absolute reference)', 1: 'compass (QMC5883L)',
                     2: 'GPS course', 3: 'IMU magnetometer'}
MAG_STATUS_NAMES = {0: 'none', 1: 'ok', 2: 'disturbed'}
MAG_CALIB_NAMES = {0: 'idle', 1: 'turning', 2: 'saved', 3: 'failed'}
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
    out['speed_source_name'] = SPEED_SOURCE_NAMES.get(int(out['speed_source']), 'unknown')
    out['missing_sensors'] = [name for bit, name in SENSOR_HEALTH
                              if int(out['sensor_health']) & bit]
    out['motor_fault_names'] = [name for bit, name in MOTOR_FAULTS
                                if int(out['motor_faults']) & bit]
    return out


def decode_flags(flags):
    return [name for bit, name in STOP_FLAGS if flags & bit]


# ---------------------------------------------------------------- config topic
# /gps_localize/config/pid selectors, see firmware config_cb()
CONFIG_HEADING_PID = 0.0
CONFIG_SPEED_PID = 1.0
CONFIG_ESTIMATOR = 2.0
CONFIG_RESET_ODOM = 3.0
CONFIG_MAG_CALIB = 4.0      # [4, 1] start turning, [4, 0] finish and save to flash
CONFIG_MOTOR_RAMP = 5.0     # [5, accel_ramp_s, decel_ramp_s]
CONFIG_MOTOR_MODEL = 6.0    # [6, ke, resistance, stall_a, dA_vf, dA_r, dB_vf, dB_r]
CONFIG_SET_HEADING = 7.0    # [7, heading_deg] - "set start pose" from the map

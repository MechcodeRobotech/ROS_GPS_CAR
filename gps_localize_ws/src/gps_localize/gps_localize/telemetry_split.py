#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Fan the board's single telemetry frame out into six typed topics.

WHY THE SPLIT HAPPENS HERE AND NOT ON THE ESP32

The board could publish six topics itself - micro-ROS has the slots once the
compile-time publisher limit is raised. It would be worse. Every message costs
its own XRCE framing and UDP header, so six small publishes at 8 Hz put roughly
six times the packet overhead on a Wi-Fi link that already drops frames when the
robot moves behind something. One compact frame out, exploded on a machine with
no such constraints, keeps the air link cheap and the ROS graph idiomatic.

It also buys three things the firmware cannot give:

  real timestamps    the ESP32 has no synchronised clock, so anything it stamped
                     would be milliseconds since boot pretending to be a time
  real types         uint8, uint32 and bool instead of 44 float32s, with named
                     constants, so `fix_quality == GpsStatus.FIX_RTK_FIXED`
                     replaces remembering that 4 means RTK
  no firmware cost   gps_localize_msgs stays dependency-free for the board,
                     which is what lets micro-ROS compile its type support at all

WHAT COMES OUT

    /gps_localize/power/battery    sensor_msgs/BatteryState
    /gps_localize/power/rails      gps_localize_msgs/RailStatus
    /gps_localize/motors           gps_localize_msgs/MotorStatus
    /gps_localize/estimator        gps_localize_msgs/EstimatorStatus
    /gps_localize/safety/status    gps_localize_msgs/SafetyStatus
    /gps_localize/gps/status       gps_localize_msgs/GpsStatus

Subscribe to the one you care about. Nothing else has to know the layout of a
44-field frame to read a battery voltage.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy,
                       QoSProfile, QoSReliabilityPolicy)
from gps_localize.qos import stream_qos
from sensor_msgs.msg import BatteryState

from gps_localize import telemetry as tel
from gps_localize_msgs.msg import (EstimatorStatus, FirmwareInfo, GpsStatus,
                                   MotorStatus, RailStatus, SafetyStatus,
                                   Telemetry)

NS = '/gps_localize'

# A 3S LiPo, which is what the charge estimate below is scaled against.
CELLS = 3

# The firmware version this host was written alongside. Bump it in step with
# firmware/config/version.h when the contract between them changes - a new
# telemetry field, a changed unit. It is only ever used to say "the board is
# behind"; nothing is refused on the strength of it, because an older board
# still connects, still drives and still reports.
HOST_EXPECTS_FIRMWARE = 162
FULL_V = 12.6
EMPTY_V = 10.7          # BATT_CUTOFF_V: the robot stops here, so call it empty


class TelemetrySplit(Node):

    def __init__(self):
        super().__init__('telemetry_split')

        self.pub_battery = self.create_publisher(BatteryState, NS + '/power/battery', 5)
        self.pub_rails = self.create_publisher(RailStatus, NS + '/power/rails', 5)
        self.pub_motors = self.create_publisher(MotorStatus, NS + '/motors', 5)
        self.pub_estimator = self.create_publisher(EstimatorStatus, NS + '/estimator', 5)
        self.pub_safety = self.create_publisher(SafetyStatus, NS + '/safety/status', 5)
        self.pub_gps = self.create_publisher(GpsStatus, NS + '/gps/status', 5)

        # Latched: someone asking what firmware is on the board wants the answer
        # now, not on the next frame. Only republished when it changes, so a
        # board that is up to date costs nothing after the first message.
        self.pub_firmware = self.create_publisher(
            FirmwareInfo, NS + '/firmware/info',
            QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self._last_version = None

        self.create_subscription(Telemetry, NS + '/telemetry', self.on_telemetry, stream_qos())

        self.frames = 0
        self.get_logger().info(
            'telemetry_split ready: one frame in, six typed topics out')

    # ------------------------------------------------------------------ util
    def _stamp(self, msg):
        """Host clock, because the board has none worth trusting."""
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        return msg

    # ------------------------------------------------------------- callback
    def on_telemetry(self, msg):
        # unpack() understands both the named message and the old packed array,
        # so a bag recorded before the change still replays through this node.
        t = tel.unpack(msg)
        self.frames += 1

        self.pub_battery.publish(self._battery(t))
        self.pub_rails.publish(self._rails(t))
        self.pub_motors.publish(self._motors(t))
        self.pub_estimator.publish(self._estimator(t))
        self.pub_safety.publish(self._safety(t))
        self.pub_gps.publish(self._gps(t))
        self._maybe_publish_firmware(t)

    def _maybe_publish_firmware(self, t):
        """Say what firmware is on the board, and whether this host is ahead.

        Only on change. A board that is up to date would otherwise republish an
        identical message ten times a second for no reader's benefit.
        """
        raw = int(t.get('firmware_version', 0) or 0)
        if raw == self._last_version:
            return
        self._last_version = raw

        m = FirmwareInfo()
        m.header.stamp = self.get_clock().now().to_msg()
        m.version = raw
        m.version_major = raw // 100
        m.version_minor = raw % 100
        m.host_expects = HOST_EXPECTS_FIRMWARE
        m.board_is_older = 0 < raw < HOST_EXPECTS_FIRMWARE

        if raw == 0:
            # Zero is not "unknown" - it is the answer. A board that does not
            # send a version predates the field, which makes it older than any
            # version that does.
            m.board_is_older = True
            m.summary = ('the board does not report a version, so it predates '
                         'firmware %d.%d - reflash to get the newer telemetry'
                         % (HOST_EXPECTS_FIRMWARE // 100, HOST_EXPECTS_FIRMWARE % 100))
        elif m.board_is_older:
            m.summary = ('board firmware %d.%d, this host expects %d.%d - it still '
                         'works, but newer fields will read as zero'
                         % (m.version_major, m.version_minor,
                            HOST_EXPECTS_FIRMWARE // 100, HOST_EXPECTS_FIRMWARE % 100))
        elif raw > HOST_EXPECTS_FIRMWARE:
            m.summary = ('board firmware %d.%d is NEWER than this host expects '
                         '(%d.%d) - update the host with git pull'
                         % (m.version_major, m.version_minor,
                            HOST_EXPECTS_FIRMWARE // 100, HOST_EXPECTS_FIRMWARE % 100))
        else:
            m.summary = 'board firmware %d.%d, up to date' % (m.version_major, m.version_minor)

        self.get_logger().info(m.summary)
        self.pub_firmware.publish(m)

    # -------------------------------------------------------------- builders
    def _battery(self, t):
        m = BatteryState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        pack_valid = bool(t.get('pack_valid', 0.0))
        volts = float(t.get('battery_v', 0.0))

        # NaN is what BatteryState means by "not measured", and it is the honest
        # answer while the contactor is open: the pack is behind the relay, so
        # there is nothing to read. Reporting 0 V would look like a flat battery.
        m.voltage = volts if pack_valid else float('nan')
        m.current = float(t.get('battery_a', 0.0)) if pack_valid else float('nan')
        m.percentage = self._charge(volts) if pack_valid else float('nan')

        m.temperature = float('nan')
        m.charge = float('nan')
        m.capacity = float('nan')
        m.design_capacity = float('nan')
        m.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        m.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if pack_valid
            else BatteryState.POWER_SUPPLY_STATUS_UNKNOWN)
        m.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_GOOD if pack_valid
            else BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN)
        m.present = pack_valid
        m.location = 'motor rail (behind the contactor)'
        return m

    @staticmethod
    def _charge(volts):
        """Rough state of charge, 0.0 to 1.0, against the tiers in power.h.

        Linear between the cutoff and a full pack. A LiPo discharge curve is not
        linear, so this is a fuel gauge rather than a measurement - which is why
        nothing in the safety path uses it. The cutoff itself works on volts.
        """
        if not math.isfinite(volts) or volts <= 0.0:
            return float('nan')
        return max(0.0, min(1.0, (volts - EMPTY_V) / (FULL_V - EMPTY_V)))

    def _rails(self, t):
        m = self._stamp(RailStatus())
        m.motor_a_volts = float(t.get('motor_a_v', 0.0))
        m.motor_a_amps = float(t.get('motor_a_a', 0.0))
        m.motor_b_volts = float(t.get('motor_b_v', 0.0))
        m.motor_b_amps = float(t.get('motor_b_a', 0.0))
        m.fan_volts = float(t.get('fan_v', 0.0))
        m.fan_amps = float(t.get('fan_a', 0.0))
        m.esp32_rail_volts = float(t.get('esp32_rail_v', 0.0))
        m.module_amps = float(t.get('battery_a', 0.0))
        m.pack_valid = bool(t.get('pack_valid', 0.0))
        return m

    def _motors(self, t):
        m = self._stamp(MotorStatus())
        m.pwm_left = float(t.get('pwm_left', 0.0))
        m.pwm_right = float(t.get('pwm_right', 0.0))
        m.wheel_rpm_left = float(t.get('wheel_rpm_left', 0.0))
        m.wheel_rpm_right = float(t.get('wheel_rpm_right', 0.0))
        m.wheel_mps_left = float(t.get('wheel_mps_left', 0.0))
        m.wheel_mps_right = float(t.get('wheel_mps_right', 0.0))
        m.back_emf_rpm_left = float(t.get('motor_rpm_left', 0.0))
        m.back_emf_rpm_right = float(t.get('motor_rpm_right', 0.0))
        m.fault_flags = _u16(t.get('motor_faults', 0.0))
        return m

    def _estimator(self, t):
        m = self._stamp(EstimatorStatus())
        m.heading_deg = float(t.get('heading_deg', 0.0))
        m.target_heading_deg = float(t.get('target_heading_deg', 0.0))
        m.heading_error_deg = float(t.get('heading_error_deg', 0.0))
        m.speed_mps = float(t.get('speed_mps', 0.0))
        m.target_speed_mps = float(t.get('target_speed_mps', 0.0))
        m.yaw_rate_dps = float(t.get('yaw_rate_dps', 0.0))
        m.x = float(t.get('odom_x', 0.0))
        m.y = float(t.get('odom_y', 0.0))
        m.position_sigma_m = float(t.get('pos_sigma_m', 0.0))
        m.position_anchored = bool(t.get('pos_anchored', 0.0))
        m.heading_source = _u8(t.get('heading_ref', 0.0))
        m.speed_source = _u8(t.get('speed_source', 0.0))
        m.sensor_health = _u8(t.get('sensor_health', 0.0))
        m.mag_heading_deg = float(t.get('mag_heading_deg', 0.0))
        m.mag_status = _u8(t.get('mag_status', 0.0))
        m.mag_calib_state = _u8(t.get('mag_calib_state', 0.0))
        return m

    def _safety(self, t):
        m = self._stamp(SafetyStatus())
        m.run_state = _u8(t.get('state', 0.0))
        m.stop_flags = _u32(t.get('stop_flags', 0.0))
        # The contactor is closed exactly when the pack can be measured: the pack
        # sits behind it. Derived rather than sent, because the firmware already
        # tells us the one implies the other.
        m.relay_closed = bool(t.get('pack_valid', 0.0))
        m.free_heap_bytes = _u32(float(t.get('free_heap_kb', 0.0)) * 1024.0)
        m.loop_us = _u32(t.get('loop_us', 0.0))
        m.reset_reason = _u8(t.get('reset_reason', 0.0))
        m.uptime_s = float(t.get('uptime_s', 0.0))
        return m

    def _gps(self, t):
        m = self._stamp(GpsStatus())
        m.fix_quality = _u8(t.get('gps_fix_quality', 0.0))
        m.satellites = _u8(t.get('gps_satellites', 0.0))
        m.hdop = float(t.get('gps_hdop', 0.0))
        m.ground_speed_mps = float(t.get('gps_speed_mps', 0.0))
        m.course_deg = float(t.get('gps_course_deg', 0.0))
        return m


# Everything arrives as float32, and a value that is NaN, negative or larger
# than the field can hold would raise inside the publisher and kill the node on
# the first bad frame. Clamping keeps a wild reading visible as a wrong number
# rather than as a stack trace - the node stopping is worse than the value being
# out of range, because a stopped node takes every other topic with it.
def _clamp(value, hi):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0
    if not math.isfinite(v):
        return 0
    return int(max(0, min(hi, v)))


def _u8(value):
    return _clamp(value, 0xFF)


def _u16(value):
    return _clamp(value, 0xFFFF)


def _u32(value):
    return _clamp(value, 0xFFFFFFFF)


def main(args=None):
    rclpy.init(args=args)
    node = TelemetrySplit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

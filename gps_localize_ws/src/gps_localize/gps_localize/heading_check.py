#!/usr/bin/env python3
"""Measure heading jitter while the robot stands still.

    ros2 run gps_localize heading_check --seconds 30

Prints mean, standard deviation and peak-to-peak of the fused heading, plus
which absolute reference it was locked to. The acceptance target is a standard
deviation below 2 degrees with the robot parked (docs/acceptance.md).

Circular statistics are used, so a robot pointing at 359.5 deg does not produce
a nonsense 180 deg standard deviation.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from gps_localize_msgs.msg import Telemetry

from gps_localize import telemetry as tel

NS = '/gps_localize'


class HeadingCheck(Node):

    def __init__(self, seconds, limit_deg):
        super().__init__('heading_check')
        self.seconds = seconds
        self.limit_deg = limit_deg
        self.samples = []
        self.refs = {}
        self.speeds = []
        self.start = self.now()
        self.create_subscription(Telemetry, NS + '/telemetry', self.on_telemetry, 20)
        self.create_timer(0.5, self.tick)
        self.get_logger().info('collecting for %.0f s - keep the robot still' % seconds)

    def on_telemetry(self, msg):
        data = tel.unpack(msg)
        self.samples.append(data['heading_deg'])
        self.speeds.append(abs(data['speed_mps']))
        name = data.get('heading_ref_name', 'unknown')
        self.refs[name] = self.refs.get(name, 0) + 1

    def tick(self):
        elapsed = self.now() - self.start
        if elapsed < self.seconds:
            return
        self.report()
        raise SystemExit(0 if self.passed else 1)

    def report(self):
        count = len(self.samples)
        if count < 10:
            print('FAIL: only %d telemetry samples - is the robot connected?' % count)
            self.passed = False
            return

        # circular mean and standard deviation
        sin_sum = sum(math.sin(math.radians(a)) for a in self.samples)
        cos_sum = sum(math.cos(math.radians(a)) for a in self.samples)
        mean = math.degrees(math.atan2(sin_sum / count, cos_sum / count)) % 360.0
        errors = [(a - mean + 540.0) % 360.0 - 180.0 for a in self.samples]
        variance = sum(e * e for e in errors) / count
        stddev = math.sqrt(variance)
        peak = max(errors) - min(errors)
        moved = max(self.speeds) if self.speeds else 0.0
        ref = max(self.refs, key=self.refs.get) if self.refs else 'unknown'

        self.passed = stddev < self.limit_deg and 'relative' not in ref

        print('')
        print('  samples          : %d over %.0f s' % (count, self.seconds))
        print('  mean heading     : %.1f deg' % mean)
        print('  std deviation    : %.2f deg   (limit %.1f)' % (stddev, self.limit_deg))
        print('  peak to peak     : %.2f deg' % peak)
        print('  reference        : %s' % ref)
        print('  max speed seen   : %.3f m/s %s'
              % (moved, '(robot was moving - retest parked)' if moved > 0.05 else ''))
        print('')
        print('  RESULT: %s' % ('PASS' if self.passed else 'FAIL'))
        if not self.passed and 'relative' in ref:
            print('  the heading had no absolute reference: calibrate the compass'
                  ' (web UI) or drive a few metres to pick up the GPS course')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    parser = argparse.ArgumentParser(description='heading jitter check')
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--limit', type=float, default=2.0, help='allowed stddev [deg]')
    known, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = HeadingCheck(known.seconds, known.limit)
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = int(exc.code or 0)
    except KeyboardInterrupt:
        node.report()
        code = 0 if getattr(node, 'passed', False) else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

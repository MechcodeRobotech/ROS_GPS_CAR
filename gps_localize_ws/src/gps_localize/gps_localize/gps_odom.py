#!/usr/bin/env python3
"""GPS -> local ENU odometry.

The firmware already dead-reckons with the IMU, but that drifts. This node
turns every NavSatFix into a metric position relative to a datum (the first
good fix, or a datum you pin from the web UI) so the map panel, the distance
readouts and any later fusion all work in plain metres.

Frames follow REP-103: x = east, y = north, yaw counter-clockwise from east.
The heading that comes from the firmware is a compass bearing, so it is
converted here - see gps_localize/geo.py.
"""

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from gps_localize_msgs.msg import Telemetry
from std_msgs.msg import String

from gps_localize import geo
from gps_localize import telemetry as tel

NS = '/gps_localize'


def latched_qos(depth=1):
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class GpsOdom(Node):

    def __init__(self):
        super().__init__('gps_odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.datum = None            # (lat, lon)
        self.last_fix = None
        self.last_fix_time = 0.0
        self.heading_deg = 0.0       # compass, from the firmware
        self.speed_mps = 0.0

        # "set start pose" from the web map: origin the robot dead-reckons from
        # when there is no GPS at all (indoor testing, or GPS lost)
        self.manual_origin = None    # (lat, lon)
        self.manual_time = 0.0
        self.odom_xy = (0.0, 0.0)    # ENU metres from the firmware
        self.fix_timeout = 3.0

        self.pub_odom = self.create_publisher(Odometry, NS + '/odom/gps', 10)
        self.pub_datum = self.create_publisher(String, NS + '/gps/datum', latched_qos())
        self.pub_pose = self.create_publisher(String, NS + '/pose/estimate', latched_qos())

        self.create_subscription(NavSatFix, NS + '/gps/fix', self.on_fix, 10)
        self.create_subscription(Telemetry, NS + '/telemetry', self.on_telemetry, 5)
        self.create_subscription(String, NS + '/nav/cmd', self.on_cmd, 5)
        self.create_timer(0.5, self.publish_pose_estimate)

        self.get_logger().info('gps_odom waiting for the first valid fix to set the datum')

    # ---------------------------------------------------------------- inputs
    def on_telemetry(self, msg):
        # Named fields now, so the positional indexing this used to do is gone.
        # unpack() takes the message itself and still understands the old packed
        # array, which is what lets a bag recorded before the change replay.
        data = tel.unpack(msg)
        self.heading_deg = data['heading_deg']
        self.speed_mps = data['speed_mps']
        self.odom_xy = (data['odom_x'], data['odom_y'])

    def on_cmd(self, msg):
        try:
            command = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        action = command.get('action')

        if action == 'set_datum':
            lat = command.get('lat')
            lon = command.get('lon')
            if lat is None or lon is None:
                if self.last_fix is None:
                    return
                lat, lon = self.last_fix
            self.set_datum(float(lat), float(lon))

        elif action == 'set_pose':
            # operator dragged a pose arrow on the map (RViz style). From here on
            # the robot dead-reckons from that spot whenever GPS is unavailable.
            try:
                lat = float(command['lat'])
                lon = float(command['lon'])
            except (KeyError, TypeError, ValueError):
                return
            self.manual_origin = (lat, lon)
            self.manual_time = self.now()
            self.odom_xy = (0.0, 0.0)          # the firmware restarts its odometry too
            if self.datum is None:
                self.set_datum(lat, lon)
            self.get_logger().info('start pose set by hand: %.7f, %.7f heading %s'
                                   % (lat, lon, command.get('heading', '?')))
            self.publish_pose_estimate()

    def on_fix(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        if abs(msg.latitude) < 1e-7 and abs(msg.longitude) < 1e-7:
            return                      # module reported 0,0 - not a real position

        self.last_fix = (msg.latitude, msg.longitude)
        self.last_fix_time = self.now()
        if self.datum is None:
            self.set_datum(msg.latitude, msg.longitude)

        east, north = geo.enu_offset_m(msg.latitude, msg.longitude, self.datum[0], self.datum[1])

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = 0.0

        yaw = math.radians(geo.compass_to_enu_deg(self.heading_deg))
        odom.pose.pose.orientation.z = math.sin(yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(yaw * 0.5)

        var = msg.position_covariance[0] if msg.position_covariance[0] > 0.0 else 6.0
        odom.pose.covariance[0] = var
        odom.pose.covariance[7] = var
        odom.pose.covariance[35] = 0.05
        odom.twist.twist.linear.x = self.speed_mps
        self.pub_odom.publish(odom)

    # ------------------------------------------------- dead reckoned pose
    def publish_pose_estimate(self):
        """Where the robot thinks it is, and how much that can be trusted.

        With a fresh fix this is simply the GPS. Without one, and only after the
        operator has set a start pose on the map, it is that origin plus the
        firmware's IMU odometry. It is published as its own topic with an
        explicit `source` field - never as a NavSatFix, because it is not one.
        """
        now = self.now()
        fix_fresh = self.last_fix is not None and (now - self.last_fix_time) < self.fix_timeout

        if fix_fresh:
            pose = {'lat': self.last_fix[0], 'lon': self.last_fix[1],
                    'source': 'gps', 'trusted': True}
        elif self.manual_origin is not None:
            east, north = self.odom_xy
            lat, lon = geo.enu_to_latlon(east, north, self.manual_origin[0], self.manual_origin[1])
            pose = {'lat': lat, 'lon': lon, 'source': 'dead_reckoning', 'trusted': False,
                    'origin': {'lat': self.manual_origin[0], 'lon': self.manual_origin[1]},
                    'drift_m': round(math.hypot(east, north), 2),
                    'age_s': round(now - self.manual_time, 1)}
        else:
            pose = {'lat': None, 'lon': None, 'source': 'none', 'trusted': False}

        pose['heading_deg'] = round(self.heading_deg, 1)
        self.pub_pose.publish(String(data=json.dumps(pose)))

    # --------------------------------------------------------------- helpers
    def set_datum(self, lat, lon):
        self.datum = (lat, lon)
        self.pub_datum.publish(String(data=json.dumps({'lat': lat, 'lon': lon})))
        self.get_logger().info('datum set to %.7f, %.7f' % (lat, lon))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = GpsOdom()
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

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
from gps_localize.qos import stream_qos
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

        # ---------------------------------------------- fused position state
        # ENU metres from self.datum. Dead-reckoned forward from the firmware's
        # heading and speed at every tick, and pulled toward GPS whenever a fix
        # arrives. See fuse_predict() and fuse_correct() for why this exists at
        # all, and why it lives here rather than on the board.
        self.fused_en = None         # (east, north), None until the first fix
        self.fuse_time = 0.0
        self.fuse_sigma = 99.9       # metres, 1-sigma, grows while coasting
        self.fuse_rejects = 0
        self.hdop = 9.9
        self.satellites = 0
        # Tunables, all with a reason rather than a taste:
        #   uere_m        the receiver's own ranging error; HDOP multiplies it
        #   dr_drift_rate metres of uncertainty gained per metre driven
        #   reject_sigma  how far out a fix must be to be called multipath
        #   max_rejects   how long the gate may hold out before it yields
        self.uere_m = 2.0
        self.dr_drift_rate = 0.05    # per metre driven
        self.dr_drift_per_s = 0.08   # per second, whether moving or not
        self.reject_sigma = 3.0
        self.max_rejects = 10
        # The floor under the fused uncertainty, and it is the part that makes
        # this work at all.
        #
        # Without it the filter drove its own sigma down to 0.5 m after a few
        # fixes, which made the GPS weight 0.7% and turned the estimate into
        # almost pure dead reckoning. Measured: the median scatter improved from
        # 8.3 m to 5.3 m and the WORST case got worse, 20 m to 27 m, because
        # nothing was pulling the drift back.
        #
        # The mistake was treating GPS errors as independent samples that average
        # away. They are not: ionospheric delay and multipath are strongly
        # correlated minute to minute, so averaging a thousand fixes does not
        # give a millimetre - it gives the bias. The floor is that bias, and it
        # keeps GPS weighted enough to stay in charge of the long term while dead
        # reckoning handles the short.
        self.min_sigma_m = 1.5

        self.pub_odom = self.create_publisher(Odometry, NS + '/odom/gps', 10)
        self.pub_datum = self.create_publisher(String, NS + '/gps/datum', latched_qos())
        self.pub_pose = self.create_publisher(String, NS + '/pose/estimate', latched_qos())

        self.create_subscription(NavSatFix, NS + '/gps/fix', self.on_fix, stream_qos(10))
        self.create_subscription(Telemetry, NS + '/telemetry', self.on_telemetry, stream_qos())
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
        # Fix quality decides how hard a new fix is allowed to pull the fused
        # position, so it has to be current before the next correction.
        self.hdop = data.get('gps_hdop') or self.hdop
        self.satellites = int(data.get('gps_satellites') or 0)

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
        self.fuse_correct(east, north, self.gps_sigma_m(), self.last_fix_time)

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
    # ------------------------------------------------------- position fusion
    #
    # WHY THIS EXISTS. Everything downstream navigated on the RAW fix: this node
    # passed GPS straight through whenever it was fresh, waypoint_nav subscribed
    # to /gps/fix, and the map drew /gps/fix. Measured stationary on 4
    # satellites the raw fix scattered over 12 metres and jumped 4 metres
    # between consecutive samples, and outdoors on 7 satellites it still visibly
    # warps. The robot steered at every one of those jumps.
    #
    # The board does carry an EKF, but its position is expressed in ENU metres
    # from the BOARD's own origin, which is reset independently of this node's
    # datum - converting it here with this datum would be right only by luck.
    # Fusing on the host keeps one origin for the fix, the map, the distance to
    # the waypoint and the filter, which is worth more than reusing the filter
    # that already exists.
    #
    # The model is deliberately the simplest thing that matches the physics: the
    # robot knows its own speed and heading far better, moment to moment, than
    # GPS knows its position, so dead reckoning carries the estimate between
    # fixes and GPS only corrects the slow drift.

    def fuse_predict(self, now):
        """Carry the estimate forward on the robot's own speed and heading."""
        if self.fused_en is None:
            return
        dt = now - self.fuse_time
        self.fuse_time = now
        if dt <= 0.0 or dt > 2.0:        # a gap this long is not a prediction
            return
        yaw = math.radians(geo.compass_to_enu_deg(self.heading_deg))
        east, north = self.fused_en
        east += self.speed_mps * math.cos(yaw) * dt
        north += self.speed_mps * math.sin(yaw) * dt
        self.fused_en = (east, north)
        # Dead reckoning drifts, so say so: the uncertainty grows with distance
        # travelled, not with time, because a robot standing still does not
        # become less sure of where it is.
        # Uncertainty grows with distance travelled AND with time. Distance
        # alone was wrong: a robot standing still stops accumulating odometry
        # error but its heading still drifts and the world still moves around
        # it, so an estimate left alone must become less certain, or it will
        # never let a fix correct it again.
        self.fuse_sigma = min(50.0, self.fuse_sigma
                              + abs(self.speed_mps) * dt * self.dr_drift_rate
                              + dt * self.dr_drift_per_s)

    def fuse_correct(self, east, north, gps_sigma, now):
        """Pull the estimate toward a new fix, unless the fix is implausible."""
        if self.fused_en is None:
            self.fused_en = (east, north)
            self.fuse_time = now
            self.fuse_sigma = max(self.min_sigma_m, gps_sigma)
            return
        self.fuse_predict(now)
        pe, pn = self.fused_en
        innov = math.hypot(east - pe, north - pn)

        # An outlier gate, with a limit on how long it may hold out. A fix that
        # disagrees with the estimate by several sigma is usually multipath - a
        # signal bounced off a wall - and following it is what produces the
        # visible jump. But a gate that can never yield would lock the robot
        # onto a wrong position for ever if the dead reckoning were the thing
        # that had gone wrong, so a run of rejections eventually gives way.
        gate = self.reject_sigma * math.hypot(self.fuse_sigma, gps_sigma)
        if innov > gate and self.fuse_rejects < self.max_rejects:
            self.fuse_rejects += 1
            return
        self.fuse_rejects = 0

        # Weight by how much each side is trusted: a sharp fix pulls hard, a
        # poor one barely moves the estimate.
        k = (self.fuse_sigma ** 2) / (self.fuse_sigma ** 2 + gps_sigma ** 2)
        self.fused_en = (pe + k * (east - pe), pn + k * (north - pn))
        self.fuse_sigma = max(self.min_sigma_m,
                              math.sqrt(max(0.01, (1.0 - k)) * self.fuse_sigma ** 2))
        self.fuse_time = now

    def gps_sigma_m(self):
        """How far the current fix can be trusted, in metres.

        HDOP alone is a geometry factor, not an error: it has to be multiplied
        by the receiver's own ranging error to become a distance. Satellite
        count is folded in because a bare-minimum constellation is fragile in a
        way HDOP does not capture.
        """
        sigma = max(0.5, self.hdop * self.uere_m)
        if self.satellites and self.satellites < 6:
            sigma *= 2.0
        return sigma

    def publish_pose_estimate(self):
        """Where the robot thinks it is, and how much that can be trusted.

        With a fresh fix this is simply the GPS. Without one, and only after the
        operator has set a start pose on the map, it is that origin plus the
        firmware's IMU odometry. It is published as its own topic with an
        explicit `source` field - never as a NavSatFix, because it is not one.
        """
        now = self.now()
        fix_fresh = self.last_fix is not None and (now - self.last_fix_time) < self.fix_timeout

        self.fuse_predict(now)

        if fix_fresh and self.fused_en is not None and self.datum is not None:
            # The FUSED position, not the raw fix. Raw GPS is what everything
            # downstream used to steer and draw on, and it is what warps.
            east, north = self.fused_en
            lat, lon = geo.enu_to_latlon(east, north, self.datum[0], self.datum[1])
            pose = {'lat': lat, 'lon': lon, 'source': 'gps', 'trusted': True,
                    'sigma_m': round(self.fuse_sigma, 2),
                    'raw': {'lat': self.last_fix[0], 'lon': self.last_fix[1]},
                    'rejected': self.fuse_rejects}
        elif fix_fresh:
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

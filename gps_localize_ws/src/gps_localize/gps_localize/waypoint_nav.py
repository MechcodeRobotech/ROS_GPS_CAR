#!/usr/bin/env python3
"""GPS waypoint navigation.

Kept deliberately thin: this node decides *where* to go, the ESP32 decides
*how* to get there. Every cycle it publishes one command:

    /gps_localize/cmd_move   linear.x = speed [m/s], angular.z = heading [compass deg]

and the firmware holds that heading with the IMU + PID at 100 Hz. If Wi-Fi
stutters the robot keeps driving straight instead of swerving, and if the link
stays down the firmware brakes by itself.

State machine: IDLE -> RUNNING -> (PAUSED | BLOCKED) -> FINISHED
  BLOCKED  the GPS fix is missing or too poor to navigate on; it clears itself
  PAUSED   the operator (or the safety watchdog) stopped the mission
"""

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from gps_localize_msgs.msg import Telemetry
from std_msgs.msg import Bool, String

from gps_localize import geo
from gps_localize import telemetry as tel
from gps_localize.configstore import ConfigStore

NS = '/gps_localize'

IDLE = 'IDLE'
RUNNING = 'RUNNING'
PAUSED = 'PAUSED'
BLOCKED = 'BLOCKED'
FINISHED = 'FINISHED'
DISCOVER = 'DISCOVER'      # driving straight on purpose, to learn which way it points


def latched_qos(depth=1):
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class WaypointNav(Node):

    def __init__(self):
        super().__init__('waypoint_nav')
        self.declare_parameter('defaults_dir', '')
        self.declare_parameter('runtime_config_dir', '~/.gps_localize')

        defaults = self.get_parameter('defaults_dir').value or _share_config_dir()
        self.store = ConfigStore(defaults,
                                 self.get_parameter('runtime_config_dir').value,
                                 self.get_logger())
        self._load_config()
        self._load_waypoints()

        # ----------------------------------------------------------- state
        self.state = IDLE
        self.index = 0
        self.fix = None
        self.fix_time = 0.0
        self.fix_source = 'none'      # 'gps' or 'dead_reckoning'
        self.gps_fix_time = 0.0       # last REAL satellite fix, never dead reckoning
        self.fix_quality = 0
        self.satellites = 0
        self.hdop = 99.9
        self.heading_deg = 0.0
        self.speed_mps = 0.0
        self.heading_ref = 0          # 0 relative, 1 compass, 2 GPS course, 3 IMU mag
        self.estop = False
        self.block_reason = ''
        self.dead_reckon = None       # pose estimate when there is no GPS
        self.discover_start = 0.0
        self.discover_from = None
        # one-off destination picked on the map; it is NOT added to the list
        self.temp_target = None
        self.leg_start_time = 0.0
        self.leg_start_distance = 0.0
        self.last_distance = 0.0
        self.last_bearing = 0.0

        # ------------------------------------------------------- interfaces
        self.pub_cmd = self.create_publisher(Twist, NS + '/cmd_move', 5)
        self.pub_status = self.create_publisher(String, NS + '/nav/status', latched_qos())
        self.pub_log = self.create_publisher(String, NS + '/log', 20)

        self.create_subscription(NavSatFix, NS + '/gps/fix', self.on_fix, 10)
        self.create_subscription(Telemetry, NS + '/telemetry', self.on_telemetry, 5)
        self.create_subscription(Bool, NS + '/estop', self.on_estop, latched_qos())
        self.create_subscription(String, NS + '/nav/cmd', self.on_cmd, 10)
        self.create_subscription(String, NS + '/config/updated', self.on_config_updated, 5)
        self.create_subscription(String, NS + '/pose/estimate', self.on_pose_estimate,
                                 latched_qos())

        self.create_timer(1.0 / max(1.0, self.command_rate), self.tick)
        self.create_timer(0.5, self.publish_status)
        self.get_logger().info('waypoint_nav ready, %d waypoints loaded' % len(self.waypoints))

    # -------------------------------------------------------------- config
    def _load_config(self):
        cfg = self.store.load('nav').get('nav', {})
        self.cruise_speed = float(cfg.get('cruise_speed_mps', 0.25))
        self.approach_speed = float(cfg.get('approach_speed_mps', 0.12))
        self.approach_distance = float(cfg.get('approach_distance_m', 1.5))
        self.arrive_radius = float(cfg.get('arrive_radius_m', 1.0))
        self.heading_tolerance = float(cfg.get('heading_tolerance_deg', 15.0))
        self.command_rate = float(cfg.get('command_rate_hz', 5.0))
        self.loop_waypoints = bool(cfg.get('loop_waypoints', False))
        self.waypoint_timeout = float(cfg.get('waypoint_timeout_s', 180.0))
        self.require_fix = bool(cfg.get('require_fix', True))
        self.min_satellites = int(cfg.get('min_satellites', 6))
        self.max_hdop = float(cfg.get('max_hdop', 3.0))
        self.fix_timeout = float(cfg.get('fix_timeout_s', 3.0))

        # heading discovery: what to do when the robot has no absolute heading
        # (compass missing or disturbed, GPS course not yet valid)
        self.discover_enabled = bool(cfg.get('discover_heading', True))
        self.discover_speed = float(cfg.get('discover_speed_mps', 0.15))
        self.discover_max_s = float(cfg.get('discover_max_s', 12.0))

        # Backup for testing without GPS: after the operator sets a start pose on
        # the map, navigate on the dead-reckoned position instead of a fix.
        # OFF by default - dead reckoning drifts metres per minute.
        self.allow_dead_reckoning = bool(cfg.get('allow_dead_reckoning', False))
        self.discover_distance = float(cfg.get('discover_distance_m', 2.0))

    def _load_waypoints(self):
        raw = self.store.load('waypoints').get('waypoints', []) or []
        points = []
        for i, item in enumerate(raw):
            try:
                lat = float(item['lat'])
                lon = float(item['lon'])
            except (KeyError, TypeError, ValueError):
                self.get_logger().warn('waypoint %d ignored: needs numeric lat and lon' % (i + 1))
                continue
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                self.get_logger().warn('waypoint %d ignored: out of range' % (i + 1))
                continue
            points.append({'name': str(item.get('name', 'WP%d' % (i + 1))), 'lat': lat, 'lon': lon})
        self.waypoints = points

    def on_config_updated(self, msg):
        try:
            group = json.loads(msg.data).get('group')
        except (ValueError, TypeError):
            return
        if group in ('nav', 'all'):
            self._load_config()
            self.log('navigation settings reloaded')
        if group in ('waypoints', 'all'):
            was_running = self.state == RUNNING
            self._load_waypoints()
            if self.index >= len(self.waypoints):
                self.index = 0
            if was_running and not self.waypoints:
                self.set_state(IDLE)
            self.log('waypoint list reloaded (%d points)' % len(self.waypoints))

    # -------------------------------------------------------------- inputs
    def on_fix(self, msg):
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        self.fix = (msg.latitude, msg.longitude)
        self.fix_time = self.now()
        self.gps_fix_time = self.fix_time
        self.fix_source = 'gps'

    def on_pose_estimate(self, msg):
        """Dead-reckoned position from gps_odom, used only as an opt-in backup."""
        try:
            pose = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.dead_reckon = pose if pose.get('source') == 'dead_reckoning' else None
        if not (self.dead_reckon and self.allow_dead_reckoning):
            return

        # Feed it in whenever the real fix is missing or has gone stale - not
        # just the first time, otherwise self.fix freezes at the moment GPS was
        # lost and the navigation blocks a few seconds later.
        gps_age = self.now() - self.gps_fix_time if self.gps_fix_time else None
        if gps_age is None or gps_age > self.fix_timeout:
            self.fix = (pose['lat'], pose['lon'])
            self.fix_time = self.now()
            self.fix_source = 'dead_reckoning'

    def on_telemetry(self, msg):
        data = tel.unpack(msg)
        self.heading_deg = data['heading_deg']
        self.speed_mps = data['speed_mps']
        self.fix_quality = int(data['gps_fix_quality'])
        self.satellites = int(data['gps_satellites'])
        self.hdop = data['gps_hdop']
        self.heading_ref = int(data['heading_ref'])

    def on_estop(self, msg):
        self.estop = bool(msg.data)
        if self.estop and self.state in (RUNNING, BLOCKED, DISCOVER):
            self.set_state(PAUSED)
            self.log('mission paused: emergency stop')

    def on_cmd(self, msg):
        try:
            command = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        action = command.get('action')

        if action == 'start':
            if not self.waypoints:
                self.log('cannot start: the waypoint list is empty')
                return
            if self.estop:
                self.log('cannot start: emergency stop is active, press RESET first')
                return
            self.temp_target = None          # a normal mission clears a map pick
            index = command.get('index')
            if index is not None:
                self.index = max(0, min(int(index), len(self.waypoints) - 1))
            elif self.state in (IDLE, FINISHED):
                self.index = 0
            self.start_leg()
            self.log('mission started at waypoint %d/%d' % (self.index + 1, len(self.waypoints)))
            if self.needs_discovery():
                self.start_discovery()
            else:
                self.set_state(RUNNING)
        elif action in ('pause', 'stop'):
            if action == 'stop':
                self.temp_target = None       # a map pick is not resumable
            if self.state in (RUNNING, BLOCKED, DISCOVER):
                self.set_state(PAUSED if action == 'pause' else IDLE)
                self.log('mission %s (%s)'
                         % ('paused' if action == 'pause' else 'stopped',
                            command.get('reason', 'operator')))
            self.publish_stop()
        elif action == 'resume':
            if self.estop:
                self.log('cannot resume: emergency stop is active')
                return
            if self.state in (PAUSED, IDLE) and self.waypoints:
                self.start_leg()
                self.set_state(RUNNING)
                self.log('mission resumed')
        elif action == 'next':
            self.advance('skipped by operator')
        elif action == 'goto':
            index = int(command.get('index', 0))
            if 0 <= index < len(self.waypoints):
                self.index = index
                self.start_leg()
                self.log('target set to waypoint %d' % (index + 1))
        elif action == 'goto_point':
            # "go here" from the map: a single destination, not stored in the list
            try:
                lat = float(command['lat'])
                lon = float(command['lon'])
            except (KeyError, TypeError, ValueError):
                self.log('goto_point needs numeric lat and lon')
                return
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                self.log('goto_point coordinates are out of range')
                return
            if self.estop:
                self.log('cannot drive there: emergency stop is active, press RESET first')
                return
            self.temp_target = {'name': str(command.get('name', 'จุดที่เลือก / picked'))[:32],
                                'lat': lat, 'lon': lon}
            self.start_leg()
            self.log('driving to the point picked on the map: %.7f, %.7f' % (lat, lon))
            if self.needs_discovery():
                self.start_discovery()
            else:
                self.set_state(RUNNING)
        elif action == 'reload':
            self._load_config()
            self._load_waypoints()
            self.log('configuration reloaded on request')

    # ---------------------------------------------------------------- loop
    def tick(self):
        if self.state == DISCOVER:
            self.tick_discover()
            return

        if self.state != RUNNING and self.state != BLOCKED:
            if self.state in (PAUSED, IDLE, FINISHED):
                self.publish_stop()
            return

        if self.temp_target is None and self.index >= len(self.waypoints):
            # the list was edited while driving
            self.set_state(IDLE)
            self.publish_stop()
            self.log('waypoint list is empty or shorter than before - mission stopped')
            return

        ok, reason = self.fix_usable()
        if not ok:
            if self.state != BLOCKED:
                self.set_state(BLOCKED)
                self.log('navigation blocked: %s' % reason)
            self.block_reason = reason
            self.publish_stop()
            return

        if self.state == BLOCKED:
            self.set_state(RUNNING)
            self.block_reason = ''
            self.log('fix recovered, navigation continues')
            self.leg_start_time = self.now()   # do not count the blocked time

        target = self.current_target()
        lat, lon = self.fix
        distance = geo.haversine_m(lat, lon, target['lat'], target['lon'])
        bearing = geo.bearing_deg(lat, lon, target['lat'], target['lon'])
        self.last_distance = distance
        self.last_bearing = bearing

        if distance <= self.arrive_radius:
            self.advance('reached (%.1f m)' % distance)
            return

        if self.waypoint_timeout > 0 and (self.now() - self.leg_start_time) > self.waypoint_timeout:
            self.set_state(PAUSED)
            self.publish_stop()
            self.log('waypoint %d timed out after %.0f s - mission paused'
                     % (self.index + 1, self.waypoint_timeout))
            return

        speed = self.cruise_speed
        if distance < self.approach_distance:
            speed = self.approach_speed
        if abs(geo.angle_error_deg(bearing, self.heading_deg)) > self.heading_tolerance:
            # large heading error: creep so the firmware can turn on the spot
            speed = min(speed, self.approach_speed)

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = float(bearing)
        self.pub_cmd.publish(cmd)

    # ------------------------------------------------- heading discovery
    def needs_discovery(self):
        """True when the robot has no absolute heading and could learn one by driving."""
        return (self.discover_enabled
                and self.heading_ref == 0        # relative only: no compass, no GPS course
                and self.fix is not None)

    def start_discovery(self):
        self.discover_start = self.now()
        self.discover_from = self.fix
        self.set_state(DISCOVER)
        self.log('heading unknown (no compass) - driving straight to learn it from the GPS course')

    def tick_discover(self):
        """Drive straight ahead at a crawl until the GPS course gives us a heading.

        This is the fallback for a dead or disturbed compass: the robot cannot
        know which way it points while standing still, but two metres of motion
        tell it exactly. After that the normal navigation turns it towards the
        waypoint - the firmware spins on the spot when the bearing error is
        large, so no distance is wasted going the wrong way.
        """
        if self.estop:
            self.set_state(PAUSED)
            self.publish_stop()
            return

        elapsed = self.now() - self.discover_start
        travelled = 0.0
        if self.fix and self.discover_from:
            travelled = geo.haversine_m(self.discover_from[0], self.discover_from[1],
                                        self.fix[0], self.fix[1])

        if self.heading_ref in (1, 2, 3):
            self.log('heading found (%s) after %.1f m' % (
                tel.HEADING_REF_NAMES.get(self.heading_ref, '?'), travelled))
            self.set_state(RUNNING)
            self.start_leg()
            return

        if elapsed > self.discover_max_s or travelled > self.discover_distance:
            self.log('could not establish a heading after %.1f s / %.1f m - continuing anyway, '
                     'the robot will correct itself once it moves' % (elapsed, travelled))
            self.set_state(RUNNING)
            self.start_leg()
            return

        cmd = Twist()
        cmd.linear.x = float(self.discover_speed)
        cmd.angular.z = float(self.heading_deg)     # straight ahead, whatever that is
        self.pub_cmd.publish(cmd)

    def current_target(self):
        """The point being driven to: a one-off map pick beats the list."""
        if self.temp_target is not None:
            return self.temp_target
        if 0 <= self.index < len(self.waypoints):
            return self.waypoints[self.index]
        return None

    def advance(self, why):
        if self.temp_target is not None:
            self.log('picked point %s' % why)
            self.temp_target = None
            self.set_state(FINISHED)
            self.publish_stop()
            return

        name = self.waypoints[self.index]['name'] if self.waypoints else '?'
        self.log('waypoint %d (%s) %s' % (self.index + 1, name, why))
        if self.index + 1 < len(self.waypoints):
            self.index += 1
            self.start_leg()
            return
        if self.loop_waypoints:
            self.index = 0
            self.start_leg()
            self.log('looping back to waypoint 1')
            return
        self.set_state(FINISHED)
        self.publish_stop()
        self.log('mission finished')

    def start_leg(self):
        self.leg_start_time = self.now()
        self.leg_start_distance = self.last_distance

    def fix_usable(self):
        if not self.require_fix:
            return True, ''
        # opt-in backup: a hand-set start pose + IMU odometry, for indoor tests
        if self.allow_dead_reckoning and self.dead_reckon is not None:
            return True, ''
        if self.fix is None:
            return False, 'no GPS fix yet'
        age = self.now() - self.fix_time
        if age > self.fix_timeout:
            return False, 'GPS fix is %.1f s old' % age
        if self.fix_quality <= 0:
            return False, 'GPS reports no fix'
        if self.satellites < self.min_satellites:
            return False, 'only %d satellites (need %d)' % (self.satellites, self.min_satellites)
        if self.hdop > self.max_hdop:
            return False, 'HDOP %.1f is worse than %.1f' % (self.hdop, self.max_hdop)
        return True, ''

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = float(self.heading_deg)   # hold the current heading
        self.pub_cmd.publish(cmd)

    # -------------------------------------------------------------- outputs
    def set_state(self, state):
        self.state = state

    def publish_status(self):
        target = self.current_target()
        heading_error = (geo.angle_error_deg(self.last_bearing, self.heading_deg)
                         if target else 0.0)
        eta = None
        if self.state == RUNNING and self.speed_mps > 0.05:
            eta = round(self.last_distance / self.speed_mps, 1)

        status = {
            'state': self.state,
            'index': self.index,
            'total': len(self.waypoints),
            'target': target,
            'distance_m': round(self.last_distance, 2),
            'bearing_deg': round(self.last_bearing, 1),
            'heading_error_deg': round(heading_error, 1),
            'eta_s': eta,
            'block_reason': self.block_reason,
            'estop': self.estop,
            'fix_ok': self.fix_usable()[0],
            'heading_ref': self.heading_ref,
            'fix_source': self.fix_source,
            'temp_target': self.temp_target is not None,
            'waypoints': self.waypoints,
        }
        self.pub_status.publish(String(data=json.dumps(status)))

    def log(self, message):
        self.get_logger().info(message)
        self.pub_log.publish(String(data=json.dumps({'source': 'nav', 'message': message})))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def _share_config_dir():
    import os
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('gps_localize'), 'config')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

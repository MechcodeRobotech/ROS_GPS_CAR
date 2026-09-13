#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""GPS waypoint navigation.

Kept deliberately thin: this node decides *where* to go, the ESP32 decides
*how* to get there. Every cycle it publishes one command:

    /gps_localize/cmd_move   linear.x = speed [m/s], angular.z = heading [compass deg]

and the firmware holds that heading with the IMU + PID at 100 Hz. If Wi-Fi
stutters the robot keeps driving straight instead of swerving, and if the link
stays down the firmware brakes by itself.

State machine: IDLE -> RUNNING -> (PAUSED | BLOCKED | ALIGN) -> FINISHED
  BLOCKED   the GPS fix is missing or too poor to navigate on; it clears itself
  PAUSED    the operator (or the safety watchdog) stopped the mission
  ALIGN     arrived at a waypoint that names a heading, turning on the spot to
            face it before the next leg starts
  DISCOVER  no absolute heading yet, driving straight to learn one from the
            GPS course
"""

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from gps_localize.qos import stream_qos
from sensor_msgs.msg import NavSatFix, NavSatStatus
from gps_localize_msgs.msg import Telemetry
from std_msgs.msg import Bool, String

from gps_localize import geo
from gps_localize import telemetry as tel
from gps_localize.configstore import ConfigStore

NS = '/gps_localize'

# The tightest arrival this receiver can honestly claim, whatever the config
# file says. See _load_config for the measurement behind it.
ARRIVE_RADIUS_FLOOR_M = 2.0


def parse_heading(value):
    """A waypoint's optional facing, as a compass bearing in 0..360.

    Returns None for anything that is not a usable heading - absent, blank,
    non-numeric or infinite - which is how a waypoint says "any direction will
    do". That is the normal case: a heading only matters where the robot has to
    end up pointing somewhere specific, such as a dock, a gate or a camera shot.

    Out-of-range numbers are wrapped rather than rejected. -90 and 270 are the
    same direction, and a waypoint typed by hand from a compass app is a likely
    source of both.
    """
    if value is None or value == '':
        return None
    try:
        heading = float(value)
    except (TypeError, ValueError):
        return None
    if heading != heading or heading in (float('inf'), float('-inf')):
        return None
    return heading % 360.0

IDLE = 'IDLE'
RUNNING = 'RUNNING'
PAUSED = 'PAUSED'
BLOCKED = 'BLOCKED'
FINISHED = 'FINISHED'
DISCOVER = 'DISCOVER'      # driving straight on purpose, to learn which way it points
ALIGN = 'ALIGN'            # arrived, now turning on the spot to the waypoint's own heading


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
        # How well the current position is known, from the fusion in gps_odom.
        # Arrival is judged against this, not against a fixed radius, because a
        # robot cannot honestly claim to be within a metre of anything while its
        # own position is uncertain to three.
        self.fix_sigma = None
        self.raw_fix = None            # last raw /gps/fix, the fallback
        self.fused_time = 0.0          # when gps_odom last published a fused pose
        self.inside_since = None       # when the robot first got inside the radius
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
        # The turn-on-the-spot that happens after arriving at a waypoint that
        # names a heading. None means the robot is not aligning.
        self.align_target = None
        self.align_start = 0.0
        self.align_why = ''
        # one-off destination picked on the map; it is NOT added to the list
        self.temp_target = None
        self.leg_start_time = 0.0
        self.leg_start_distance = 0.0
        self.last_distance = 0.0
        self.last_bearing = 0.0

        # ------------------------------------------------------- interfaces
        self.pub_cmd = self.create_publisher(Twist, NS + '/cmd_move', 5)
        # One-shot guard for the idle stop; see publish_stop_once().
        self._stop_sent = False
        self.pub_status = self.create_publisher(String, NS + '/nav/status', latched_qos())
        self.pub_log = self.create_publisher(String, NS + '/log', 20)

        self.create_subscription(NavSatFix, NS + '/gps/fix', self.on_fix, stream_qos(10))
        self.create_subscription(Telemetry, NS + '/telemetry', self.on_telemetry, stream_qos())
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
        # The radius asked for, and the two things that stop it from being a lie.
        #
        # ARRIVE_RADIUS_FLOOR_M is a hard lower bound. A GEP-M10 outputs a
        # position solution with no carrier phase, so its absolute accuracy has
        # a floor of roughly 1.5-2.5 m that no amount of filtering removes.
        # Measured on this robot, stationary outdoors on 8 satellites at HDOP
        # 1.3: the fix scattered 1.56 m median and 4.86 m worst. A 1 m arrival
        # radius is below the noise, so whether the robot ever "arrives" is
        # decided by luck - which is exactly what was seen, twice, as a mission
        # that drove to the point and never finished.
        #
        # ARRIVE_HOLD_S is the other half. Widening the radius alone would trade
        # never-arriving for arriving early, because a single noisy sample can
        # land inside the circle from several metres away. Requiring the robot
        # to stay inside for a continuous period rejects the single sample
        # without rejecting a genuine arrival.
        self.arrive_radius = float(cfg.get('arrive_radius_m', 2.5))
        self.arrive_hold_s = float(cfg.get('arrive_hold_s', 1.5))
        self.arrive_sigma_k = float(cfg.get('arrive_sigma_k', 1.5))
        self.heading_tolerance = float(cfg.get('heading_tolerance_deg', 15.0))
        # How long the robot may spend turning to a waypoint's heading before it
        # gives up and carries on. It advances either way - see tick_align.
        self.align_timeout = float(cfg.get('align_timeout_s', 20.0))
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
            point = {'name': str(item.get('name', 'WP%d' % (i + 1))), 'lat': lat, 'lon': lon}
            heading = parse_heading(item.get('heading_deg'))
            if heading is not None:
                point['heading_deg'] = heading
            points.append(point)
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
        now = self.now()
        self.gps_fix_time = now            # freshness, always, whoever steers
        self.raw_fix = (msg.latitude, msg.longitude)

        # The raw fix refreshes freshness but must NOT replace the fused
        # position while gps_odom is publishing one. Both callbacks write
        # self.fix, so the steering position was alternating between the
        # smoothed estimate and the raw fix that jumps - measured moving 17.6 m
        # between consecutive samples with the robot standing still, which is
        # the whole reason the fusion exists. The raw fix is still the fallback
        # the moment the fused stream goes quiet.
        if self.fused_time and (now - self.fused_time) <= self.fix_timeout:
            return
        self.fix = self.raw_fix
        self.fix_time = now
        self.fix_source = 'gps'

    def on_pose_estimate(self, msg):
        """Where gps_odom believes the robot is.

        Two different things arrive on this topic and they are not
        interchangeable, which is why the source field exists:

        'gps'             the FUSED position - dead reckoning from the robot's
                          own heading and speed, corrected by GPS. This is what
                          the robot should navigate on. The raw fix jumps: it
                          was measured moving 17.6 m between consecutive samples
                          while the robot stood still, and steering at every one
                          of those jumps is what made the path wander.
        'dead_reckoning'  no GPS at all, running from a hand-set start pose.
                          Opt-in only, because it drifts without bound.
        """
        try:
            pose = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        source = pose.get('source')

        if source == 'gps' and pose.get('lat') is not None:
            # Prefer the fused position over the raw fix. on_fix still refreshes
            # gps_fix_time, so the freshness and quality gates are unchanged -
            # this only changes WHICH position of the two is steered on.
            self.fix = (pose['lat'], pose['lon'])
            self.fix_time = self.now()
            self.fused_time = self.fix_time
            self.fix_source = 'gps'
            self.fix_sigma = pose.get('sigma_m')
            self.dead_reckon = None
            return

        self.dead_reckon = pose if source == 'dead_reckoning' else None
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
        if self.estop and self.state in (RUNNING, BLOCKED, DISCOVER, ALIGN):
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
                # Keep what it was driving to, then forget it as a target.
                #
                # A point picked on the map lived only in memory, so stopping -
                # or closing the browser, or a reboot - threw it away and the
                # next session started with nothing to go to. Somebody who has
                # just driven somewhere worth going has already decided it is
                # worth keeping; making them type it again is the sort of thing
                # that gets blamed on the robot.
                #
                # It is appended to the saved list rather than replacing it, so
                # a mission list built earlier is not lost to a single map pick.
                self.remember_temp_target()
                self.temp_target = None       # a map pick is not resumable
            if self.state in (RUNNING, BLOCKED, DISCOVER, ALIGN):
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
            heading = parse_heading(command.get('heading_deg'))
            if heading is not None:
                self.temp_target['heading_deg'] = heading
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

        if self.state == ALIGN:
            self.tick_align()
            return

        if self.state != RUNNING and self.state != BLOCKED:
            # Send the stop ONCE on entering an idle state, then say nothing.
            #
            # Republishing a zero command every tick is worse than useless: it
            # is a second command source competing with manual driving, and it
            # was doing exactly that - a jog held down on the web UI ramped up,
            # got zeroed by the next idle tick, and sawtoothed instead of
            # driving. It is also the wrong SHAPE of safety. A stop that must
            # arrive to be obeyed fails silently when the message is the thing
            # that goes missing; the firmware already stops itself when no
            # command has arrived for CMD_TIMEOUT_MS, so silence is the safe
            # state and absence does the work. Keeping quiet is therefore both
            # safer and less intrusive.
            self.publish_stop_once()
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

        radius = self.effective_arrive_radius()
        if distance <= radius:
            if self.inside_since is None:
                self.inside_since = self.now()
            held = self.now() - self.inside_since
            if held >= self.arrive_hold_s:
                self.reached('reached (%.1f m, held %.1f s inside %.1f m)'
                             % (distance, held, radius))
                return
            # Inside the circle but not yet convinced. Stand still rather than
            # keep driving: at this range the remaining "distance" is mostly the
            # receiver's own noise, so steering on it drives the robot in and out
            # of the circle and restarts this timer every time.
            self.publish_stop()
            return
        else:
            self.inside_since = None

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
                and self.fix is not None
                # A position that is stale or too poor to navigate on cannot
                # teach a heading either: the GPS course never arrives, the
                # travelled distance never grows, and the robot drives blind
                # for the whole discover_max_s. Ask the same question the
                # normal navigation asks before it moves.
                and self.fix_usable()[0])

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

        # tick() dispatches DISCOVER before its own fix_usable() gate, so this
        # is the only place that check can happen for a discovery drive. GPS
        # can go stale after the drive has started, and the robot would carry
        # on forward with nothing watching where it is.
        ok, reason = self.fix_usable()
        if not ok:
            self.set_state(BLOCKED)
            self.block_reason = reason
            self.publish_stop()
            self.log('heading discovery stopped: %s' % reason)
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

    def reached(self, why):
        """The robot is at the point. Face the right way before moving on.

        A waypoint is a place AND, optionally, a direction to be facing once the
        robot gets there. Position alone is not enough wherever what happens at
        the point depends on which way the robot points - driving onto a dock,
        lining up with a gate, aiming a camera. The heading is optional because
        most waypoints are only somewhere to be.

        Turning happens after arrival rather than during it, on purpose. This
        robot steers by driving, so any attempt to arrive already facing the
        right way costs a curved approach and lands it somewhere else. Arrive
        first, then turn on the spot - the firmware does that without GPS, from
        the compass and the gyro.
        """
        target = self.current_target()
        heading = parse_heading(target.get('heading_deg')) if target else None
        if heading is None:
            self.advance(why)
            return
        self.align_target = heading
        self.align_start = self.now()
        self.align_why = why
        self.set_state(ALIGN)
        self.log('%s - turning to face %.0f deg' % (why, heading))

    def tick_align(self):
        """Turn on the spot to the waypoint's heading, then carry on.

        The firmware owns the turn: `linear.y = 1.0` means "rotate to this
        heading and stop", and it has its own tolerance and settle time, so it
        will not hand back a heading it is still swinging through. This only
        decides when to give up.

        Giving up still advances the mission. A robot that cannot reach the
        requested facing - a wheel against a kerb, a disturbed compass - has
        still ARRIVED, and stopping the whole mission over the last few degrees
        would be a worse failure than finishing pointed slightly wrong. The log
        says which happened.
        """
        if self.estop:
            self.set_state(PAUSED)
            self.publish_stop()
            return

        error = geo.angle_error_deg(self.align_target, self.heading_deg)
        elapsed = self.now() - self.align_start

        if abs(error) <= self.heading_tolerance:
            self.advance('%s, facing %.0f deg' % (self.align_why, self.heading_deg))
            return

        if elapsed > self.align_timeout:
            self.log('could not turn to %.0f deg within %.0f s (%.0f deg off) - '
                     'continuing anyway' % (self.align_target, self.align_timeout, error))
            self.advance(self.align_why)
            return

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 1.0                        # rotate on the spot and stop
        cmd.angular.z = float(self.align_target)
        self.pub_cmd.publish(cmd)

    def advance(self, why):
        self.align_target = None
        if self.temp_target is not None:
            self.log('picked point %s' % why)
            self.temp_target = None
            self.set_state(FINISHED)
            self.publish_stop()
            return

        name = self.waypoints[self.index]['name'] if self.waypoints else '?'
        self.log('waypoint %d (%s) %s' % (self.index + 1, name, why))
        # Leaving ALIGN is not optional. advance() clears align_target, and
        # tick() dispatches on the state - so returning here while still in
        # ALIGN sent the next tick into tick_align() with no target, where
        # geo.angle_error_deg(None, heading) raised a TypeError and killed the
        # node that drives the robot. It needed a waypoint with a heading AND
        # another waypoint after it, which is why single-point tests missed it.
        if self.index + 1 < len(self.waypoints):
            self.index += 1
            self.set_state(RUNNING)
            self.start_leg()
            return
        if self.loop_waypoints:
            self.index = 0
            self.set_state(RUNNING)
            self.start_leg()
            self.log('looping back to waypoint 1')
            return
        self.set_state(FINISHED)
        self.publish_stop()
        self.log('mission finished')

    def start_leg(self):
        self.leg_start_time = self.now()
        self.leg_start_distance = self.last_distance
        # A new leg has its own arrival: never inherit the dwell from the last
        # one, or a waypoint placed close behind the previous one is declared
        # reached the moment the leg starts.
        self.inside_since = None

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
        self._stop_sent = True

    def publish_stop_once(self):
        """Stop, but only on the transition into a stopped state.

        The firmware's own command timeout is what keeps the robot stopped
        afterwards, so repeating this adds no safety and takes control away
        from whoever is driving manually.
        """
        if not self._stop_sent:
            self.publish_stop()

    # -------------------------------------------------------------- outputs
    def set_state(self, state):
        # Any state change re-arms the one-shot stop, so entering PAUSED or
        # IDLE again does send a fresh stop rather than assuming the last one
        # is still in force.
        if state != self.state:
            self._stop_sent = False
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
            'fix_source': self.live_fix_source(),
            'temp_target': self.temp_target is not None,
            'arrive_radius_m': round(self.effective_arrive_radius(), 2),
            'align_target_deg': self.align_target,
            'align_error_deg': (round(geo.angle_error_deg(self.align_target, self.heading_deg), 1)
                                if self.align_target is not None else None),
            'arrive_held_s': (round(self.now() - self.inside_since, 1)
                              if self.inside_since else 0.0),
            'fix_sigma_m': self.fix_sigma,
            'waypoints': self.waypoints,
        }
        self.pub_status.publish(String(data=json.dumps(status)))

    def effective_arrive_radius(self):
        """The radius arrival is actually judged against.

        Three things set it, and the largest wins:

        - what the config asked for
        - ARRIVE_RADIUS_FLOOR_M, below which this receiver cannot tell whether
          it has arrived or not
        - the fusion's own reported uncertainty, scaled by arrive_sigma_k

        The third is what makes it adapt. On a clean 8-satellite fix the sigma
        is around 1.5 m and the radius stays at the configured value; under
        trees or beside a building the sigma grows and the radius grows with it,
        so the robot still finishes its mission instead of circling a point it
        can no longer resolve. It never shrinks below the floor.
        """
        radius = max(self.arrive_radius, ARRIVE_RADIUS_FLOOR_M)
        if self.fix_sigma:
            radius = max(radius, self.arrive_sigma_k * self.fix_sigma)
        return radius

    def remember_temp_target(self):
        """Append the current map-picked target to the saved waypoint list.

        Only a TEMPORARY target needs this: everything reached through the
        waypoint list is in the file already. Duplicates are skipped, because
        stopping twice at the same place should not fill the list with copies -
        a metre is far below what this GPS can resolve, so anything closer than
        that is the same point.
        """
        target = self.temp_target
        if not target:
            return
        try:
            lat = float(target['lat'])
            lon = float(target['lon'])
        except (KeyError, TypeError, ValueError):
            return
        points = self.store.load('waypoints').get('waypoints', []) or []
        for p in points:
            try:
                if (abs(float(p['lat']) - lat) < 1e-5
                        and abs(float(p['lon']) - lon) < 1e-5):
                    return                      # already saved, roughly a metre
            except (KeyError, TypeError, ValueError):
                continue
        saved = {'name': 'WP%d' % (len(points) + 1), 'lat': lat, 'lon': lon}
        # Keep the heading the point was picked with, if it had one. A map pick
        # normally does not, and the robot's own heading at the moment it was
        # stopped is NOT a substitute: it is wherever the robot happened to be
        # pointing, which is not a direction anyone chose.
        heading = parse_heading(target.get('heading_deg'))
        if heading is not None:
            saved['heading_deg'] = heading
        points.append(saved)
        if self.store.save('waypoints', {'waypoints': points}):
            self.log('saved the point it was driving to (%.7f, %.7f) - '
                     '%d in the list now' % (lat, lon, len(points)))
            self.waypoints = points

    def live_fix_source(self):
        """What the position is ACTUALLY coming from, right now.

        `self.fix_source` is a latch: it is set to 'gps' the moment a fix
        arrives and only ever changes if a dead-reckoning pose turns up to
        replace it. When GPS simply stops - walking indoors, losing sky view -
        and dead reckoning is off or silent, nothing clears it, so it went on
        claiming 'gps' indefinitely.

        Seen on the robot: the dashboard banner reading "OUTDOOR: navigating on
        GPS" in green while the mission was blocked underneath it with "GPS fix
        is 2574.9 s old", zero satellites and HDOP 100. Both came from the same
        node. The banner exists precisely so nobody has to guess which position
        source is live, and it was the thing guessing.

        Freshness is a property of the reading, not of the last event, so it is
        computed when asked rather than remembered.
        """
        if not self.gps_fix_time:
            return self.fix_source if self.fix_source == 'dead_reckoning' else 'none'
        age = self.now() - self.gps_fix_time
        if age <= self.fix_timeout:
            return 'gps'
        # GPS has gone stale. Dead reckoning is only the answer if it is both
        # allowed and actually producing something.
        if self.allow_dead_reckoning and self.dead_reckon:
            return 'dead_reckoning'
        return 'none'

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

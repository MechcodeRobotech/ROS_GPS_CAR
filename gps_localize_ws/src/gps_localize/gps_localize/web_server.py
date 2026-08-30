#!/usr/bin/env python3
"""Local web UI server - the only thing the customer ever touches.

    http://<robot-ip>:8080

Serves the static UI from share/gps_localize/web, streams telemetry over
Server-Sent Events and turns button presses into ROS messages. It also owns the
YAML configuration: the Settings page reads and writes ~/.gps_localize/*.yaml
and the changes take effect immediately (PID gains are pushed straight into the
firmware, no reflash, no reboot).

Browser heartbeat: the page calls POST /api/heartbeat a few times per second.
Every beat is republished on /gps_localize/web/heartbeat, which is what the
safety watchdog watches to decide whether the network is still alive.

MEMORY: bounded log ring buffer, bounded SSE queues, capped client count, no
per-request state kept anywhere. Free heap of the ESP32 and the client count
are part of the telemetry stream so a leak shows up on screen.
"""

import json
import os
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from gps_localize_msgs.msg import Telemetry
# Float32MultiArray is still used for /config/pid, which stays a packed array:
# it is a short, positional selector list, not a set of named signals.
from std_msgs.msg import Bool, Float32MultiArray, Int32, String

from gps_localize import telemetry as tel
from gps_localize.configstore import GROUPS, ConfigStore
from gps_localize import qr
from gps_localize.httpd import (
    SseBroadcaster, WebApp, WebServer, connect_info, urls_for)

NS = '/gps_localize'


def latched_qos(depth=1):
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class WebServerNode(Node):

    def __init__(self):
        super().__init__('web_server')
        self.declare_parameter('defaults_dir', '')
        self.declare_parameter('web_dir', '')
        self.declare_parameter('runtime_config_dir', '~/.gps_localize')

        share = _share_dir()
        defaults = self.get_parameter('defaults_dir').value or os.path.join(share, 'config')
        web_dir = self.get_parameter('web_dir').value or os.path.join(share, 'web')

        self.store = ConfigStore(defaults,
                                 self.get_parameter('runtime_config_dir').value,
                                 self.get_logger())
        cfg = self.store.load('web').get('web', {})
        self.host = str(cfg.get('host', '0.0.0.0'))
        self.port = int(cfg.get('port', 8080))
        self.telemetry_rate = float(cfg.get('telemetry_rate_hz', 5.0))
        self.max_clients = int(cfg.get('max_clients', 8))
        self.log_lines = int(cfg.get('log_lines', 200))
        self.manual_speed = float(cfg.get('manual_speed_mps', 0.18))
        self.manual_turn = float(cfg.get('manual_turn_dps', 45.0))
        self.allow_config_edit = bool(cfg.get('allow_config_edit', True))
        # Port 80 as well as the usual one, so the address is just
        # http://gps-robot-web.local with nothing after it. Set to 0 to skip.
        self.friendly_port = int(cfg.get('friendly_port', 80))
        self.max_speed = float(self.store.load('robot').get('robot', {}).get('max_speed_mps', 0.42))

        # ------------------------------------------------------------ state
        self._lock = threading.Lock()
        # Dynamic subscriptions created by the browser's topic viewer, keyed by
        # topic name. Reaped once nobody is polling them.
        self._echo = {}
        self.started_at = time.time()
        self.telemetry = {}
        self.telemetry_time = 0.0
        self.fix = {}
        self.odom = {}
        self.nav = {}
        self.safety = {}
        self.pose = {}          # gps or dead-reckoned position estimate
        self.logs = deque(maxlen=max(20, self.log_lines))
        self.beats = 0

        # ------------------------------------------------------- ROS wiring
        self.pub_manual = self.create_publisher(Twist, NS + '/cmd_manual', 5)
        self.pub_cmd_move = self.create_publisher(Twist, NS + '/cmd_move', 5)
        self.pub_nav_cmd = self.create_publisher(String, NS + '/nav/cmd', 5)
        self.pub_estop = self.create_publisher(Bool, NS + '/estop_request', latched_qos())
        self.pub_beat = self.create_publisher(Int32, NS + '/web/heartbeat', 5)
        self.pub_config = self.create_publisher(Float32MultiArray, NS + '/config/pid', 5)
        self.pub_config_updated = self.create_publisher(String, NS + '/config/updated', 5)

        self.create_subscription(Telemetry, NS + '/telemetry', self.on_telemetry, 5)
        self.create_subscription(NavSatFix, NS + '/gps/fix', self.on_fix, 5)
        self.create_subscription(Odometry, NS + '/odom/gps', self.on_odom, 5)
        self.create_subscription(String, NS + '/nav/status', self.on_nav, latched_qos())
        self.create_subscription(String, NS + '/pose/estimate', self.on_pose, latched_qos())
        self.create_subscription(String, NS + '/safety', self.on_safety, latched_qos())
        self.create_subscription(String, NS + '/log', self.on_log, 20)

        # ----------------------------------------------------------- server
        self.broadcaster = SseBroadcaster(max_clients=self.max_clients)
        self.app = WebApp(web_dir, self.broadcaster, self.get_logger())
        self._register_routes()
        self.server = WebServer(self.host, self.port, self.app)
        self.server.start()

        # A second listener on port 80, serving the SAME app. Port 80 is the
        # only way a URL can carry no port at all, which is what makes it
        # something you can tell someone rather than something you have to send
        # them. The original port keeps working either way, so nothing that
        # already points at it breaks.
        #
        # Binding below 1024 needs CAP_NET_BIND_SERVICE, which the systemd unit
        # grants. When it is missing - running by hand, or an old unit - this
        # must NOT take the web UI down with it: losing the short address is a
        # small loss, losing the UI entirely is not.
        self.friendly_server = None
        if self.friendly_port and self.friendly_port != self.port:
            try:
                self.friendly_server = WebServer(self.host, self.friendly_port, self.app)
                self.friendly_server.start()
            except PermissionError:
                self.get_logger().warn(
                    'no permission to bind port %d, so the address needs its '
                    'port. Grant it with: sudo setcap '
                    "'cap_net_bind_service=+ep' $(readlink -f $(which python3))"
                    ' - or reinstall the systemd unit, which asks for it.'
                    % self.friendly_port)
                self.friendly_port = None
            except OSError as exc:
                self.get_logger().warn(
                    'port %d is not available (%s), carrying on without it'
                    % (self.friendly_port, exc))
                self.friendly_port = None
        else:
            self.friendly_port = None

        self.create_timer(1.0 / max(0.5, self.telemetry_rate), self.push_telemetry)
        self._pid_pushed = False
        self._boot_timer = self.create_timer(2.0, self.push_pid_to_firmware_once)

        # Print the address a phone can actually reach, not just localhost.
        info = connect_info(self.host, self.port, self.friendly_port)
        self.local_url, self.lan_url = info['local'], info['lan']
        self.add_log('web', 'web UI on %s%s'
                     % (self.local_url, (' · ' + self.lan_url) if self.lan_url else ''))
        self.get_logger().info('web UI ready')
        self.get_logger().info('    on this machine : %s' % self.local_url)
        if info['mdns']:
            self.get_logger().info('    by name         : %s' % info['mdns'])
        if self.lan_url:
            self.get_logger().info('    from a phone    : %s' % self.lan_url)
        else:
            self.get_logger().info('    bound to loopback only - set web.host: 0.0.0.0 '
                                   'to reach it from other devices')
        self.get_logger().info('    serving files   : %s' % web_dir)

    # =============================== ROS in ================================
    def on_telemetry(self, msg):
        data = tel.unpack(msg)
        now = time.time()
        with self._lock:
            gap = now - self.telemetry_time if self.telemetry_time else 0.0
            self.telemetry = data
            self.telemetry_time = now
        # the ESP32 was away long enough to have rebooted: it came back with the
        # gains compiled into the firmware, so push the saved ones again
        if self._pid_pushed and gap > 5.0:
            self.add_log('web', 'firmware reconnected, re-sending the saved gains')
            self.push_pid_to_firmware()

    def on_fix(self, msg):
        with self._lock:
            self.fix = {
                'lat': msg.latitude,
                'lon': msg.longitude,
                'alt': msg.altitude,
                'status': int(msg.status.status),
                'age_s': 0.0,
                'stamp': time.time(),
            }

    def on_odom(self, msg):
        with self._lock:
            self.odom = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'speed': msg.twist.twist.linear.x,
            }

    def on_nav(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self.nav = data

    def on_pose(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self.pose = data

    def on_safety(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self.safety = data

    def on_log(self, msg):
        try:
            entry = json.loads(msg.data)
        except (ValueError, TypeError):
            entry = {'source': 'ros', 'message': msg.data}
        self.add_log(entry.get('source', 'ros'), entry.get('message', ''))

    # ============================== snapshot ===============================
    def snapshot(self):
        now = time.time()
        with self._lock:
            fix = dict(self.fix)
            if fix:
                fix['age_s'] = round(now - fix.pop('stamp', now), 2)
            snap = {
                'time': now,
                'telemetry': dict(self.telemetry),
                'telemetry_age_s': round(now - self.telemetry_time, 2) if self.telemetry_time else None,
                'fix': fix,
                'odom': dict(self.odom),
                'nav': dict(self.nav),
                'safety': dict(self.safety),
                'pose': dict(self.pose),
                'server': {
                    'mode': 'robot',          # the desktop viewer reports 'viewer'
                    'clients': self.broadcaster.count(),
                    'dropped_frames': self.broadcaster.dropped,
                    'uptime_s': int(now - self.started_at),
                    'allow_config_edit': self.allow_config_edit,
                    'lan_url': self.lan_url,
                    'manual_speed_mps': self.manual_speed,
                    'manual_turn_dps': self.manual_turn,
                    'max_speed_mps': self.max_speed,
                },
            }
        return snap

    def push_telemetry(self):
        if self.broadcaster.count() == 0:
            return
        self.broadcaster.broadcast('telemetry', self.snapshot())

    def add_log(self, source, message):
        entry = {'t': time.strftime('%H:%M:%S'), 'source': source, 'message': message}
        with self._lock:
            self.logs.append(entry)
        self.broadcaster.broadcast('log', entry)

    # ============================== HTTP API ===============================
    def _register_routes(self):
        route = self.app.route
        route('GET', '/api/state', self.api_state)
        route('GET', '/api/logs', self.api_logs)
        route('GET', '/api/topics', self.api_topics)
        route('GET', '/api/connect', self.api_connect)
        route('GET', '/api/qr', self.api_qr)
        route('GET', '/api/topic', self.api_topic_echo)
        route('GET', '/api/config', self.api_config_get)
        route('POST', '/api/config', self.api_config_set)
        route('POST', '/api/config/reset', self.api_config_reset)
        route('GET', '/api/waypoints', self.api_waypoints_get)
        route('POST', '/api/waypoints', self.api_waypoints_set)
        route('POST', '/api/waypoints/add_here', self.api_waypoint_add_here)
        route('POST', '/api/nav', self.api_nav)
        route('POST', '/api/manual', self.api_manual)
        route('POST', '/api/estop', self.api_estop)
        route('POST', '/api/heartbeat', self.api_heartbeat)
        route('POST', '/api/mag_calib', self.api_mag_calib)
        route('POST', '/api/turn', self.api_turn)
        route('POST', '/api/manual_limits', self.api_manual_limits)
        route('POST', '/api/set_pose', self.api_set_pose)

    def api_state(self, _body, _query):
        return 200, {'ok': True, 'state': self.snapshot()}

    def api_logs(self, _body, _query):
        with self._lock:
            return 200, {'ok': True, 'logs': list(self.logs)}


    # --------------------------- ROS graph -------------------------------
    # There is no ros2 CLI on a Windows or Docker Desktop machine, so the
    # browser has to be able to show the whole graph: every topic, its type,
    # how many nodes are on each end, the rate, and the live value.

    def api_connect(self, _body, _query):
        """How to reach this web UI, so the Help tab never invents an address."""
        return 200, {'ok': True,
                     'connect': connect_info(self.host, self.port, self.friendly_port)}

    def api_qr(self, _body, query):
        """A QR of the best URL, as SVG so it scales and needs no image library."""
        target = ((query.get('url') or [''])[0]
                  or connect_info(self.host, self.port, self.friendly_port)['best'])
        try:
            return 200, {'ok': True, 'url': target, 'svg': qr.to_svg(target)}
        except ValueError as exc:
            return 400, {'ok': False, 'error': str(exc)}

    def api_topics(self, _body, _query):
        """Every topic in the graph, not just the ones this robot defines."""
        try:
            names_and_types = self.get_topic_names_and_types()
        except Exception as exc:
            return 500, {'ok': False, 'error': 'cannot read the ROS graph: %r' % exc}

        out = []
        now = time.time()
        with self._lock:
            rates = {name: self._topic_rate(name, now) for name, _ in names_and_types}
        for name, types in sorted(names_and_types):
            try:
                pubs = self.count_publishers(name)
                subs = self.count_subscribers(name)
            except Exception:
                pubs = subs = -1
            out.append({
                'name': name,
                'types': list(types),
                'publishers': pubs,
                'subscribers': subs,
                'hz': rates.get(name),
                'watched': name in self._echo,
            })
        return 200, {'ok': True, 'topics': out, 'count': len(out)}

    def _topic_rate(self, name, now):
        """Measured rate, or None when nothing has been seen recently."""
        st = self._echo.get(name)
        if not st or len(st['stamps']) < 2:
            return None
        stamps = list(st['stamps'])
        if now - stamps[-1] > 5.0:
            return 0.0
        span = stamps[-1] - stamps[0]
        return round((len(stamps) - 1) / span, 2) if span > 0 else None

    def api_topic_echo(self, _body, query):
        """Latest message on one topic.

        A subscription is created on first request and kept while the page
        keeps asking. Anything not polled for a while is dropped again, so
        opening the topics page does not leave subscriptions behind.
        """
        # httpd.py flattens the query string: {k: v[0]}, so these are plain
        # strings, not parse_qs lists.
        name = (query or {}).get('name', '').strip()
        if not name:
            return 400, {'ok': False, 'error': 'name is required'}

        now = time.time()
        self._reap_echo(now)

        st = self._echo.get(name)
        if st is None:
            created, err = self._start_echo(name, now)
            if not created:
                return 404, {'ok': False, 'error': err}
            return 200, {'ok': True, 'name': name, 'value': None,
                         'note': 'just subscribed, ask again in a moment'}

        st['last_poll'] = now
        with self._lock:
            value = st['value']
            stamp = st['stamp']
        return 200, {'ok': True, 'name': name, 'type': st['type'],
                     'value': value,
                     'age_s': round(now - stamp, 2) if stamp else None,
                     'hz': self._topic_rate(name, now)}

    def _start_echo(self, name, now):
        try:
            from rosidl_runtime_py.utilities import get_message
        except ImportError:
            return False, 'rosidl_runtime_py is not available'
        try:
            types = dict(self.get_topic_names_and_types()).get(name)
        except Exception as exc:
            return False, 'cannot read the ROS graph: %r' % exc
        if not types:
            return False, 'no such topic: %s' % name
        type_name = types[0]
        try:
            msg_class = get_message(type_name)
        except Exception as exc:
            return False, 'cannot load %s: %r' % (type_name, exc)

        st = {'type': type_name, 'value': None, 'stamp': 0.0,
              'last_poll': now, 'stamps': deque(maxlen=20), 'sub': None}

        def on_msg(msg, _name=name, _st=st):
            try:
                from rosidl_runtime_py.convert import message_to_ordereddict
                value = json.loads(json.dumps(message_to_ordereddict(msg),
                                              default=str))
            except Exception as exc:
                value = {'error': 'cannot convert: %r' % exc}
            t = time.time()
            with self._lock:
                _st['value'] = value
                _st['stamp'] = t
                _st['stamps'].append(t)

        try:
            st['sub'] = self.create_subscription(msg_class, name, on_msg, 10)
        except Exception as exc:
            return False, 'cannot subscribe: %r' % exc
        self._echo[name] = st
        return True, ''

    def _reap_echo(self, now, idle_s=60.0):
        """Drop subscriptions nobody is looking at any more."""
        for name in [n for n, st in self._echo.items()
                     if now - st['last_poll'] > idle_s]:
            st = self._echo.pop(name)
            try:
                if st['sub'] is not None:
                    self.destroy_subscription(st['sub'])
            except Exception:
                pass

    def api_heartbeat(self, _body, _query):
        self.beats = (self.beats + 1) % 1000000
        self.pub_beat.publish(Int32(data=self.beats))
        with self._lock:
            state = self.telemetry.get('state_name', 'UNKNOWN')
            estop = bool(self.safety.get('estop', False))
        return 200, {'ok': True, 'state': state, 'estop': estop}

    # ------------------------------------------------------------- config
    def api_config_get(self, _body, query):
        group = query.get('group')
        if group:
            if group not in GROUPS:
                return 400, {'ok': False, 'error': 'unknown group'}
            return 200, {'ok': True, 'group': group, 'data': self.store.load(group)}
        return 200, {'ok': True, 'data': self.store.load_all()}

    def api_config_set(self, body, _query):
        if not self.allow_config_edit:
            return 403, {'ok': False, 'error': 'configuration editing is locked (web.allow_config_edit)'}
        group = body.get('group')
        data = body.get('data')
        if group not in GROUPS:
            return 400, {'ok': False, 'error': 'unknown group'}
        if not isinstance(data, dict):
            return 400, {'ok': False, 'error': 'data must be an object'}
        if not self.store.save(group, data):
            return 500, {'ok': False, 'error': 'could not write the file'}

        self.pub_config_updated.publish(String(data=json.dumps({'group': group})))
        if group in ('pid', 'motor', 'estimator'):
            self.push_pid_to_firmware()
        self.add_log('web', 'settings saved: %s' % group)
        return 200, {'ok': True, 'data': self.store.load(group)}

    def api_config_reset(self, body, _query):
        if not self.allow_config_edit:
            return 403, {'ok': False, 'error': 'configuration editing is locked'}
        group = body.get('group')
        if group not in GROUPS:
            return 400, {'ok': False, 'error': 'unknown group'}
        data = self.store.reset(group)
        self.pub_config_updated.publish(String(data=json.dumps({'group': group})))
        if group in ('pid', 'motor', 'estimator'):
            self.push_pid_to_firmware()
        self.add_log('web', 'settings restored to factory defaults: %s' % group)
        return 200, {'ok': True, 'data': data}

    # ---------------------------------------------------------- waypoints
    def api_waypoints_get(self, _body, _query):
        return 200, {'ok': True, 'waypoints': self.store.load('waypoints').get('waypoints', [])}

    def api_waypoints_set(self, body, _query):
        if not self.allow_config_edit:
            return 403, {'ok': False, 'error': 'configuration editing is locked'}
        points = body.get('waypoints')
        if not isinstance(points, list):
            return 400, {'ok': False, 'error': 'waypoints must be a list'}
        clean = []
        for i, item in enumerate(points):
            try:
                lat = float(item['lat'])
                lon = float(item['lon'])
            except (KeyError, TypeError, ValueError):
                return 400, {'ok': False, 'error': 'waypoint %d needs numeric lat and lon' % (i + 1)}
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                return 400, {'ok': False, 'error': 'waypoint %d is out of range' % (i + 1)}
            name = str(item.get('name') or 'WP%d' % (i + 1))[:32]
            clean.append({'name': name, 'lat': lat, 'lon': lon})

        if not self.store.save('waypoints', {'waypoints': clean}):
            return 500, {'ok': False, 'error': 'could not write the file'}
        self.pub_config_updated.publish(String(data=json.dumps({'group': 'waypoints'})))
        self.add_log('web', 'waypoint list saved (%d points)' % len(clean))
        return 200, {'ok': True, 'waypoints': clean}

    def api_waypoint_add_here(self, body, _query):
        with self._lock:
            fix = dict(self.fix)
        if not fix or fix.get('status', -1) < 0:
            return 409, {'ok': False, 'error': 'no GPS fix yet'}
        points = self.store.load('waypoints').get('waypoints', []) or []
        name = str(body.get('name') or 'WP%d' % (len(points) + 1))[:32]
        points.append({'name': name, 'lat': float(fix['lat']), 'lon': float(fix['lon'])})
        return self.api_waypoints_set({'waypoints': points}, {})

    # ------------------------------------------------------------ commands
    # Movement commands the guard applies to. Anything that makes the robot
    # SAFER is deliberately absent: stop and pause must work at all times, and
    # so must clearing a software e-stop, or the guard would block the only way
    # out of the condition it is guarding against.
    MOTION_ACTIONS = ('start', 'resume', 'next', 'goto', 'goto_point')

    def motion_blocker(self):
        """Why the robot may not move, or None. Server-side, deliberately.

        The browser refuses these too, with a clearer message, but a browser
        guard is advice rather than enforcement: a stale tab, a page that missed
        the last frame, or anything talking to the API directly would otherwise
        still be able to ask a robot with its mushroom down to drive. The
        firmware would refuse it, correctly and silently - and silence is what
        leaves someone pressing a button that does nothing.
        """
        with self._lock:
            reasons = list(self.telemetry.get('stop_reasons') or [])
            estop = bool(self.safety.get('estop'))
            age = time.time() - self.telemetry_time if self.telemetry_time else None

        # No telemetry at all means the state is unknown, not safe. Refusing is
        # the only honest answer: the last thing to do with a robot whose state
        # we cannot read is command it to move.
        if age is None or age > 5.0:
            return {'reason': 'no_telemetry', 'clearable': False,
                    'message': 'no telemetry from the robot, so its state is '
                               'unknown - not commanding motion'}

        if 'hardware_emergency' in reasons or 'esp_button' in reasons:
            return {'reason': 'hardware_emergency', 'clearable': False,
                    'message': 'the emergency button is pressed - twist it on '
                               'the robot until it pops back up. This cannot be '
                               'cleared from software.'}
        if 'software_estop' in reasons or estop:
            return {'reason': 'software_estop', 'clearable': True,
                    'message': 'the software e-stop is latched - clear it first'}
        if 'battery_low' in reasons:
            return {'reason': 'battery_low', 'clearable': False,
                    'message': 'the battery cutoff is latched - charge or swap '
                               'the pack, then switch the robot off and on'}
        if 'agent_lost' in reasons:
            return {'reason': 'agent_lost', 'clearable': False,
                    'message': 'the robot is not connected'}
        if 'startup_lockout' in reasons:
            return {'reason': 'startup_lockout', 'clearable': False,
                    'message': 'still starting up, try again in a moment'}
        return None

    def api_nav(self, body, _query):
        action = body.get('action')
        if action not in ('start', 'pause', 'resume', 'stop', 'next', 'goto',
                          'goto_point', 'reload', 'set_datum'):
            return 400, {'ok': False, 'error': 'unknown action'}

        if action in self.MOTION_ACTIONS:
            blocker = self.motion_blocker()
            if blocker:
                self.add_log('web', 'refused %s: %s' % (action, blocker['message']))
                return 409, {'ok': False, 'error': blocker['message'],
                             'blocked_by': blocker['reason'],
                             'clearable': blocker['clearable']}
        if action == 'goto_point':
            try:
                lat = float(body['lat'])
                lon = float(body['lon'])
            except (KeyError, TypeError, ValueError):
                return 400, {'ok': False, 'error': 'goto_point needs numeric lat and lon'}
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                return 400, {'ok': False, 'error': 'coordinates out of range'}
        self.pub_nav_cmd.publish(String(data=json.dumps(body)))
        self.add_log('web', 'navigation command: %s' % action)
        return 200, {'ok': True}

    def api_manual(self, body, _query):
        # Stop first and unconditionally: it is the one command that must never
        # be refused, whatever else is wrong.
        if body.get('action') == 'stop':
            self.pub_manual.publish(Twist())
            return 200, {'ok': True, 'speed': 0.0, 'turn': 0.0}

        blocker = self.motion_blocker()
        if blocker:
            return 409, {'ok': False, 'error': blocker['message'],
                         'blocked_by': blocker['reason'],
                         'clearable': blocker['clearable']}

        try:
            forward = float(body.get('forward', 0.0))
            turn = float(body.get('turn', 0.0))
        except (TypeError, ValueError):
            return 400, {'ok': False, 'error': 'forward and turn must be numbers'}
        forward = max(-1.0, min(1.0, forward))
        turn = max(-1.0, min(1.0, turn))

        cmd = Twist()
        cmd.linear.x = forward * self.manual_speed
        cmd.angular.z = turn * self.manual_turn      # deg/s, clockwise positive
        self.pub_manual.publish(cmd)
        return 200, {'ok': True, 'speed': cmd.linear.x, 'turn': cmd.angular.z}

    COMPASS_POINTS = {'N': 0.0, 'NE': 45.0, 'E': 90.0, 'SE': 135.0,
                      'S': 180.0, 'SW': 225.0, 'W': 270.0, 'NW': 315.0}

    def api_turn(self, body, _query):
        """Rotate on the spot. Works indoors, no GPS needed.

        {"deg": 90}       turn 90 deg clockwise from where it points now
        {"deg": -45}      turn 45 deg anticlockwise
        {"heading": 270}  turn to an absolute compass heading
        {"compass": "N"}  turn to north / NE / E / ... / NW
        """
        with self._lock:
            current = float(self.telemetry.get('heading_deg', 0.0))
            has_telemetry = bool(self.telemetry)

        if not has_telemetry:
            return 409, {'ok': False, 'error': 'no telemetry from the robot yet'}

        if 'compass' in body:
            point = str(body.get('compass', '')).upper()
            if point not in self.COMPASS_POINTS:
                return 400, {'ok': False, 'error': 'compass must be one of N NE E SE S SW W NW'}
            target = self.COMPASS_POINTS[point]
            label = point
        elif 'heading' in body:
            try:
                target = float(body['heading']) % 360.0
            except (TypeError, ValueError):
                return 400, {'ok': False, 'error': 'heading must be a number'}
            label = '%.0f deg' % target
        else:
            try:
                delta = float(body.get('deg', 0.0))
            except (TypeError, ValueError):
                return 400, {'ok': False, 'error': 'deg must be a number'}
            if abs(delta) < 0.5:
                return 400, {'ok': False, 'error': 'angle too small'}
            target = (current + delta) % 360.0
            label = '%+.0f deg' % delta

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 1.0            # firmware flag: rotate on the spot, then stop
        cmd.angular.z = float(target)
        self.pub_cmd_move.publish(cmd)
        self.add_log('web', 'turn %s (target heading %.0f)' % (label, target))
        return 200, {'ok': True, 'target_heading': target, 'from': current}

    def api_set_pose(self, body, _query):
        """Start pose dragged on the map (RViz "2D Pose Estimate" style).

        {"lat": .., "lon": .., "heading_deg": ..}

        Two things happen: the firmware pins its heading to the arrow direction
        (topic /config/pid selector 7) and gps_odom takes the point as the
        origin it dead-reckons from while there is no GPS. Nothing here fakes a
        GPS fix - the estimate is published separately and marked untrusted.
        """
        try:
            lat = float(body['lat'])
            lon = float(body['lon'])
            heading = float(body.get('heading_deg', 0.0)) % 360.0
        except (KeyError, TypeError, ValueError):
            return 400, {'ok': False, 'error': 'need numeric lat, lon and heading_deg'}
        if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return 400, {'ok': False, 'error': 'coordinates out of range'}

        msg = Float32MultiArray()
        msg.data = [float(tel.CONFIG_SET_HEADING), heading]
        self.pub_config.publish(msg)

        self.pub_nav_cmd.publish(String(data=json.dumps(
            {'action': 'set_pose', 'lat': lat, 'lon': lon, 'heading': heading})))

        self.add_log('web', 'start pose set by hand: %.7f, %.7f facing %.0f deg'
                     % (lat, lon, heading))
        return 200, {'ok': True, 'lat': lat, 'lon': lon, 'heading_deg': heading}

    def api_manual_limits(self, body, _query):
        """The + / - buttons: change how fast manual driving and turning are.

        {"speed_mps": 0.22}          set the forward speed
        {"speed_delta": 0.02}        nudge it up or down
        {"turn_dps": 60}             set the turn rate
        {"turn_delta": -5}           nudge it
        {"save": true}               also write it to web.yaml as the new default
        """
        speed = self.manual_speed
        turn = self.manual_turn
        try:
            if 'speed_mps' in body:
                speed = float(body['speed_mps'])
            if 'speed_delta' in body:
                speed += float(body['speed_delta'])
            if 'turn_dps' in body:
                turn = float(body['turn_dps'])
            if 'turn_delta' in body:
                turn += float(body['turn_delta'])
        except (TypeError, ValueError):
            return 400, {'ok': False, 'error': 'values must be numbers'}

        # keep it inside what the robot can actually do
        speed = max(0.02, min(speed, self.max_speed))
        turn = max(5.0, min(turn, 180.0))
        self.manual_speed = round(speed, 3)
        self.manual_turn = round(turn, 1)

        saved = False
        if body.get('save') and self.allow_config_edit:
            cfg = self.store.load('web')
            cfg.setdefault('web', {})['manual_speed_mps'] = self.manual_speed
            cfg['web']['manual_turn_dps'] = self.manual_turn
            saved = self.store.save('web', cfg)
            if saved:
                self.pub_config_updated.publish(String(data=json.dumps({'group': 'web'})))
                self.add_log('web', 'manual speed %.2f m/s, turn %.0f deg/s saved as default'
                             % (self.manual_speed, self.manual_turn))

        return 200, {'ok': True, 'speed_mps': self.manual_speed,
                     'turn_dps': self.manual_turn, 'saved': saved,
                     'max_speed_mps': self.max_speed}

    def api_estop(self, body, _query):
        stop = bool(body.get('stop', True))
        self.pub_estop.publish(Bool(data=stop))
        if stop:
            self.pub_manual.publish(Twist())
            self.pub_nav_cmd.publish(String(data=json.dumps({'action': 'pause', 'reason': 'e-stop'})))
        self.add_log('web', 'EMERGENCY STOP' if stop else 'emergency stop reset')
        return 200, {'ok': True, 'estop': stop}

    # ---------------------------------------------------- firmware config
    def push_pid_to_firmware_once(self):
        """One shot, a couple of seconds after start: make the firmware match the YAML."""
        if self._pid_pushed:
            return
        self._pid_pushed = True
        self._boot_timer.cancel()
        self.push_pid_to_firmware()

    def push_pid_to_firmware(self):
        """Send every firmware-side setting: gains, estimator, motor model."""
        pid = self.store.load('pid').get('pid', {})
        est = self.store.load('estimator').get('estimator', {})
        cfg = {'motor': self.store.load('motor').get('motor', {})}

        for selector, name in ((tel.CONFIG_HEADING_PID, 'heading'), (tel.CONFIG_SPEED_PID, 'speed')):
            block = pid.get(name, {})
            if not block:
                continue
            msg = Float32MultiArray()
            msg.data = [
                float(selector),
                float(block.get('kp', 0.0)),
                float(block.get('ki', 0.0)),
                float(block.get('kd', 0.0)),
                float(block.get('kf', 0.0)),
                float(block.get('tol', 0.0)),
                float(block.get('i_min', -1000.0)),
                float(block.get('i_max', 1000.0)),
            ]
            self.pub_config.publish(msg)

        if est:
            msg = Float32MultiArray()
            msg.data = [
                float(tel.CONFIG_ESTIMATOR),
                float(est.get('heading_source', 3)),
                float(est.get('align_min_mps', 0.25)),
                float(est.get('align_tau_s', 6.0)),
                float(est.get('gps_tau_s', 1.5)),
                float(est.get('model_tau_s', 2.5)),
                float(est.get('track_m', 0.30)),
                1.0 if est.get('mag_enable', True) else 0.0,
                float(est.get('mag_tau_s', 4.0)),
                float(est.get('mag_still_mps', 0.10)),
                float(est.get('mag_declination_deg', 0.0)),
                float(est.get('mag_mount_offset_deg', 0.0)),
            ]
            self.pub_config.publish(msg)

        motor = cfg.get('motor', {})
        if motor:
            msg = Float32MultiArray()
            msg.data = [
                float(tel.CONFIG_MOTOR_RAMP),
                float(motor.get('accel_ramp_s', 0.6)),
                float(motor.get('decel_ramp_s', 0.3)),
            ]
            self.pub_config.publish(msg)

            msg = Float32MultiArray()
            msg.data = [
                float(tel.CONFIG_MOTOR_MODEL),
                float(motor.get('ke_v_per_rpm', 0.12)),
                float(motor.get('resistance_ohm', 5.0)),
                float(motor.get('stall_current_a', 0.9)),
                # each motor has its own series diode (schematic draft_5)
                float(motor.get('diode_a_vf_v', 0.45)),
                float(motor.get('diode_a_r_ohm', 0.03)),
                float(motor.get('diode_b_vf_v', 0.45)),
                float(motor.get('diode_b_r_ohm', 0.03)),
            ]
            self.pub_config.publish(msg)
        self.get_logger().info('controller gains pushed to the firmware')

    def api_mag_calib(self, body, _query):
        """Compass calibration: start, turn the robot in a slow full circle, finish."""
        start = bool(body.get('start', True))
        msg = Float32MultiArray()
        msg.data = [float(tel.CONFIG_MAG_CALIB), 1.0 if start else 0.0]
        self.pub_config.publish(msg)
        self.add_log('web', 'compass calibration %s' % ('started - turn the robot slowly through a full circle'
                                                        if start else 'finished - result saved on the board'))
        return 200, {'ok': True, 'calibrating': start}

    # ------------------------------------------------------------ shutdown
    def close(self):
        try:
            self.server.stop()
        except OSError:
            pass


def _share_dir():
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('gps_localize')


def _display_host(host):
    return 'localhost' if host in ('0.0.0.0', '') else host


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

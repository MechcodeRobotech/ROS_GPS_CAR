#!/usr/bin/env python3
"""GPS_Localize Viewer - the web UI without ROS.

Runs on a plain Windows or Ubuntu machine (no ROS 2, no robot) so you can look
at the interface, learn where everything is and prepare waypoints/settings
files before going to the field.

    VIEW ONLY. It cannot drive the robot. The driving buttons are disabled and
    the page says so. Telemetry is simulated and labelled as such.

To actually control the robot, run the ROS stack on the robot PC
(`ros2 launch gps_localize bringup.launch.py`) and open its address from any
phone, tablet or laptop on the same Wi-Fi: http://<robot-ip>:8080

Usage:
    python viewer_server.py                    # localhost only, opens the browser
    python viewer_server.py --host 0.0.0.0     # let other devices see the demo
    python viewer_server.py --no-demo          # blank values instead of simulated ones
    python viewer_server.py --config-dir ./my_settings
"""

import argparse
import math
import os
import sys
import threading
import time
import webbrowser
from collections import deque

# This program prints Thai. On Windows the console (or a redirected file) often
# defaults to a legacy code page that cannot encode it, and Python then raises
# UnicodeEncodeError and the whole server dies before it starts serving. Force
# UTF-8 and never let a character kill the process.
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding='utf-8', errors='replace')
    except (AttributeError, ValueError, OSError):
        pass

# --------------------------------------------------------------- asset paths
# Works in three situations: frozen exe (PyInstaller), inside this repository,
# and installed next to the ROS package.
HERE = os.path.dirname(os.path.abspath(__file__))


def _find_assets():
    if getattr(sys, 'frozen', False):                      # PyInstaller bundle
        base = getattr(sys, '_MEIPASS', HERE)
        return os.path.join(base, 'web'), os.path.join(base, 'config'), os.path.dirname(sys.executable)
    pkg = os.path.normpath(os.path.join(HERE, '..', 'gps_localize_ws', 'src', 'gps_localize'))
    return os.path.join(pkg, 'web'), os.path.join(pkg, 'config'), HERE


WEB_DIR, CONFIG_DEFAULTS_DIR, APP_DIR = _find_assets()

# the shared HTTP/SSE server and the YAML store live in the ROS package, but
# neither of them imports rclpy - they work standalone
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..', 'gps_localize_ws', 'src', 'gps_localize')))
try:
    from gps_localize.configstore import GROUPS, ConfigStore
    from gps_localize import qr
    from gps_localize.httpd import SseBroadcaster, WebApp, WebServer, urls_for
except ImportError as exc:
    # Only fall back when it is OUR package that is missing. A broad except
    # here used to swallow a missing third-party dependency and then fail on
    # the fallback instead, reporting "No module named 'configstore'" when the
    # real problem was pyyaml - which nobody could guess from that message.
    # Seen for real on a fresh Windows machine.
    # exc.name is the module the failing import ASKED for, but when the failure
    # happens inside our package (configstore doing 'import yaml') it can come
    # back empty, so fall back to parsing the message.
    missing = getattr(exc, 'name', None) or ''
    if not missing:
        import re as _re
        m = _re.search(r"No module named '?([\w.]+)'?", str(exc))
        missing = m.group(1) if m else ''
    if missing.split('.')[0] not in ('gps_localize', 'configstore', 'httpd'):
        raise SystemExit(
            "\nGPS_Localize viewer cannot start.\n"
            "\n  missing module : %s\n"
            "  install it with: pip install %s\n"
            "\nEverything else it needs is in the Python standard library.\n"
            "Original error: %s\n"
            % (missing or 'unknown',
               {'yaml': 'pyyaml'}.get(missing, missing or 'pyyaml'), exc)
        ) from exc
    try:                                   # frozen build: modules are bundled flat
        from configstore import GROUPS, ConfigStore            # type: ignore
        import qr                                              # type: ignore
        from httpd import SseBroadcaster, WebApp, WebServer, urls_for   # type: ignore
    except ImportError as inner:
        raise SystemExit(
            "\nGPS_Localize viewer cannot find its own modules.\n"
            "Run it from inside the project so gps_localize_ws/ is alongside webapp/.\n"
            "  looked in: %s\n  error: %s\n" % (sys.path[0], inner)
        ) from inner


BANNER = r"""
  GPS_Localize Viewer  -  VIEW ONLY / โหมดดูอย่างเดียว
  ---------------------------------------------------
  This window is NOT connected to a robot.
  หน้าต่างนี้ไม่ได้ต่อกับหุ่นยนต์จริง ควบคุมหุ่นยนต์ไม่ได้
"""


class DemoRobot:
    """Simulated robot so the pages are not empty. Clearly flagged as simulated.

    It also reacts to the buttons, so the whole interface can be demonstrated and
    taught on a Windows laptop with no hardware: drive it by hand, send it to a
    point picked on the map, turn it by an angle, set its start pose. Nothing
    here can reach a real robot - this process has no ROS and no radio.
    """

    def __init__(self, waypoints):
        self.waypoints = waypoints or []
        self.index = 0
        self.lat = self.waypoints[0]['lat'] if self.waypoints else 13.736717
        self.lon = self.waypoints[0]['lon'] if self.waypoints else 100.523186
        self.heading = 0.0
        self.speed = 0.0
        self.t0 = time.time()
        self.mode = 'auto'          # auto | manual | goto | turn | stopped
        self.manual = (0.0, 0.0)    # (forward m/s, turn deg/s)
        self.goto = None            # {'lat','lon'}
        self.turn_target = None     # compass degrees
        self.estop = False

    # -------------------------------------------------- commands from the UI
    def drive(self, forward, turn):
        self.mode = 'manual'
        self.manual = (forward, turn)

    def go_to(self, lat, lon):
        self.mode = 'goto'
        self.goto = {'lat': lat, 'lon': lon}

    def turn_to(self, heading):
        self.mode = 'turn'
        self.turn_target = heading % 360.0

    def set_pose(self, lat, lon, heading):
        self.lat, self.lon, self.heading = lat, lon, heading % 360.0
        self.speed = 0.0
        self.mode = 'stopped'

    def stop(self):
        self.mode = 'stopped'
        self.speed = 0.0
        self.manual = (0.0, 0.0)

    def resume_mission(self):
        self.estop = False
        self.mode = 'auto'

    # ------------------------------------------------------------- physics
    def move(self, dt, heading_cmd, speed_cmd, turn_rate=None):
        if turn_rate is not None:
            self.heading = (self.heading + turn_rate * dt) % 360.0
        elif heading_cmd is not None:
            error = (heading_cmd - self.heading + 540.0) % 360.0 - 180.0
            self.heading = (self.heading + max(-60.0 * dt, min(60.0 * dt, error))) % 360.0
        self.speed = speed_cmd
        step = speed_cmd * dt
        self.lat += math.cos(math.radians(self.heading)) * step / 110540.0
        self.lon += math.sin(math.radians(self.heading)) * step / (111320.0 * math.cos(math.radians(self.lat)))

    def step(self, dt):
        if self.estop or self.mode == 'stopped':
            self.speed = 0.0
            return

        if self.mode == 'manual':
            forward, turn = self.manual
            self.move(dt, None, forward, turn_rate=turn)
            return

        if self.mode == 'turn' and self.turn_target is not None:
            error = (self.turn_target - self.heading + 540.0) % 360.0 - 180.0
            if abs(error) < 2.0:
                self.mode = 'stopped'
                self.turn_target = None
                self.speed = 0.0
                return
            self.move(dt, None, 0.0, turn_rate=math.copysign(45.0, error))
            return

        if self.mode == 'goto' and self.goto is not None:
            target = self.goto
        elif self.waypoints:
            target = self.waypoints[self.index % len(self.waypoints)]
        else:
            return
        dlat = target['lat'] - self.lat
        dlon = target['lon'] - self.lon
        east = dlon * 111320.0 * math.cos(math.radians(self.lat))
        north = dlat * 110540.0
        distance = math.hypot(east, north)
        if distance < 1.0:
            if self.mode == 'goto':          # a one-off destination: arrive and stop
                self.mode = 'stopped'
                self.goto = None
                self.speed = 0.0
            else:
                self.index += 1
            return

        bearing = (math.degrees(math.atan2(east, north)) + 360.0) % 360.0
        error = (bearing - self.heading + 540.0) % 360.0 - 180.0
        self.move(dt, bearing, 0.25 if abs(error) < 20.0 else 0.12)

    def telemetry(self):
        yaw_rate = math.sin((time.time() - self.t0) * 0.7) * 4.0
        label = {'auto': 'AUTO', 'manual': 'MANUAL', 'goto': 'AUTO',
                 'turn': 'MANUAL', 'stopped': 'IDLE'}.get(self.mode, 'IDLE')
        if self.estop:
            label = 'ESTOP'
        return {
            'state': 1.0, 'state_name': label + ' (SIMULATED)',
            'heading_deg': self.heading,
            'target_heading_deg': self.heading,
            'heading_error_deg': 0.0,
            'speed_mps': self.speed, 'target_speed_mps': 0.25,
            'wheel_rpm_left': self.speed / (math.pi * 0.081) * 60.0,
            'wheel_rpm_right': self.speed / (math.pi * 0.081) * 60.0,
            'wheel_mps_left': self.speed, 'wheel_mps_right': self.speed,
            'yaw_rate_dps': yaw_rate,
            'pwm_left': 480.0, 'pwm_right': 500.0,
            'battery_v': 11.6, 'battery_a': 1.4,
            'motor_a_v': 11.5, 'motor_a_a': 0.6,
            'motor_b_v': 11.5, 'motor_b_a': 0.65,
            'gps_fix_quality': 1.0, 'gps_satellites': 11.0, 'gps_hdop': 0.9,
            'gps_speed_mps': self.speed, 'gps_course_deg': self.heading,
            'stop_flags': 0.0, 'stop_reasons': [],
            'free_heap_kb': 168.0, 'loop_us': 2200.0,
            'odom_x': 0.0, 'odom_y': 0.0,
        }


class Viewer:

    def __init__(self, args):
        self.args = args
        self.store = ConfigStore(CONFIG_DEFAULTS_DIR, args.config_dir)
        self.logs = deque(maxlen=200)
        self.started = time.time()
        self.lock = threading.Lock()

        waypoints = self.store.load('waypoints').get('waypoints', [])
        self.demo = DemoRobot(waypoints) if args.demo else None
        web_cfg = self.store.load('web').get('web', {})
        self.manual_speed = float(web_cfg.get('manual_speed_mps', 0.18))
        self.manual_turn = float(web_cfg.get('manual_turn_dps', 45.0))

        self.broadcaster = SseBroadcaster(max_clients=8)
        self.app = WebApp(WEB_DIR, self.broadcaster)
        self._routes()
        self.server = WebServer(args.host, args.port, self.app)

        self.log('viewer', 'VIEW ONLY - this window cannot control a robot')
        if self.demo:
            self.log('viewer', 'telemetry below is SIMULATED, not from real hardware')
        self.log('viewer', 'settings are read from and written to %s' % os.path.abspath(args.config_dir))

    # ------------------------------------------------------------- routing
    def _routes(self):
        route = self.app.route
        route('GET', '/api/state', lambda body, query: (200, {'ok': True, 'state': self.snapshot()}))
        route('GET', '/api/logs', lambda body, query: (200, {'ok': True, 'logs': list(self.logs)}))
        route('POST', '/api/heartbeat', lambda body, query: (200, {'ok': True, 'state': 'VIEWER', 'estop': False}))
        route('GET', '/api/topics', self.topics_list)
        route('GET', '/api/connect', self.api_connect)
        route('GET', '/api/qr', self.api_qr)
        route('GET', '/api/topic', self.topic_echo)
        route('GET', '/api/config', self.config_get)
        route('POST', '/api/config', self.config_set)
        route('POST', '/api/config/reset', self.config_reset)
        route('GET', '/api/waypoints', self.waypoints_get)
        route('POST', '/api/waypoints', self.waypoints_set)
        route('POST', '/api/waypoints/add_here', self.add_here)
        # These drive the SIMULATION only - there is no ROS and no radio in this
        # process, so they physically cannot reach a robot. Every reply says so.
        route('POST', '/api/nav', self.sim_nav)
        route('POST', '/api/manual', self.sim_manual)
        route('POST', '/api/turn', self.sim_turn)
        route('POST', '/api/estop', self.sim_estop)
        route('POST', '/api/set_pose', self.sim_set_pose)
        route('POST', '/api/mag_calib', self.sim_unsupported)
        route('POST', '/api/manual_limits', self.sim_limits)

    def sim_unsupported(self, _body, _query):
        return 200, {'ok': True, 'simulated': True,
                     'note': 'ไม่มีฮาร์ดแวร์ในโหมดจำลอง / no hardware in the simulator'}

    def no_demo(self):
        return 409, {'ok': False, 'simulated': True,
                     'error': 'ปิดโหมดจำลองอยู่ (--no-demo) / simulation disabled'}

    # ---------------------------------------------------- simulated control
    def sim_nav(self, body, _query):
        if not self.demo:
            return self.no_demo()
        action = body.get('action')
        if action in ('start', 'resume'):
            self.demo.resume_mission()
        elif action in ('pause', 'stop'):
            self.demo.stop()
        elif action == 'next':
            self.demo.index += 1
            self.demo.mode = 'auto'
        elif action == 'goto':
            self.demo.index = int(body.get('index', 0))
            self.demo.mode = 'auto'
        elif action == 'goto_point':
            try:
                self.demo.go_to(float(body['lat']), float(body['lon']))
            except (KeyError, TypeError, ValueError):
                return 400, {'ok': False, 'error': 'goto_point needs lat and lon'}
        self.log('sim', 'navigation: %s' % action)
        return 200, {'ok': True, 'simulated': True}

    def sim_manual(self, body, _query):
        if not self.demo:
            return self.no_demo()
        if body.get('action') == 'stop':
            self.demo.stop()
            return 200, {'ok': True, 'simulated': True, 'speed': 0.0, 'turn': 0.0}
        try:
            forward = max(-1.0, min(1.0, float(body.get('forward', 0.0))))
            turn = max(-1.0, min(1.0, float(body.get('turn', 0.0))))
        except (TypeError, ValueError):
            return 400, {'ok': False, 'error': 'forward and turn must be numbers'}
        self.demo.drive(forward * self.manual_speed, turn * self.manual_turn)
        return 200, {'ok': True, 'simulated': True,
                     'speed': forward * self.manual_speed, 'turn': turn * self.manual_turn}

    def sim_turn(self, body, _query):
        if not self.demo:
            return self.no_demo()
        points = {'N': 0.0, 'NE': 45.0, 'E': 90.0, 'SE': 135.0,
                  'S': 180.0, 'SW': 225.0, 'W': 270.0, 'NW': 315.0}
        if 'compass' in body:
            target = points.get(str(body['compass']).upper())
            if target is None:
                return 400, {'ok': False, 'error': 'unknown compass point'}
        elif 'heading' in body:
            target = float(body['heading']) % 360.0
        else:
            target = (self.demo.heading + float(body.get('deg', 0.0))) % 360.0
        self.demo.turn_to(target)
        return 200, {'ok': True, 'simulated': True, 'target_heading': target}

    def sim_estop(self, body, _query):
        if not self.demo:
            return self.no_demo()
        stop = bool(body.get('stop', True))
        self.demo.estop = stop
        if stop:
            self.demo.stop()
        self.log('sim', 'EMERGENCY STOP (simulated)' if stop else 'e-stop reset (simulated)')
        return 200, {'ok': True, 'simulated': True, 'estop': stop}

    def sim_set_pose(self, body, _query):
        if not self.demo:
            return self.no_demo()
        try:
            lat = float(body['lat'])
            lon = float(body['lon'])
            heading = float(body.get('heading_deg', 0.0)) % 360.0
        except (KeyError, TypeError, ValueError):
            return 400, {'ok': False, 'error': 'need numeric lat, lon and heading_deg'}
        self.demo.set_pose(lat, lon, heading)
        self.log('sim', 'start pose set: %.7f, %.7f facing %.0f deg' % (lat, lon, heading))
        return 200, {'ok': True, 'simulated': True,
                     'lat': lat, 'lon': lon, 'heading_deg': heading}

    def sim_limits(self, body, _query):
        try:
            if 'speed_mps' in body:
                self.manual_speed = float(body['speed_mps'])
            if 'speed_delta' in body:
                self.manual_speed += float(body['speed_delta'])
            if 'turn_dps' in body:
                self.manual_turn = float(body['turn_dps'])
            if 'turn_delta' in body:
                self.manual_turn += float(body['turn_delta'])
        except (TypeError, ValueError):
            return 400, {'ok': False, 'error': 'values must be numbers'}
        self.manual_speed = max(0.02, min(self.manual_speed, 0.42))
        self.manual_turn = max(5.0, min(self.manual_turn, 180.0))
        return 200, {'ok': True, 'simulated': True,
                     'speed_mps': round(self.manual_speed, 3),
                     'turn_dps': round(self.manual_turn, 1)}

    # -------------------------------------------------------------- config
    # --------------------------- ROS graph -------------------------------
    # The viewer has no ROS, so it answers with the topics the robot WOULD
    # publish, clearly flagged as simulated. That keeps the topics page usable
    # as a demo and keeps the endpoint set identical to the robot server -
    # which QC checks, because a page that works against the robot must not
    # 404 here.

    DEMO_TOPICS = [
        ('/gps_localize/odom', 'nav_msgs/msg/Odometry', 1, 1),
        ('/gps_localize/imu/data', 'sensor_msgs/msg/Imu', 1, 0),
        ('/gps_localize/gps/fix', 'sensor_msgs/msg/NavSatFix', 1, 1),
        ('/gps_localize/telemetry', 'gps_localize_msgs/msg/Telemetry', 1, 1),
        ('/gps_localize/cmd_move', 'geometry_msgs/msg/Twist', 1, 1),
        ('/gps_localize/cmd_manual', 'geometry_msgs/msg/Twist', 1, 1),
        ('/gps_localize/cmd_vel', 'geometry_msgs/msg/Twist', 0, 1),
        ('/gps_localize/heartbeat', 'std_msgs/msg/Int32', 1, 1),
        ('/gps_localize/estop', 'std_msgs/msg/Bool', 1, 1),
        ('/gps_localize/config/pid', 'std_msgs/msg/Float32MultiArray', 1, 1),
        ('/gps_localize/log', 'std_msgs/msg/String', 1, 1),
        ('/gps_localize/safety', 'std_msgs/msg/Int32', 1, 1),
    ]

    def api_connect(self, _body, _query):
        """Same shape as the robot server, so the Help tab needs no special case.

        The viewer runs on a Windows PC with no avahi, so there is no mDNS name
        to offer - saying so plainly is better than printing an address that
        will not resolve.
        """
        return 200, {'ok': True, 'connect': {
            'local': 'http://localhost:%d' % self.port,
            'lan': None, 'mdns': None, 'mdns_name': None,
            'best': 'http://localhost:%d' % self.port,
            'ip': None, 'port': self.port, 'friendly_port': None,
            'qr_url': 'http://localhost:%d' % self.port,
            'mdns_may_be_blocked': False,
        }}

    def api_qr(self, _body, query):
        target = ((query.get('url') or [''])[0]
                  or 'http://localhost:%d' % self.port)
        try:
            return 200, {'ok': True, 'url': target, 'svg': qr.to_svg(target)}
        except ValueError as exc:
            return 400, {'ok': False, 'error': str(exc)}

    def topics_list(self, _body, _query):
        topics = [{'name': n, 'types': [t], 'publishers': p, 'subscribers': sc,
                   'hz': None, 'watched': False, 'simulated': True}
                  for n, t, p, sc in self.DEMO_TOPICS]
        return 200, {'ok': True, 'topics': topics, 'count': len(topics),
                     'simulated': True,
                     'note': 'viewer has no ROS - this is the topic list the '
                             'robot would publish'}

    def topic_echo(self, _body, query):
        name = (query or {}).get('name', '').strip()
        if not name:
            return 400, {'ok': False, 'error': 'name is required'}
        match = next((t for t in self.DEMO_TOPICS if t[0] == name), None)
        if match is None:
            return 404, {'ok': False, 'error': 'no such topic: %s' % name}
        return 200, {'ok': True, 'name': name, 'type': match[1], 'value': None,
                     'simulated': True,
                     'note': 'viewer has no ROS, so there is no live value. '
                             'Open the robot address to see real data.'}

    def config_get(self, _body, query):
        group = query.get('group')
        if group:
            if group not in GROUPS:
                return 400, {'ok': False, 'error': 'unknown group'}
            return 200, {'ok': True, 'group': group, 'data': self.store.load(group)}
        return 200, {'ok': True, 'data': self.store.load_all()}

    def config_set(self, body, _query):
        group = body.get('group')
        data = body.get('data')
        if group not in GROUPS or not isinstance(data, dict):
            return 400, {'ok': False, 'error': 'bad request'}
        if not self.store.save(group, data):
            return 500, {'ok': False, 'error': 'could not write the file'}
        self.log('viewer', 'saved %s.yaml (local file only, the robot is not updated)' % group)
        return 200, {'ok': True, 'data': self.store.load(group)}

    def config_reset(self, body, _query):
        group = body.get('group')
        if group not in GROUPS:
            return 400, {'ok': False, 'error': 'unknown group'}
        return 200, {'ok': True, 'data': self.store.reset(group)}

    # ----------------------------------------------------------- waypoints
    def waypoints_get(self, _body, _query):
        return 200, {'ok': True, 'waypoints': self.store.load('waypoints').get('waypoints', [])}

    def waypoints_set(self, body, _query):
        points = body.get('waypoints')
        if not isinstance(points, list):
            return 400, {'ok': False, 'error': 'waypoints must be a list'}
        clean = []
        for i, item in enumerate(points):
            try:
                lat, lon = float(item['lat']), float(item['lon'])
            except (KeyError, TypeError, ValueError):
                return 400, {'ok': False, 'error': 'waypoint %d needs numeric lat and lon' % (i + 1)}
            clean.append({'name': str(item.get('name') or 'WP%d' % (i + 1))[:32], 'lat': lat, 'lon': lon})
        if not self.store.save('waypoints', {'waypoints': clean}):
            return 500, {'ok': False, 'error': 'could not write the file'}
        if self.demo:
            self.demo.waypoints = clean
        self.log('viewer', 'waypoints saved to %s' % self.store.path('waypoints'))
        return 200, {'ok': True, 'waypoints': clean}

    def add_here(self, _body, _query):
        return 409, {'ok': False,
                     'error': 'ไม่มีพิกัดจริงในโหมดดูอย่างเดียว ให้พิมพ์ค่าเอง / no real GPS in viewer mode'}

    # ------------------------------------------------------------ snapshot
    def snapshot(self):
        now = time.time()
        telemetry = self.demo.telemetry() if self.demo else {}
        fix = {}
        if self.demo:
            fix = {'lat': self.demo.lat, 'lon': self.demo.lon, 'alt': 12.0, 'status': 0, 'age_s': 0.0}
        return {
            'time': now,
            'telemetry': telemetry,
            'telemetry_age_s': 0.0 if self.demo else None,
            'fix': fix,
            'odom': {},
            'nav': {'state': 'VIEWER', 'index': self.demo.index % max(1, len(self.demo.waypoints)) if self.demo else 0,
                    'total': len(self.demo.waypoints) if self.demo else 0,
                    'waypoints': self.demo.waypoints if self.demo else [],
                    'distance_m': 0.0, 'bearing_deg': 0.0, 'block_reason': ''},
            'safety': {'estop': False, 'link_ok': True, 'esp_ok': False},
            'pose': ({'lat': self.demo.lat, 'lon': self.demo.lon, 'source': 'simulated',
                      'trusted': False, 'heading_deg': self.demo.heading} if self.demo else {}),
            'server': {
                'mode': 'viewer',
                'demo': bool(self.demo),
                'simulation': bool(self.demo),   # buttons drive the simulated robot
                'clients': self.broadcaster.count(),
                'uptime_s': int(now - self.started),
                'allow_config_edit': True,
                'manual_speed_mps': self.manual_speed,
                'manual_turn_dps': self.manual_turn,
                'max_speed_mps': 0.42,
                'config_dir': os.path.abspath(self.args.config_dir),
                'lan_url': urls_for(self.args.host, self.args.port)[1],
                'robot_url': self.args.robot_url or '',
            },
        }

    def log(self, source, message):
        entry = {'t': time.strftime('%H:%M:%S'), 'source': source, 'message': message}
        with self.lock:
            self.logs.append(entry)
        self.broadcaster.broadcast('log', entry)
        print('[%s] %s' % (source, message))

    # ---------------------------------------------------------------- run
    def run(self):
        self.server.start()
        url, lan = urls_for(self.args.host, self.args.port)
        # flush: piped output is block buffered, and the address is the one
        # thing someone needs to see straight away
        print(BANNER, flush=True)
        print('  On this computer : %s' % url, flush=True)
        if lan:
            print('  From a phone     : %s' % lan, flush=True)
        else:
            print('  From a phone     : start with --host 0.0.0.0 to allow it', flush=True)
        print('  Settings folder  : %s' % os.path.abspath(self.args.config_dir), flush=True)
        print('  Press Ctrl+C to stop.\n', flush=True)

        if not self.args.no_browser:
            threading.Timer(0.8, lambda: webbrowser.open(url)).start()

        period = 0.2
        try:
            while True:
                time.sleep(period)
                if self.demo:
                    self.demo.step(period)
                if self.broadcaster.count():
                    self.broadcaster.broadcast('telemetry', self.snapshot())
        except KeyboardInterrupt:
            print('\nstopping...')
        finally:
            self.server.stop()


def main():
    parser = argparse.ArgumentParser(description='GPS_Localize web UI viewer (no ROS, view only)')
    parser.add_argument('--host', default='127.0.0.1',
                        help='0.0.0.0 to let other devices open the demo (default: this PC only)')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--config-dir', default=os.path.join(APP_DIR, 'settings'),
                        help='folder for the editable YAML files')
    parser.add_argument('--robot-url', default='',
                        help='address of the real robot UI, shown as a shortcut, e.g. http://192.168.1.50:8080')
    parser.add_argument('--no-demo', dest='demo', action='store_false',
                        help='do not simulate a moving robot, show empty values')
    parser.add_argument('--no-browser', action='store_true', help='do not open a browser window')
    parser.set_defaults(demo=True)
    args = parser.parse_args()

    os.makedirs(args.config_dir, exist_ok=True)
    Viewer(args).run()


if __name__ == '__main__':
    main()

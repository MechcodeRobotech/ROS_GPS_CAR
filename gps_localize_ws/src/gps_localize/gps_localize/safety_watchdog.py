#!/usr/bin/env python3
"""Safety watchdog - the only node allowed to feed the firmware heartbeat.

    browser --(POST /api/heartbeat)--> web_server --/web/heartbeat--> THIS NODE
                                                                        |
                                          /gps_localize/heartbeat -------+--> ESP32

While every link in that chain is alive the watchdog publishes a heartbeat
counter at safety.heartbeat_rate_hz. The moment the browser goes quiet (Wi-Fi
dropped, tab closed, laptop asleep) the heartbeat stops, and the firmware
brakes on its own after HEARTBEAT_TIMEOUT_MS. Navigation is paused and a zero
command is published as well, so the robot stops even if only the ROS side
survived.

Nothing in here can start the robot again by itself: leaving an emergency stop
needs an explicit RESET from the web UI (safety.auto_clear_estop: false).
"""

import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from gps_localize_msgs.msg import Telemetry
from std_msgs.msg import Bool, Int32, String

from gps_localize import telemetry as tel
from gps_localize.configstore import ConfigStore

NS = '/gps_localize'


def latched_qos(depth=1):
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class SafetyWatchdog(Node):

    def __init__(self):
        super().__init__('safety_watchdog')
        self.declare_parameter('defaults_dir', '')
        self.declare_parameter('runtime_config_dir', '~/.gps_localize')

        defaults = self.get_parameter('defaults_dir').value or _share_config_dir()
        self.store = ConfigStore(defaults,
                                 self.get_parameter('runtime_config_dir').value,
                                 self.get_logger())
        self._load_config()

        # ---------------------------------------------------------- state
        self.counter = 0
        self.last_telemetry = 0.0
        self.last_web_beat = 0.0
        self.web_seen = False
        self.estop_latched = False
        self.link_ok = True
        self.esp_ok = False
        self.firmware_flags = 0
        self.firmware_state = 0
        self._last_report = ''

        # ------------------------------------------------------- interfaces
        self.pub_heartbeat = self.create_publisher(Int32, NS + '/heartbeat', 1)
        self.pub_estop = self.create_publisher(Bool, NS + '/estop', latched_qos())
        self.pub_status = self.create_publisher(String, NS + '/safety', latched_qos())
        self.pub_nav_cmd = self.create_publisher(String, NS + '/nav/cmd', 5)
        self.pub_cmd_move = self.create_publisher(Twist, NS + '/cmd_move', 5)

        self.create_subscription(Telemetry, NS + '/telemetry', self.on_telemetry, 5)
        self.create_subscription(Int32, NS + '/web/heartbeat', self.on_web_beat, 5)
        self.create_subscription(Bool, NS + '/estop_request', self.on_estop_request, 5)
        self.create_subscription(String, NS + '/config/updated', self.on_config_updated, 5)

        period = 1.0 / max(1.0, self.heartbeat_rate)
        self.create_timer(period, self.tick)
        self.create_timer(0.5, self.publish_status)

        self.publish_estop()
        self.get_logger().info(
            'safety watchdog ready: heartbeat %.1f Hz, web heartbeat %s (timeout %.1f s)'
            % (self.heartbeat_rate,
               'required' if self.require_web else 'not required',
               self.web_timeout))

    # ------------------------------------------------------------- config
    def _load_config(self):
        cfg = self.store.load('safety').get('safety', {})
        self.heartbeat_rate = float(cfg.get('heartbeat_rate_hz', 10.0))
        self.esp_timeout = float(cfg.get('esp_telemetry_timeout_s', 1.0))
        self.require_web = bool(cfg.get('require_web_heartbeat', True))
        self.web_timeout = float(cfg.get('web_heartbeat_timeout_s', 1.5))
        self.web_grace = float(cfg.get('web_heartbeat_grace_s', 5.0))
        self.auto_clear = bool(cfg.get('auto_clear_estop', False))
        self.publish_stop_cmd = bool(cfg.get('publish_stop_command', True))

    def on_config_updated(self, msg):
        try:
            group = json.loads(msg.data).get('group')
        except (ValueError, TypeError):
            return
        if group in ('safety', 'all'):
            self._load_config()
            self.get_logger().info('safety config reloaded')

    # --------------------------------------------------------- subscriptions
    def on_telemetry(self, msg):
        self.last_telemetry = self.now()
        data = tel.unpack(msg)
        self.firmware_flags = int(data['stop_flags'])
        self.firmware_state = int(data['state'])

    def on_web_beat(self, _msg):
        self.last_web_beat = self.now()
        if not self.web_seen:
            self.web_seen = True
            self.get_logger().info('web client connected')

    def on_estop_request(self, msg):
        if msg.data:
            if not self.estop_latched:
                self.get_logger().warn('EMERGENCY STOP requested from the web UI')
            self.estop_latched = True
            self.stop_everything('web e-stop')
        else:
            if self.estop_latched:
                self.get_logger().info('e-stop reset requested')
            self.estop_latched = False
        self.publish_estop()

    # ---------------------------------------------------------------- loop
    def tick(self):
        now = self.now()
        self.esp_ok = (now - self.last_telemetry) < self.esp_timeout

        if not self.require_web:
            web_ok = True
        elif not self.web_seen:
            # nothing has ever connected: allow the system to sit idle, the
            # firmware refuses to move without commands anyway
            web_ok = True
        else:
            age = now - self.last_web_beat
            web_ok = age < max(self.web_timeout, 0.2)

        if web_ok != self.link_ok:
            self.link_ok = web_ok
            if web_ok:
                self.get_logger().info('web heartbeat restored - robot stays paused until you press START')
            else:
                self.get_logger().error('web heartbeat lost -> cutting the firmware heartbeat')
                self.stop_everything('web heartbeat lost')

        # The heartbeat tracks the LINK only. An e-stop is signalled on its own
        # topic, so the firmware can tell "operator stopped me" (solid red LED)
        # apart from "I lost the network" (fast blinking red LED).
        if self.link_ok:
            self.counter = (self.counter + 1) % 1000000
            self.pub_heartbeat.publish(Int32(data=self.counter))

    def stop_everything(self, reason):
        if self.publish_stop_cmd:
            self.pub_cmd_move.publish(Twist())
        self.pub_nav_cmd.publish(String(data=json.dumps({'action': 'pause', 'reason': reason})))

    # -------------------------------------------------------------- outputs
    def publish_estop(self):
        self.pub_estop.publish(Bool(data=self.estop_latched))

    def publish_status(self):
        now = self.now()
        status = {
            'estop': self.estop_latched,
            'link_ok': self.link_ok,
            'esp_ok': self.esp_ok,
            'web_seen': self.web_seen,
            'web_age_s': round(now - self.last_web_beat, 2) if self.web_seen else None,
            'telemetry_age_s': round(now - self.last_telemetry, 2) if self.last_telemetry else None,
            'heartbeat': self.counter,
            'firmware_state': tel.STATE_NAMES.get(self.firmware_state, 'UNKNOWN'),
            'firmware_stop_reasons': tel.decode_flags(self.firmware_flags),
            'require_web_heartbeat': self.require_web,
        }
        report = json.dumps(status, sort_keys=True)
        self.pub_status.publish(String(data=report))
        if report != self._last_report:
            self._last_report = report
        self.publish_estop()

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def _share_config_dir():
    from ament_index_python.packages import get_package_share_directory
    import os
    return os.path.join(get_package_share_directory('gps_localize'), 'config')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_everything('shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

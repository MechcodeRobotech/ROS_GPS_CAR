"""Safety regression tests for stationary waiting in automatic GPS navigation."""
import json
from types import SimpleNamespace
from unittest.mock import patch

from test_nav_guards import nav, FakeFix


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Robot(nav.WaypointNav):
    def __init__(self):
        self.clock = 100.0
        self.state = nav.RUNNING
        self.index = 0
        self.waypoints = [{'name': 'north', 'lat': 13.001, 'lon': 100.0},
                          {'name': 'east', 'lat': 13.0, 'lon': 100.001}]
        self.temp_target = None
        self.fix = (13.0, 100.0)
        self.raw_fix = self.fix
        self.fix_source = 'gps'
        self.fix_time = self.gps_fix_time = self.fused_time = self.clock
        self.fix_timeout = 3.0
        self.fix_quality = 1
        self.satellites = 8
        self.min_satellites = 6
        self.hdop = 1.0
        self.max_hdop = 3.0
        self.heading_deg = 0.0
        self.heading_ref = 1
        self.speed_mps = 0.0
        self.telemetry_time = self.clock
        self.telemetry_timeout = 1.0
        self.require_fix = False
        self.allow_dead_reckoning = True
        self.dead_reckon = {'lat': 13.0, 'lon': 100.0}
        self.blocked_since = None
        self.blocked_resume_state = nav.RUNNING
        self.block_reason = ''
        self.estop = False
        self.leg_start_time = self.clock
        self.leg_start_distance = 0.0
        self.waypoint_timeout = 180.0
        self.last_distance = 0.0
        self.last_bearing = 0.0
        self.arrive_radius = 2.5
        self.arrive_hold_s = 1.5
        self.arrive_sigma_k = 1.5
        self.fix_sigma = None
        self.inside_since = None
        self.cruise_speed = 0.25
        self.approach_speed = 0.12
        self.approach_distance = 1.5
        self.heading_tolerance = 15.0
        self.align_target = None
        self.align_start = 0.0
        self.align_timeout = 20.0
        self.align_why = ''
        self.loop_waypoints = False
        self._stop_sent = False
        self.pub_cmd = Publisher()
        self.logs = []

    def now(self):
        return self.clock

    def log(self, message):
        self.logs.append(message)

    def refresh(self):
        self.fix_time = self.gps_fix_time = self.telemetry_time = self.clock

    def command(self, action, **fields):
        self.on_cmd(SimpleNamespace(data=json.dumps(dict(action=action, **fields))))


def stopped(robot):
    cmd = robot.pub_cmd.messages[-1]
    assert cmd.linear.x == 0.0 and cmd.linear.y == 0.0


def moving(robot):
    return robot.pub_cmd.messages[-1].linear.x > 0.0


@patch.object(nav, 'Twist', Twist)
def test_missing_data_stops_repeatedly_and_never_discovers_by_driving():
    for field, value in [('heading_ref', 0), ('telemetry_time', None),
                         ('gps_fix_time', 0.0), ('fix_source', 'dead_reckoning'),
                         ('fix_quality', 0), ('satellites', 3), ('hdop', float('nan')),
                         ('fix', (float('nan'), 100.0))]:
        robot = Robot()
        setattr(robot, field, value)
        robot.tick()
        assert robot.state == nav.BLOCKED, field
        stopped(robot)
        robot.clock += 0.2
        robot.tick()
        stopped(robot)
        assert len(robot.pub_cmd.messages) == 2
    robot = Robot()
    robot.state = nav.DISCOVER
    robot.heading_ref = 0
    robot.tick()
    assert robot.state == nav.BLOCKED
    stopped(robot)


@patch.object(nav, 'Twist', Twist)
def test_lost_data_mid_motion_stops_and_recovery_recomputes_new_target():
    robot = Robot()
    robot.tick()
    assert moving(robot)
    assert abs(robot.last_bearing) < 1.0
    robot.clock += 1.1
    robot.tick()
    assert robot.state == nav.BLOCKED
    stopped(robot)
    robot.command('goto', index=1)
    robot.tick()  # the new target is also blocked while telemetry is stale
    assert robot.state == nav.BLOCKED
    robot.clock += 500
    robot.refresh()
    robot.tick()
    assert robot.state == nav.RUNNING
    assert moving(robot)
    assert 80 < robot.last_bearing < 100


@patch.object(nav, 'Twist', Twist)
def test_wait_does_not_consume_waypoint_timeout():
    robot = Robot()
    robot.heading_ref = 0
    robot.clock += 10
    robot.refresh()
    robot.tick()
    robot.clock += 400
    robot.heading_ref = 1
    robot.refresh()
    robot.tick()
    assert robot.state == nav.RUNNING
    assert robot.clock - robot.leg_start_time == 10
    assert moving(robot)


@patch.object(nav, 'Twist', Twist)
def test_invalid_target_and_invalid_raw_fix_cannot_be_hidden_by_fused_pose():
    robot = Robot()
    robot.waypoints[0]['lat'] = float('nan')
    robot.tick()
    assert robot.state == nav.BLOCKED
    stopped(robot)
    robot = Robot()
    robot.on_fix(FakeFix(13.0, 100.0, status=-1))
    robot.tick()
    assert robot.state == nav.BLOCKED
    stopped(robot)


@patch.object(nav, 'Twist', Twist)
def test_align_is_stopped_on_loss_and_resumes_with_its_timeout_paused():
    robot = Robot()
    robot.state = nav.ALIGN
    robot.align_target = 90.0
    robot.align_start = robot.clock
    robot.tick()
    assert robot.pub_cmd.messages[-1].linear.y == 1.0
    robot.clock += 2
    robot.tick()
    assert robot.state == nav.BLOCKED
    stopped(robot)
    robot.clock += 100
    robot.refresh()
    robot.tick()
    assert robot.state == nav.ALIGN
    assert robot.pub_cmd.messages[-1].linear.y == 1.0
    assert robot.clock - robot.align_start == 2


@patch.object(nav, 'Twist', Twist)
def test_pause_stop_estop_and_idle_do_not_automatically_resume():
    for action in ('pause', 'stop'):
        robot = Robot()
        robot.heading_ref = 0
        robot.tick()
        robot.command(action)
        assert robot.state == (nav.PAUSED if action == 'pause' else nav.IDLE)
        stopped(robot)
        robot.heading_ref = 1
        robot.refresh()
        robot.tick()
        assert robot.state in (nav.PAUSED, nav.IDLE)
        count = len(robot.pub_cmd.messages)
        robot.tick()
        assert len(robot.pub_cmd.messages) == count
    robot = Robot()
    robot.heading_ref = 0
    robot.tick()
    robot.on_estop(SimpleNamespace(data=True))
    assert robot.state == nav.PAUSED
    robot.heading_ref = 1
    robot.refresh()
    robot.tick()
    assert robot.state == nav.PAUSED
    stopped(robot)


@patch.object(nav, 'Twist', Twist)
def test_missing_stale_and_nonfinite_telemetry_refuse_motion():
    for value in (None, 98.9):
        robot = Robot()
        robot.telemetry_time = value
        robot.tick()
        assert robot.state == nav.BLOCKED
        stopped(robot)
    robot = Robot()
    sample = {'heading_deg': float('nan'), 'speed_mps': 0.0, 'gps_fix_quality': 1,
              'gps_satellites': 8, 'gps_hdop': 1.0, 'heading_ref': 1}
    with patch.object(nav.tel, 'unpack', return_value=sample):
        robot.on_telemetry(object())
    robot.tick()
    assert robot.state == nav.BLOCKED
    stopped(robot)


@patch.object(nav, 'Twist', Twist)
def test_estop_blocks_motion_even_if_an_active_state_was_requested():
    robot = Robot()
    robot.estop = True
    robot.tick()
    assert robot.state == nav.PAUSED
    stopped(robot)


@patch.object(nav, 'Twist', Twist)
def test_fresh_fused_pose_cannot_mask_old_raw_gps_and_recovers_on_live_fix():
    robot = Robot()
    robot.clock += 4
    robot.fix_time = robot.telemetry_time = robot.clock
    robot.tick()
    assert robot.state == nav.BLOCKED
    stopped(robot)
    robot.on_fix(FakeFix(13.0, 100.0))
    robot.tick()
    assert robot.state == nav.RUNNING
    assert moving(robot)


@patch.object(nav, 'Twist', Twist)
def test_start_and_resume_wait_for_heading_without_motion():
    for action, initial in [('start', nav.IDLE), ('resume', nav.PAUSED)]:
        robot = Robot()
        robot.state = initial
        robot.heading_ref = 0
        robot.command(action)
        robot.tick()
        assert robot.state == nav.BLOCKED
        stopped(robot)
        robot.heading_ref = 1
        robot.refresh()
        robot.tick()
        assert robot.state == nav.RUNNING
        assert moving(robot)


@patch.object(nav, 'Twist', Twist)
def test_legacy_settings_cannot_allow_dead_reckoning_or_unchecked_gps():
    robot = Robot()
    robot.require_fix = False
    robot.allow_dead_reckoning = True
    robot.fix_source = 'dead_reckoning'
    robot.tick()
    assert robot.state == nav.BLOCKED
    stopped(robot)


@patch.object(nav, 'Twist', Twist)
def test_recovery_does_not_inherit_arrival_dwell():
    robot = Robot()
    robot.inside_since = 90.0
    robot.heading_ref = 0
    robot.tick()
    assert robot.inside_since is None
    stopped(robot)

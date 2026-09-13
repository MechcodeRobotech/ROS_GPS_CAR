# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""The guards around navigation state and around commanding motion.

Every test here stands for a defect that a three-way code audit found in code
that was already running on the robot, and that the existing tests did not
catch. They are grouped by the question they answer:

  does the state machine always leave the state it is in?
  does the steering position come from the filter, or from the raw fix?
  does a motion endpoint ask the safety gate before it publishes?

The ROS packages are stubbed the same way `test_arrival.py` stubs them, because
the logic under test is ordinary Python that does not need a running graph.
"""
import math
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class StubModule(types.ModuleType):
    """Any attribute asked of it becomes a throwaway class."""

    def __getattr__(self, name):
        cls = type(name, (), {'__init__': lambda self, *a, **k: None})
        setattr(self, name, cls)
        return cls


def _stub_ros():
    for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.duration',
                 'rclpy.executors', 'rclpy.serialization',
                 'std_msgs', 'std_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg',
                 'sensor_msgs', 'sensor_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
                 'gps_localize_msgs', 'gps_localize_msgs.msg'):
        if name not in sys.modules:
            sys.modules[name] = StubModule(name)


_stub_ros()
from gps_localize import waypoint_nav as nav          # noqa: E402

# The real value from sensor_msgs/NavSatStatus. The stub manufactures classes
# on demand but has no class attributes of its own, and on_fix compares against
# this one - so it is stated here rather than guessed at.
nav.NavSatStatus.STATUS_NO_FIX = -1


class FakeNav(object):
    """A WaypointNav with the real methods bound and nothing else running.

    Only the attributes the methods under test actually touch are present, so a
    method that starts depending on something new fails loudly here rather than
    passing on a stub that quietly answers every question.
    """

    def __init__(self, waypoints=None, loop=False):
        self.state = nav.ALIGN
        self.index = 0
        self.waypoints = waypoints if waypoints is not None else []
        self.loop_waypoints = loop
        self.temp_target = None
        self.align_target = 90.0
        self.align_why = 'reached'
        self.last_distance = 0.0
        self.leg_start_time = 0.0
        self.inside_since = None
        self.clock = 100.0
        self.logs = []
        self.stops = 0
        self._stop_sent = False

        # position plumbing
        self.fix = None
        self.raw_fix = None
        self.fix_time = 0.0
        self.fused_time = 0.0
        self.gps_fix_time = 0.0
        self.fix_source = 'none'
        self.fix_sigma = None
        self.fix_timeout = 3.0
        self.dead_reckon = None

        # discovery plumbing
        self.discover_enabled = True
        self.heading_ref = 0
        self._fix_usable = (True, '')

    def now(self):
        return self.clock

    def log(self, msg):
        self.logs.append(msg)

    def publish_stop(self):
        self.stops += 1

    def fix_usable(self):
        return self._fix_usable

    # the real implementations under test
    advance = nav.WaypointNav.advance
    set_state = nav.WaypointNav.set_state
    start_leg = nav.WaypointNav.start_leg
    on_fix = nav.WaypointNav.on_fix
    needs_discovery = nav.WaypointNav.needs_discovery


class FakeFix(object):
    """Just enough of sensor_msgs/NavSatFix for on_fix()."""

    class _Status(object):
        def __init__(self, status):
            self.status = status

    def __init__(self, lat, lon, status=0):
        self.latitude = lat
        self.longitude = lon
        self.status = self._Status(status)


class TestAlignAlwaysExits(unittest.TestCase):
    """advance() cleared the alignment target and left the state on ALIGN.

    The next tick then dispatched tick_align() with no target and raised
    `TypeError: unsupported operand type(s) for -: 'NoneType' and 'float'`
    inside the timer callback of the node that drives the robot. It needed a
    waypoint carrying a heading AND another waypoint after it, which is exactly
    what the single-waypoint tests never built.
    """

    def test_intermediate_waypoint_leaves_align(self):
        fake = FakeNav(waypoints=[{'name': 'WP1', 'lat': 1.0, 'lon': 2.0, 'heading_deg': 90.0},
                                  {'name': 'WP2', 'lat': 1.1, 'lon': 2.1}])
        fake.advance('reached')
        self.assertEqual(fake.index, 1, 'it should move on to the next waypoint')
        self.assertNotEqual(fake.state, nav.ALIGN,
                            'staying in ALIGN with align_target cleared is the crash')
        self.assertEqual(fake.state, nav.RUNNING)
        self.assertIsNone(fake.align_target)

    def test_looping_leaves_align(self):
        fake = FakeNav(waypoints=[{'name': 'WP1', 'lat': 1.0, 'lon': 2.0, 'heading_deg': 90.0},
                                  {'name': 'WP2', 'lat': 1.1, 'lon': 2.1}],
                       loop=True)
        fake.index = 1                       # the last one, so the loop branch runs
        fake.advance('reached')
        self.assertEqual(fake.index, 0)
        self.assertNotEqual(fake.state, nav.ALIGN)

    def test_the_last_waypoint_still_finishes(self):
        """The branch that already worked must keep working."""
        fake = FakeNav(waypoints=[{'name': 'WP1', 'lat': 1.0, 'lon': 2.0, 'heading_deg': 90.0}])
        fake.advance('reached')
        self.assertEqual(fake.state, nav.FINISHED)
        self.assertEqual(fake.stops, 1, 'finishing must still publish a stop')

    def test_a_picked_point_still_finishes(self):
        fake = FakeNav()
        fake.temp_target = {'name': 'picked', 'lat': 1.0, 'lon': 2.0}
        fake.advance('reached')
        self.assertEqual(fake.state, nav.FINISHED)
        self.assertIsNone(fake.temp_target)


class TestSteeringPositionComesFromTheFilter(unittest.TestCase):
    """Both callbacks wrote self.fix, so the robot steered on whichever landed last.

    The raw fix was measured moving 17.6 m between consecutive samples with the
    robot standing still. Alternating between that and the fused estimate is
    the jitter the fusion was built to remove.
    """

    def test_a_raw_fix_does_not_replace_a_fresh_fused_position(self):
        fake = FakeNav()
        fake.fix = (10.0, 20.0)              # fused
        fake.fused_time = 100.0              # just now
        fake.clock = 100.5
        fake.on_fix(FakeFix(11.0, 21.0))
        self.assertEqual(fake.fix, (10.0, 20.0), 'the fused position must survive')
        self.assertEqual(fake.raw_fix, (11.0, 21.0), 'the raw fix is still kept')
        self.assertEqual(fake.gps_fix_time, 100.5, 'freshness still comes from the raw fix')

    def test_a_raw_fix_takes_over_when_the_fused_stream_stops(self):
        fake = FakeNav()
        fake.fix = (10.0, 20.0)
        fake.fused_time = 100.0
        fake.clock = 100.0 + 3.5             # older than fix_timeout of 3 s
        fake.on_fix(FakeFix(11.0, 21.0))
        self.assertEqual(fake.fix, (11.0, 21.0),
                         'with no fused pose the raw fix is the only position there is')

    def test_a_raw_fix_is_used_before_any_fused_pose_arrives(self):
        fake = FakeNav()
        fake.on_fix(FakeFix(11.0, 21.0))
        self.assertEqual(fake.fix, (11.0, 21.0))

    def test_a_fix_with_no_lock_is_ignored(self):
        fake = FakeNav()
        fake.on_fix(FakeFix(11.0, 21.0, status=-1))   # STATUS_NO_FIX
        self.assertIsNone(fake.fix)

    def test_a_non_finite_fix_is_ignored(self):
        fake = FakeNav()
        fake.on_fix(FakeFix(float('nan'), 21.0))
        self.assertIsNone(fake.fix)


class TestDiscoveryNeedsAUsableFix(unittest.TestCase):
    """Discovery drives the robot forward to learn a heading from the GPS course.

    With a stale or rejected fix there is no course to learn and no position to
    watch, so the robot drove blind for the whole discover window. tick()
    dispatches DISCOVER before its own fix gate, so the check has to live here.
    """

    def test_a_usable_fix_allows_discovery(self):
        fake = FakeNav()
        fake.fix = (1.0, 2.0)
        fake._fix_usable = (True, '')
        self.assertTrue(fake.needs_discovery())

    def test_a_stale_fix_refuses_discovery(self):
        fake = FakeNav()
        fake.fix = (1.0, 2.0)
        fake._fix_usable = (False, 'GPS fix is 100.0 s old')
        self.assertFalse(fake.needs_discovery(),
                         'driving forward to learn a heading needs a position to learn it from')

    def test_an_absolute_heading_already_present_refuses_discovery(self):
        fake = FakeNav()
        fake.fix = (1.0, 2.0)
        fake.heading_ref = 1                 # compass
        self.assertFalse(fake.needs_discovery())


if __name__ == '__main__':
    unittest.main()

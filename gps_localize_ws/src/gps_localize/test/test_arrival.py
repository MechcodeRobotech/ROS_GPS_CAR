"""Arrival must survive the receiver's own noise.

The robot drove to a waypoint twice and never declared it reached. The cause was
not the driving: `arrive_radius_m` was 1.0 m, and this receiver cannot resolve a
metre. Measured on the real robot, stationary outdoors on 8 satellites at HDOP
1.3, the fix scattered 1.56 m median and 4.86 m worst - so whether a sample ever
landed inside a 1 m circle was luck.

Widening the circle on its own trades one failure for another: with 5 m of
scatter a single sample can fall inside a 2.5 m circle while the robot is really
4 m away, and the mission advances early. Both halves are needed, and these tests
hold both of them:

  the floor   no configuration can claim an arrival tighter than the receiver
  the dwell   a single lucky sample is not an arrival; staying is

These run without ROS. `waypoint_nav` imports rclpy at module scope, so the
arrival logic is exercised through a small stand-in that carries the same fields
and calls the same two methods off the real class.
"""
import os
import sys
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def _load_nav_module():
    """Import waypoint_nav with the ROS packages stubbed out.

    Only the names touched at import time need to exist. Anything the tests
    actually call is real code from the module.
    """
    class StubModule(types.ModuleType):
        """Any attribute asked of it becomes a throwaway class.

        Naming each ROS message by hand only guarantees the test breaks the next
        time an import is added to `waypoint_nav`, which says nothing about
        arrival. The names are not used here - only the import has to succeed.
        """

        def __getattr__(self, name):
            cls = type(name, (), {'__init__': lambda self, *a, **k: None})
            setattr(self, name, cls)
            return cls

    for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'rclpy.duration',
                 'std_msgs', 'std_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg',
                 'sensor_msgs', 'sensor_msgs.msg',
                 'gps_localize_msgs', 'gps_localize_msgs.msg'):
        if name not in sys.modules:
            sys.modules[name] = StubModule(name)

    from gps_localize import waypoint_nav
    return waypoint_nav


nav_mod = _load_nav_module()


class FakeNav(object):
    """The arrival state of a WaypointNav, with a clock the test controls."""

    def __init__(self, radius=2.5, hold=1.5, sigma_k=1.5, sigma=None):
        self.arrive_radius = radius
        self.arrive_hold_s = hold
        self.arrive_sigma_k = sigma_k
        self.fix_sigma = sigma
        self.inside_since = None
        self.clock = 0.0
        self.arrived_at = None

    def now(self):
        return self.clock

    effective_arrive_radius = nav_mod.WaypointNav.effective_arrive_radius

    def step(self, distance, dt=0.2):
        """One navigation tick at `distance` metres from the target.

        Mirrors the arrival branch in `WaypointNav.navigate` exactly: inside the
        radius starts or continues the dwell, outside it clears the dwell.
        """
        self.clock += dt
        radius = self.effective_arrive_radius()
        if distance <= radius:
            if self.inside_since is None:
                self.inside_since = self.now()
            if self.now() - self.inside_since >= self.arrive_hold_s:
                self.arrived_at = distance
                return True
        else:
            self.inside_since = None
        return False


class TestArriveRadius(unittest.TestCase):

    def test_config_cannot_ask_for_a_radius_the_receiver_cannot_resolve(self):
        """The 1.0 m that shipped, and anything under it, is raised to the floor."""
        for asked in (0.1, 0.6, 1.0, 1.9):
            nav = FakeNav(radius=asked)
            self.assertEqual(nav.effective_arrive_radius(), nav_mod.ARRIVE_RADIUS_FLOOR_M,
                             'a %.1f m radius must be raised to the floor' % asked)

    def test_a_radius_above_the_floor_is_left_alone(self):
        self.assertAlmostEqual(FakeNav(radius=5.0).effective_arrive_radius(), 5.0)

    def test_radius_grows_with_the_fusion_uncertainty(self):
        """Under cover the position is worse, so the circle has to be bigger."""
        clean = FakeNav(radius=2.5, sigma=1.5)
        self.assertAlmostEqual(clean.effective_arrive_radius(), 2.5,
                               msg='a good fix must not inflate the radius')

        degraded = FakeNav(radius=2.5, sigma=6.0)
        self.assertAlmostEqual(degraded.effective_arrive_radius(), 9.0)

    def test_uncertainty_never_shrinks_the_radius_below_the_floor(self):
        nav = FakeNav(radius=2.5, sigma=0.01)
        self.assertEqual(nav.effective_arrive_radius(), 2.5)


class TestArriveDwell(unittest.TestCase):

    def test_one_lucky_sample_inside_the_circle_is_not_an_arrival(self):
        """The failure widening the radius would otherwise introduce.

        The robot is 4 m away and not moving. A single noisy sample puts it at
        2 m, then the fix returns to the truth. The mission must not advance.
        """
        nav = FakeNav()
        self.assertFalse(nav.step(4.0))
        self.assertFalse(nav.step(2.0))          # the lucky one
        self.assertFalse(nav.step(4.1))
        self.assertIsNone(nav.inside_since, 'one sample outside must clear the dwell')
        for _ in range(30):
            self.assertFalse(nav.step(4.0))
        self.assertIsNone(nav.arrived_at)

    def test_a_robot_that_stays_inside_does_arrive(self):
        nav = FakeNav()
        for _ in range(5):
            self.assertFalse(nav.step(6.0))
        arrived = False
        for _ in range(20):
            if nav.step(1.8):
                arrived = True
                break
        self.assertTrue(arrived, 'staying inside the circle must eventually count')
        self.assertGreaterEqual(nav.clock, nav.arrive_hold_s)

    def test_arrival_is_not_declared_before_the_hold_elapses(self):
        nav = FakeNav(hold=1.5)
        self.assertFalse(nav.step(1.0, dt=0.2))
        self.assertFalse(nav.step(1.0, dt=0.2))   # 0.2 s inside
        self.assertFalse(nav.step(1.0, dt=1.0))   # 1.2 s inside, still short
        self.assertTrue(nav.step(1.0, dt=0.5))    # 1.7 s inside

    def test_real_scatter_around_a_point_the_robot_is_standing_on(self):
        """The measured case: the robot IS at the point, the fix wanders.

        These are the distances a 1.56 m median / 4.86 m worst scatter produces
        around a target the robot is sitting on. With the old 1 m radius almost
        none of them counted; the mission must finish now.
        """
        scatter = [1.2, 3.4, 0.8, 2.1, 4.8, 1.9, 0.6, 2.4, 1.1, 3.0,
                   1.5, 0.9, 2.2, 1.7, 1.3, 2.0, 0.7, 1.4, 1.8, 1.0]
        nav = FakeNav(sigma=1.6)
        arrived = any(nav.step(d) for d in scatter)
        self.assertTrue(arrived, 'a robot standing on the point must finish')

    def test_driving_past_the_point_does_not_count_as_arriving(self):
        """Overshoot: inside briefly, then gone. Not an arrival."""
        nav = FakeNav()
        for d in (5.0, 3.5, 2.0, 1.0, 2.0, 3.5, 5.0):
            self.assertFalse(nav.step(d, dt=0.4))
        self.assertIsNone(nav.arrived_at)


class TestWaypointHeading(unittest.TestCase):
    """A waypoint is a place and, optionally, a direction to be facing there."""

    def test_no_heading_is_the_normal_case(self):
        for value in (None, '', 'north', float('nan'), float('inf')):
            self.assertIsNone(nav_mod.parse_heading(value),
                              '%r must mean "any direction"' % (value,))

    def test_zero_is_a_heading_not_an_absence(self):
        """North is a real answer, and `if not heading` would throw it away."""
        self.assertEqual(nav_mod.parse_heading(0), 0.0)
        self.assertEqual(nav_mod.parse_heading('0'), 0.0)

    def test_a_heading_typed_from_a_compass_app_is_accepted(self):
        self.assertEqual(nav_mod.parse_heading(90), 90.0)
        self.assertEqual(nav_mod.parse_heading('187.5'), 187.5)

    def test_out_of_range_wraps_rather_than_being_rejected(self):
        """-90 and 270 are the same direction; neither should be thrown away."""
        self.assertEqual(nav_mod.parse_heading(-90), 270.0)
        self.assertEqual(nav_mod.parse_heading(450), 90.0)
        self.assertEqual(nav_mod.parse_heading(360), 0.0)


if __name__ == '__main__':
    unittest.main()

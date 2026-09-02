"""What someone typed, and what it should be taken to mean.

Pure string handling, so it is fully testable with no robot attached - which is
the point of writing it now. Every case here is a form someone actually types:
copied from a shell prompt, read off a router, pasted from a browser bar.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from gps_localize.connect_target import Target, candidates, parse


class TestSerial(unittest.TestCase):

    def test_linux_and_mac_ports(self):
        for text in ('/dev/ttyUSB0', '/dev/ttyACM1', '/dev/cu.usbserial-1420'):
            self.assertEqual(parse(text).kind, Target.SERIAL, text)

    def test_windows_ports(self):
        # COM7 looks enough like a hostname to be mistaken for one.
        for text in ('COM1', 'COM7', 'com12'):  # qc: allow-windows-path
            self.assertEqual(parse(text).kind, Target.SERIAL, text)

    def test_a_serial_target_has_no_url(self):
        self.assertIsNone(parse('/dev/ttyUSB0').url())


class TestAddresses(unittest.TestCase):

    def test_plain_ip(self):
        t = parse('192.168.1.50')
        self.assertEqual(t.kind, Target.IP)
        self.assertEqual(t.value, '192.168.1.50')
        self.assertIsNone(t.port)

    def test_ip_with_port(self):
        t = parse('192.168.1.50:8080')
        self.assertEqual((t.kind, t.value, t.port), (Target.IP, '192.168.1.50', 8080))

    def test_octets_above_255_are_not_an_ip(self):
        # 999.1.1.1 matches the shape but is not an address. Treating it as a
        # hostname is right - it might resolve - but calling it an IP would be
        # a lie the UI would repeat.
        self.assertNotEqual(parse('999.1.1.1').kind, Target.IP)

    def test_url_from_a_browser_bar(self):
        t = parse('http://192.168.1.50:8080/')
        self.assertEqual((t.kind, t.value, t.port), (Target.IP, '192.168.1.50', 8080))
        self.assertIn('ignored the http:// prefix', t.note)

    def test_url_with_a_path_keeps_only_the_host(self):
        t = parse('http://gps-robot-web.local/settings')
        self.assertEqual(t.value, 'gps-robot-web.local')


class TestNames(unittest.TestCase):

    def test_bare_name_tries_mdns_first(self):
        t = parse('module')
        self.assertEqual(t.kind, Target.HOSTNAME)
        self.assertEqual(candidates(t), ['module.local', 'module'])

    def test_mdns_name(self):
        t = parse('module.local')
        self.assertEqual(t.kind, Target.MDNS)
        self.assertEqual(candidates(t), ['module.local'])
        self.assertIn('some networks block', t.note)

    def test_user_at_host_from_a_shell_prompt(self):
        # The exact thing someone copies out of "mannaja@mannaja:~$".
        t = parse('module@module.local')
        self.assertEqual(t.kind, Target.MDNS)
        self.assertEqual(t.value, 'module.local')
        self.assertEqual(t.user, 'module')
        self.assertIn('username', t.note)

    def test_ssh_url_with_user_and_port(self):
        t = parse('ssh://pi@robot.local:22')
        self.assertEqual((t.kind, t.value, t.port, t.user),
                         (Target.MDNS, 'robot.local', 22, 'pi'))

    def test_names_are_lowercased(self):
        self.assertEqual(parse('Module.Local').value, 'module.local')


class TestRejections(unittest.TestCase):

    def test_empty(self):
        for text in ('', '   ', None):
            self.assertFalse(parse(text).ok)

    def test_underscore_is_named_as_the_problem(self):
        # This is the exact mistake the robot's own naming avoids: avahi
        # publishes an underscore happily and then no browser resolves it. The
        # message has to say so, or the person retypes the same thing.
        t = parse('gps_robot_web.local')
        self.assertFalse(t.ok)
        self.assertIn('underscore', t.note)
        self.assertIn('gps-robot-web.local', t.note)

    def test_spaces_are_not_a_hostname(self):
        self.assertFalse(parse('my robot').ok)


class TestUrls(unittest.TestCase):

    def test_port_80_is_left_off(self):
        # The whole reason for binding 80: an address with no port is something
        # you can tell someone.
        self.assertEqual(parse('gps-robot-web.local').url(80),
                         'http://gps-robot-web.local')

    def test_other_ports_are_shown(self):
        self.assertEqual(parse('gps-robot-web.local').url(8080),
                         'http://gps-robot-web.local:8080')

    def test_an_explicit_port_wins_over_the_default(self):
        self.assertEqual(parse('robot.local:9090').url(80),
                         'http://robot.local:9090')


if __name__ == '__main__':
    unittest.main(verbosity=2)

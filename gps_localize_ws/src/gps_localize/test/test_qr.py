"""Prove the QR encoder produces codes that actually scan.

A QR code cannot be reviewed by looking at it. Both bugs found while writing
this encoder - a Reed-Solomon generator that was not monic, and format
information written with its two copies transposed - produced codes that were
indistinguishable from correct ones by eye, and that no scanner could read. The
failure would have landed on whoever was standing next to the robot with a phone
that would not connect, with nothing on screen to explain why.

So the test decodes. It renders each code to a PNG and reads it back with
zbarimg, which is a real scanner implementation rather than our own assumptions
run backwards. Round-tripping through our own decoder would only prove we are
consistently wrong.

zbarimg is a test-only dependency and is not needed to RUN anything: the encoder
itself has no dependencies at all, which is the whole reason it is written out
rather than pip-installed. When zbarimg is missing the decode tests skip rather
than fail, the same way the live-link check skips with no board attached - but
the structural tests below still run everywhere.

    sudo apt install zbar-tools        # to enable the decode tests
"""

import os
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from gps_localize import qr


def _have(tool):
    return subprocess.run(['which', tool], capture_output=True).returncode == 0


HAVE_ZBAR = _have('zbarimg')
try:
    from PIL import Image
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False


def _decode(text, scale=8, quiet=4):
    """Render `text` as a QR, then read it back with a real scanner."""
    matrix = qr.encode(text)
    n = len(matrix) + quiet * 2
    img = Image.new('L', (n * scale, n * scale), 255)
    px = img.load()
    for r, row in enumerate(matrix):
        for c, value in enumerate(row):
            if value:
                for dy in range(scale):
                    for dx in range(scale):
                        px[(c + quiet) * scale + dx, (r + quiet) * scale + dy] = 0
    tmp = tempfile.NamedTemporaryFile(suffix='.png', delete=False)
    try:
        img.save(tmp.name)
        out = subprocess.run(['zbarimg', '--quiet', '--raw', tmp.name],
                             capture_output=True, text=True)
        return out.stdout.strip()
    finally:
        os.unlink(tmp.name)


class TestStructure(unittest.TestCase):
    """Checks that need no scanner, so they run on any machine."""

    def test_generator_polynomial_is_monic(self):
        # The division in _rs_remainder reads only poly[1:] and assumes poly[0]
        # is 1. A generator that is a scalar multiple of the right one passes
        # every eyeball check and fails every scanner.
        for degree in (10, 16, 18, 22, 24, 26):
            self.assertEqual(qr._rs_generator(degree)[0], 1)
        self.assertEqual(qr._rs_generator(2), [1, 3, 2])

    def test_data_capacity_matches_the_spec(self):
        # Free modules must equal the codeword bits plus the spec's remainder
        # bits: 7 for versions 2-6, none for 1 and 7-13.
        remainder = {1: 0, 2: 7, 3: 7, 4: 7, 5: 7, 6: 7, 7: 0, 8: 0, 9: 0, 10: 0}
        total = {1: 26, 2: 44, 3: 70, 4: 100, 5: 134,
                 6: 172, 7: 196, 8: 242, 9: 292, 10: 346}
        for version in sorted(qr._SPEC):
            reserved = qr._function_mask(version)
            size = len(reserved)
            free = sum(1 for r in range(size) for c in range(size)
                       if not reserved[r][c])
            self.assertEqual(free, total[version] * 8 + remainder[version],
                             'version %d' % version)

    def test_version_grows_with_length(self):
        self.assertLess(len(qr.encode('short')), len(qr.encode('x' * 150)))

    def test_too_long_is_refused_not_truncated(self):
        # A truncated URL still scans and sends someone somewhere useless.
        with self.assertRaises(ValueError):
            qr.encode('x' * 400)

    def test_renders_without_a_scanner(self):
        svg = qr.to_svg('http://gps-robot-web.local:8080')
        self.assertTrue(svg.startswith('<svg'))
        self.assertIn('</svg>', svg)
        self.assertIn('viewBox', svg)
        art = qr.to_ascii('http://gps-robot-web.local:8080')
        self.assertGreater(len(art.splitlines()), 10)


@unittest.skipUnless(HAVE_ZBAR and HAVE_PIL,
                     'needs zbar-tools and pillow: sudo apt install zbar-tools')
class TestActuallyScans(unittest.TestCase):
    """The tests that matter: a real scanner reads back what we encoded."""

    def test_lan_urls_round_trip(self):
        for url in ('http://gps-robot-web.local:8080',
                    'http://mannaja.local:8080/',
                    'http://192.168.1.55:8080',
                    'http://10.126.95.220:8080',
                    'http://gps-robot-web.local:8080/?tab=drive'):
            self.assertEqual(_decode(url), url)

    def test_every_supported_version_round_trips(self):
        # One payload per version, so a table error in any single version is
        # caught rather than hidden behind whichever one the URL happens to use.
        seen = set()
        for length in range(10, 200, 7):
            payload = 'A' * length
            version = qr._pick_version(length)
            if version in seen:
                continue
            seen.add(version)
            self.assertEqual(_decode(payload), payload,
                             'version %d failed' % version)
        self.assertGreaterEqual(len(seen), 6)

    def test_utf8_payload_round_trips(self):
        text = 'http://gps-robot-web.local:8080/?q=หุ่นยนต์'
        self.assertEqual(_decode(text), text)


if __name__ == '__main__':
    unittest.main(verbosity=2)

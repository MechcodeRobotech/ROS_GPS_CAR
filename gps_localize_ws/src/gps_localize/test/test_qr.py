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

THE ENCODER IS NOT THE WHOLE STORY. A correct code can still be PAINTED
unscannably, and that is a separate failure with the same symptom: something
that looks exactly like a QR and reads as nothing. TestTerminalRenderingsScan
below rasterises the terminal forms the way a terminal grid actually paints them
- fixed cell advance, several cell shapes, both themes - and scans those. It
exists because the ascii form, published on /gps_localize/web/connect and
verified once against a light background, turned out to scan on no dark theme at
all: it borrows the terminal's text colour, so a dark theme inverts every
module.

zbarimg is a test-only dependency and is not needed to RUN anything: the encoder
itself has no dependencies at all, which is the whole reason it is written out
rather than pip-installed. When zbarimg is missing the decode tests skip rather
than fail, the same way the live-link check skips with no board attached - but
the structural tests below still run everywhere.

    sudo apt install zbar-tools        # to enable the decode tests
"""

import os
import re
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
    from PIL import Image, ImageDraw, ImageFont
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False

# A font with the half-block glyphs the ascii form is built from. Without one
# there is nothing to rasterise, so those tests skip rather than lie.
MONO_FONTS = (
    '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',
    '/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf',
)
MONO_FONT = next((path for path in MONO_FONTS if os.path.exists(path)), None)

# Character cell shapes, width x height in pixels, spanning the range a terminal
# at a normal font size actually uses. The ascii form packs two modules into one
# cell vertically, so the cell's aspect ratio decides whether its modules come
# out square - which is why this is tested at more than one shape.
CELL_SHAPES = ((9, 19), (8, 17), (10, 22), (7, 15))

LIGHT_THEME, DARK_THEME = 255, 0


def _scan(img):
    """What a real scanner reads out of this image, or '' if it reads nothing."""
    tmp = tempfile.NamedTemporaryFile(suffix='.png', delete=False)
    try:
        img.save(tmp.name)
        out = subprocess.run(['zbarimg', '--quiet', '--raw', tmp.name],
                             capture_output=True, text=True)
        return out.stdout.strip()
    finally:
        os.unlink(tmp.name)


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
    return _scan(img)


def _paint_text(art, cell, theme_bg):
    """Draw plain text the way a terminal does: one fixed-size cell per glyph.

    Not the same as asking PIL to render the string - a terminal advances by a
    fixed cell whatever the glyph's own width is, and the block characters this
    art is made of are exactly where that difference shows up.
    """
    cell_w, cell_h = cell
    lines = art.split('\n')
    cols = max(len(line) for line in lines)
    fg = DARK_THEME if theme_bg == LIGHT_THEME else LIGHT_THEME
    img = Image.new('L', (cols * cell_w, len(lines) * cell_h), theme_bg)
    draw = ImageDraw.Draw(img)
    font = ImageFont.truetype(MONO_FONT, cell_h)
    for r, line in enumerate(lines):
        for c, char in enumerate(line):
            if char != ' ':
                draw.text((c * cell_w, r * cell_h), char, font=font, fill=fg)
    return img


def _paint_ansi(art, cell, theme_bg):
    """Draw SGR-coloured output the way a terminal does: a filled cell per module.

    The escape sequences set a background colour, and a terminal paints that as
    the whole cell rather than as a glyph, so this fills cells instead of
    drawing text. Cells the art leaves uncoloured fall back to the theme, which
    is exactly what the terminal would do and is the thing being tested.
    """
    cell_w, cell_h = cell
    colours = {'40': DARK_THEME, '107': LIGHT_THEME, '0': None}
    grid = []
    for line in art.split('\n'):
        row, current = [], None
        for token in re.split(r'(\033\[[0-9;]*m)', line):
            if not token:
                continue
            if token.startswith('\033['):
                current = colours.get(token[2:-1], current)
                continue
            row.extend([current] * len(token))
        grid.append(row)

    cols = max(len(row) for row in grid)
    img = Image.new('L', (cols * cell_w, len(grid) * cell_h), theme_bg)
    px = img.load()
    for r, row in enumerate(grid):
        for c, value in enumerate(row):
            fill = theme_bg if value is None else value
            for dy in range(cell_h):
                for dx in range(cell_w):
                    px[c * cell_w + dx, r * cell_h + dy] = fill
    return img


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


@unittest.skipUnless(HAVE_ZBAR and HAVE_PIL and MONO_FONT,
                     'needs zbar-tools, pillow and a monospace font')
class TestTerminalRenderingsScan(unittest.TestCase):
    """Scan what the TERMINAL paints, not what the encoder produced.

    The encoder was already proven correct when the published QR turned out to
    be unreadable on a dark terminal. Nothing above this class could have caught
    that, because nothing above it looked at a rendering.
    """

    URL = 'http://10.0.2.15'

    def test_ansi_scans_on_a_dark_terminal(self):
        # The whole reason the ansi form exists. This is the form the docs, the
        # message comment and robot_qr.py all point at, and most terminals are
        # dark, so this is the case that decides whether any of that works.
        art = qr.to_ansi(self.URL)
        for cell in CELL_SHAPES:
            self.assertEqual(_scan(_paint_ansi(art, cell, DARK_THEME)), self.URL,
                             'cell %dx%d' % cell)

    def test_ansi_scans_on_a_light_terminal_too(self):
        art = qr.to_ansi(self.URL)
        for cell in CELL_SHAPES:
            self.assertEqual(_scan(_paint_ansi(art, cell, LIGHT_THEME)), self.URL,
                             'cell %dx%d' % cell)

    def test_ansi_survives_a_long_url(self):
        # A longer address moves to a bigger version with smaller modules
        # relative to the cell, which is where a rendering starts to fail first.
        url = 'http://gps-robot-web.local:8080/?tab=drive&from=terminal'
        art = qr.to_ansi(url)
        self.assertEqual(_scan(_paint_ansi(art, (9, 19), DARK_THEME)), url)

    def test_ascii_scans_on_a_light_terminal(self):
        art = qr.to_ascii(self.URL)
        for cell in CELL_SHAPES:
            self.assertEqual(_scan(_paint_text(art, cell, LIGHT_THEME)), self.URL,
                             'cell %dx%d' % cell)

    def test_ascii_is_unreadable_on_a_dark_terminal(self):
        # Asserting the KNOWN BREAKAGE, so it cannot quietly become the thing we
        # recommend again. The ascii form is drawn in the terminal's text
        # colour: on a dark theme every module is inverted and zbar refuses it.
        # If this test ever starts failing, the ascii form has been fixed and
        # the advice in WebConnect.msg, docs/topics.md and robot_qr.py should be
        # revisited rather than this assertion simply relaxed.
        art = qr.to_ascii(self.URL)
        for cell in CELL_SHAPES:
            self.assertEqual(_scan(_paint_text(art, cell, DARK_THEME)), '',
                             'cell %dx%d unexpectedly scanned' % cell)

    def test_both_forms_carry_the_spec_quiet_zone(self):
        # Four modules of light on every side. A terminal offers no margin of
        # its own - the prompt sits directly above whatever is printed.
        ascii_rows = qr.to_ascii(self.URL).split('\n')
        self.assertTrue(all(row.strip() == '' for row in ascii_rows[:2]))
        self.assertTrue(all(row.strip() == '' for row in ascii_rows[-2:]))
        self.assertTrue(ascii_rows[3].startswith('    '))

        modules = len(qr.encode(self.URL))
        ansi_rows = qr.to_ansi(self.URL).split('\n')
        self.assertEqual(len(ansi_rows), modules + 8)


if __name__ == '__main__':
    unittest.main(verbosity=2)

#!/usr/bin/env python3
"""Print the robot's web address as a QR code in the terminal.

Getting a phone onto the robot's page means typing an address, and the address
is the part people get wrong - a hyphen read as an underscore, a digit dropped
from an IP. A QR removes the typing entirely: point the camera at the terminal.

    python3 tools/robot_qr.py                  # ask the running robot
    python3 tools/robot_qr.py --url http://... # encode something specific
    python3 tools/robot_qr.py --svg out.svg    # write an SVG instead

The address comes from the running server when it can be reached, because that
is the only source that knows which ports actually bound - port 80 may have lost
a race with something else, in which case the real address carries :8080 and a
QR promising otherwise would be worse than none. With no server running it falls
back to the mDNS name, and says that it is guessing.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'gps_localize_ws', 'src', 'gps_localize'))

try:
    from gps_localize import qr
except ImportError as exc:                       # pragma: no cover
    raise SystemExit(
        '\ncannot find the QR encoder.\n'
        '  looked in: %s\n  error: %s\n'
        'Run this from inside the project, so gps_localize_ws/ is alongside '
        'tools/.\n' % (sys.path[0], exc))

DEFAULT_NAME = 'gps-robot-web.local'


def ask_server(timeout=3.0):
    """The address the robot itself reports, or None if it is not answering."""
    for base in ('http://127.0.0.1', 'http://127.0.0.1:8080'):
        try:
            raw = urllib.request.urlopen(base + '/api/connect', timeout=timeout).read()
            info = json.loads(raw)['connect']
            # qr_url is the IP form. A .local name needs multicast DNS, which
            # guest Wi-Fi, corporate networks, VLANs and VPNs routinely drop -
            # and the person scanning this is standing here now, with no way to
            # work out why a name did not resolve.
            return info.get('qr_url') or info.get('best'), info
        except (urllib.error.URLError, OSError, ValueError, KeyError):
            continue
    return None, None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--url', help='encode this address instead of asking the robot')
    ap.add_argument('--svg', metavar='FILE', help='also write the QR as SVG')
    ap.add_argument('--quiet', action='store_true', help='print only the QR')
    args = ap.parse_args()

    info = None
    if args.url:
        url, source = args.url, 'given on the command line'
    else:
        url, info = ask_server()
        if url:
            source = 'reported by the running robot'
        else:
            url = 'http://' + DEFAULT_NAME
            source = ('GUESSED - no server answered, so this assumes port 80 '
                      'bound cleanly')

    plain = not sys.stdout.isatty() or os.environ.get('NO_COLOR')
    try:
        # Colours are stated explicitly for a terminal, because a QR drawn in
        # the terminal's own colours comes out inverted on a dark theme and many
        # phone cameras reject that. Piped or redirected output gets the plain
        # block form instead, which survives being pasted into a file.
        art = qr.to_ascii(url) if plain else qr.to_ansi(url)
    except ValueError as exc:
        raise SystemExit('cannot encode that address: %s' % exc)

    if not args.quiet:
        print()
        print('  %s' % url)
        print('  (%s)' % source)
        if info:
            name = info.get('mdns')
            if name and name != url:
                print('  %-16s %s' % ('or by name:', name))
                print('  %-16s %s' % ('', '(needs mDNS - some networks block it)'))
            local = info.get('local')
            if local and local != url:
                print('  %-16s %s' % ('on this machine:', local))
        print()

    for line in art.splitlines():
        print('  ' + line)

    if not args.quiet:
        print()
        print('  Point a phone camera at it, or type the address above.')
        print()

    if args.svg:
        with open(args.svg, 'w') as handle:
            handle.write(qr.to_svg(url))
        if not args.quiet:
            print('  SVG written to %s' % args.svg)
    return 0


if __name__ == '__main__':
    sys.exit(main())

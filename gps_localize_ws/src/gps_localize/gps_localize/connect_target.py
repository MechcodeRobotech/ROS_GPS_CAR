"""Work out what someone meant when they typed an address for the robot.

WHY THIS IS NOT JUST str.strip()

People type what they have in front of them, and what they have in front of them
is usually a terminal prompt or a label. All of these mean the same robot:

    module                      the bare name
    module.local               the mDNS name
    module@module.local        copied straight out of a shell prompt
    ssh://module.local:22      pasted from somewhere else
    192.168.1.50               the address someone read off a router
    http://192.168.1.50:8080   the URL from a browser bar
    /dev/ttyUSB0               plugged in over USB instead
    COM7                       the same, on Windows   # qc: allow-windows-path

Refusing any of them is unhelpful; guessing wrongly is worse. So this turns a
typed string into an explicit decision - what KIND of target it is, and what to
actually connect to - and says so, rather than silently doing something the
person did not ask for.

The `user@host` form is worth naming: it is what a shell prompt looks like, and
the part before the @ is a username, not part of the address. Treating the whole
string as a hostname produces a lookup for "module@module.local" that fails with
no hint as to why.
"""

from __future__ import annotations

import re

# A serial port on either platform. Linux/macOS give a path; Windows gives COM<n>.
_SERIAL_PATTERNS = (
    re.compile(r'^/dev/tty\S+$'),
    re.compile(r'^/dev/serial/\S+$'),
    re.compile(r'^/dev/cu\.\S+$'),
    re.compile(r'^COM\d+$', re.IGNORECASE),
)

_IPV4 = re.compile(r'^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$')

# RFC 1123: letters, digits and hyphens, and a label may not start or end with a
# hyphen. Underscores are NOT legal, which is why the robot's own name uses
# hyphens - avahi will publish an underscore quite happily and then no browser
# will resolve it.
_HOSTNAME_LABEL = re.compile(r'^[A-Za-z0-9]([A-Za-z0-9-]{0,61}[A-Za-z0-9])?$')


class Target:
    """What the user meant, made explicit."""

    SERIAL = 'serial'
    IP = 'ip'
    HOSTNAME = 'hostname'
    MDNS = 'mdns'
    INVALID = 'invalid'

    def __init__(self, kind, value, port=None, note='', user=None):
        self.kind = kind
        self.value = value          # what to actually connect to
        self.port = port            # None when the caller should use its default
        self.note = note            # why it was read this way, for the UI to show
        self.user = user            # stripped from user@host, kept for reference

    @property
    def ok(self):
        return self.kind != Target.INVALID

    def url(self, default_port=80):
        """The web address for this target, or None when it is a serial port."""
        if self.kind == Target.SERIAL or not self.ok:
            return None
        port = self.port or default_port
        host = self.value
        return 'http://%s' % host if port == 80 else 'http://%s:%d' % (host, port)

    def as_dict(self):
        return {'kind': self.kind, 'value': self.value, 'port': self.port,
                'note': self.note, 'user': self.user, 'ok': self.ok,
                'url': self.url()}

    def __repr__(self):                          # pragma: no cover - debugging
        return 'Target(%s, %r, port=%r)' % (self.kind, self.value, self.port)


def _valid_ipv4(text):
    m = _IPV4.match(text)
    if not m:
        return False
    return all(0 <= int(part) <= 255 for part in m.groups())


def _valid_hostname(text):
    if len(text) > 253:
        return False
    return all(_HOSTNAME_LABEL.match(label) for label in text.split('.') if True) \
        and text.count('..') == 0 and not text.startswith('.')


def parse(text: str) -> Target:
    """Turn what someone typed into an explicit target."""
    if text is None:
        return Target(Target.INVALID, '', note='nothing to connect to')
    raw = text.strip()
    if not raw:
        return Target(Target.INVALID, '', note='nothing to connect to')

    # Serial ports first: /dev/... would otherwise be mangled by URL parsing,
    # and COM7 looks enough like a hostname to be mistaken for one.
    for pattern in _SERIAL_PATTERNS:
        if pattern.match(raw):
            return Target(Target.SERIAL, raw, note='a serial port, so USB rather than Wi-Fi')

    # Strip a scheme if one was pasted. ssh:// appears because people copy from
    # wherever they last connected from.
    note_parts = []
    scheme = None
    if '://' in raw:
        scheme, raw = raw.split('://', 1)
        scheme = scheme.lower()
        note_parts.append('ignored the %s:// prefix' % scheme)
        raw = raw.split('/', 1)[0]               # drop any path
    raw = raw.rstrip('/')

    # user@host - what a shell prompt looks like. The username is not part of
    # the address, and leaving it in produces a lookup that fails with no hint.
    user = None
    if '@' in raw:
        user, raw = raw.rsplit('@', 1)
        note_parts.append('dropped the "%s@" - that is a username, not part of '
                          'the address' % user)

    # An explicit port. Only split on the LAST colon so an IPv6 literal is not
    # torn apart, and only when what follows is actually a number.
    port = None
    if ':' in raw and not raw.startswith('['):
        head, _, tail = raw.rpartition(':')
        if head and tail.isdigit() and 0 < int(tail) < 65536:
            raw, port = head, int(tail)

    if not raw:
        return Target(Target.INVALID, '', note='no host in that address')

    if _valid_ipv4(raw):
        note_parts.append('an IP address, which works even where mDNS is blocked')
        return Target(Target.IP, raw, port, '; '.join(note_parts), user)

    host = raw.lower()
    if not _valid_hostname(host):
        hint = ''
        if '_' in host:
            # Worth naming precisely: this is the mistake the project's own
            # naming was chosen to avoid.
            hint = (' - underscores are not legal in a hostname, try "%s"'
                    % host.replace('_', '-'))
        return Target(Target.INVALID, host,
                      note='that is not a valid hostname%s' % hint, user=user)

    if host.endswith('.local'):
        note_parts.append('an mDNS name - needs multicast, which some networks '
                          'block; use the IP if it does not resolve')
        return Target(Target.MDNS, host, port, '; '.join(note_parts), user)

    # A bare name. mDNS is tried first because that is what the robot publishes,
    # and plain DNS after, because a router that registers DHCP names will
    # answer for it.
    note_parts.append('a bare name - will try %s.local first, then plain DNS' % host)
    return Target(Target.HOSTNAME, host, port, '; '.join(note_parts), user)


def candidates(target: Target) -> list[str]:
    """Hosts to try, best first.

    A bare name gets both forms, because the robot answers to its mDNS name and
    a router may also answer for the plain one. Trying both costs one failed
    lookup and removes an entire class of "it works on my machine".
    """
    if not target.ok or target.kind == Target.SERIAL:
        return []
    if target.kind == Target.HOSTNAME:
        return ['%s.local' % target.value, target.value]
    return [target.value]

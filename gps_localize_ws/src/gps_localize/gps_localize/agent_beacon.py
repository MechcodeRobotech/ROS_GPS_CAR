#!/usr/bin/env python3
# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Announce where the micro-ROS agent is, so the robot never has to be told.

WHY THIS EXISTS

Every other way the board can find the agent breaks the moment something on
the network moves, and all three failures happened on this robot in a single
day:

  * an address set by hand goes stale as soon as the PC changes network, and
    the board then has no way back - it needed a USB cable to be told an
    address it could have been given;
  * mDNS is link-local, so it cannot cross from a phone hotspot to the wired
    LAN the control PC is on;
  * guessing the DHCP gateway is only right when the PC happens to BE the
    gateway, which is true for a shared connection and false for a router.

So instead of the board guessing, the host says. A tiny datagram goes out on
the broadcast address a few times a second:

    GPSLOC-AGENT 1 <ip> <port>

A board with no session adopts it; a board that is already linked ignores it.
Change the Wi-Fi, move the PC, hand out fresh leases, and the robot is back in
a couple of seconds with nothing to configure on either side.

WHAT THIS DELIBERATELY DOES NOT DO

It never interrupts a working link, because the firmware only listens while it
has no session. And the address the firmware acts on is the one the packet
CAME FROM, not the one written inside it - a datagram cannot lie about its
source without the reply going somewhere else.

Broadcasts are re-derived from the live interface list on every tick, so a new
address on this machine is announced without a restart. That is the case that
matters: the reason the robot got lost was that this PC's address changed.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

MAGIC = 'GPSLOC-AGENT'
BEACON_VERSION = 1
DEFAULT_BEACON_PORT = 8889
DEFAULT_AGENT_PORT = 8888


def broadcast_addresses():
    """Every IPv4 broadcast address this machine can send on, plus the global one.

    Read fresh each time rather than cached at startup. A cached list is exactly
    what stops working when the machine changes network, which is the fault this
    whole thing exists to survive.
    """
    addrs = {'255.255.255.255'}
    try:
        import fcntl  # Linux only; the fallback below covers everything else.
        SIOCGIFBRDADDR = 0x8919
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            for _, name in socket.if_nameindex():
                if name == 'lo':
                    continue
                try:
                    packed = fcntl.ioctl(
                        s.fileno(), SIOCGIFBRDADDR,
                        struct.pack('256s', name.encode()[:15]))
                    addrs.add(socket.inet_ntoa(packed[20:24]))
                except OSError:
                    continue        # interface has no broadcast (point-to-point, down)
        finally:
            s.close()
    except (ImportError, OSError):
        pass
    return sorted(addrs)


def local_ip_for(peer='8.8.8.8'):
    """This machine's address on the route that leaves it.

    Connecting a UDP socket sends nothing; it just makes the kernel pick the
    source address it would use, which is the one a robot on the LAN can reach.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((peer, 9))
        return s.getsockname()[0]
    except OSError:
        return '0.0.0.0'
    finally:
        s.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--beacon-port', type=int, default=DEFAULT_BEACON_PORT,
                    help='UDP port the robot listens on (default %d)' % DEFAULT_BEACON_PORT)
    ap.add_argument('--agent-port', type=int, default=DEFAULT_AGENT_PORT,
                    help='UDP port the micro-ROS agent listens on (default %d)'
                         % DEFAULT_AGENT_PORT)
    ap.add_argument('--interval', type=float, default=2.0,
                    help='seconds between announcements (default 2)')
    ap.add_argument('--once', action='store_true',
                    help='send a single announcement and exit, for testing')
    args = ap.parse_args(argv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    last_said = None
    try:
        while True:
            ip = local_ip_for()
            payload = ('%s %d %s %d' % (MAGIC, BEACON_VERSION, ip, args.agent_port)).encode()
            targets = broadcast_addresses()

            # Say something only when it changes. A line every two seconds
            # forever would bury the one message that matters - that the
            # address moved.
            if (ip, tuple(targets)) != last_said:
                last_said = (ip, tuple(targets))
                print('[beacon] announcing agent at %s:%d on %s'
                      % (ip, args.agent_port, ', '.join(targets)), flush=True)

            for target in targets:
                try:
                    sock.sendto(payload, (target, args.beacon_port))
                except OSError:
                    # An interface can disappear between listing and sending.
                    # That is normal while the network is changing, which is
                    # precisely when this is most needed - so carry on.
                    continue

            if args.once:
                return 0
            time.sleep(args.interval)
    except KeyboardInterrupt:
        return 0
    finally:
        sock.close()


if __name__ == '__main__':
    sys.exit(main())

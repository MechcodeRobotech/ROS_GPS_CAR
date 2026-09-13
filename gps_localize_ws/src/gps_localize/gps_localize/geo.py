# Copyright 2026 Phuthiphong Wongchantib
#
# Licensed under the Apache License, Version 2.0.
#
# Original author / contributor:
# Phuthiphong Wongchantib
"""Geodesy helpers for GPS waypoint navigation.

Everything works in WGS-84 degrees in, metres/degrees out. The robot moves a
few hundred metres at most, so an equirectangular projection around a local
datum is accurate to a few centimetres and costs a fraction of a proper UTM
conversion (no extra dependency either).
"""

import math

EARTH_RADIUS_M = 6378137.0


def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance between two WGS-84 points [m]."""
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp * 0.5) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl * 0.5) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial bearing from point 1 to point 2, 0..360, 0 = north, clockwise."""
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def enu_offset_m(lat, lon, datum_lat, datum_lon):
    """Local east/north offset from a datum [m] (equirectangular)."""
    east = math.radians(lon - datum_lon) * EARTH_RADIUS_M * math.cos(math.radians(datum_lat))
    north = math.radians(lat - datum_lat) * EARTH_RADIUS_M
    return east, north


def enu_to_latlon(east, north, datum_lat, datum_lon):
    """Inverse of :func:`enu_offset_m`."""
    lat = datum_lat + math.degrees(north / EARTH_RADIUS_M)
    lon = datum_lon + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(datum_lat))))
    return lat, lon


def wrap360(deg):
    return deg % 360.0


def angle_error_deg(target, current):
    """Shortest signed angle target-current, -180..180."""
    return (target - current + 540.0) % 360.0 - 180.0


def compass_to_enu_deg(compass_deg):
    """Compass bearing (0 = north, CW) -> ENU heading (0 = east, CCW)."""
    return (90.0 - compass_deg) % 360.0


def enu_to_compass_deg(enu_deg):
    """ENU heading -> compass bearing."""
    return (90.0 - enu_deg) % 360.0

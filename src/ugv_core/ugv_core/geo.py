"""Latitude/longitude <-> local tangent plane (east, north) in metres.

An equirectangular approximation around a datum is accurate to millimetres
over the few hundred metres a UGV covers in this project.
"""
import math

R_EARTH = 6378137.0  # WGS-84 equatorial radius [m]


def ll_to_en(lat_deg, lon_deg, lat0_deg, lon0_deg):
    """Return (east, north) metres of (lat, lon) relative to the datum."""
    lat0 = math.radians(lat0_deg)
    east = R_EARTH * math.radians(lon_deg - lon0_deg) * math.cos(lat0)
    north = R_EARTH * math.radians(lat_deg - lat0_deg)
    return east, north


def en_to_ll(east, north, lat0_deg, lon0_deg):
    """Inverse of :func:`ll_to_en`."""
    lat0 = math.radians(lat0_deg)
    lat = lat0_deg + math.degrees(north / R_EARTH)
    lon = lon0_deg + math.degrees(east / (R_EARTH * math.cos(lat0)))
    return lat, lon

"""WGS84 geodetic position to a fixed local East/North/Up datum."""
import math


def ecef(lat, lon, altitude):
    if not all(math.isfinite(v) for v in (lat, lon, altitude)) or not -90 <= lat <= 90 or not -180 <= lon <= 180:
        raise ValueError('Invalid geodetic position')
    lat, lon = math.radians(lat), math.radians(lon)
    a, e2 = 6378137.0, 6.69437999014e-3
    n = a / math.sqrt(1-e2 * math.sin(lat)**2)
    return ((n+altitude)*math.cos(lat)*math.cos(lon),
            (n+altitude)*math.cos(lat)*math.sin(lon),
            (n*(1-e2)+altitude)*math.sin(lat))


def enu(lat, lon, altitude, datum):
    point, origin = ecef(lat, lon, altitude), ecef(*datum)
    dx, dy, dz = (p-o for p,o in zip(point, origin))
    phi, lam = math.radians(datum[0]), math.radians(datum[1])
    return (-math.sin(lam)*dx+math.cos(lam)*dy,
            -math.sin(phi)*math.cos(lam)*dx-math.sin(phi)*math.sin(lam)*dy+math.cos(phi)*dz,
            math.cos(phi)*math.cos(lam)*dx+math.cos(phi)*math.sin(lam)*dy+math.sin(phi)*dz)

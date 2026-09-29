"""Seeded independent sensor error streams; SI standard deviations and variances."""
import math
import random


class SensorNoise:
    def __init__(self, seed):
        self.streams = {name: random.Random(int(seed) + offset)
                        for name, offset in [('orientation', 0), ('gps', 10000),
                                             ('angular_velocity', 20000), ('linear_acceleration', 30000)]}

    def sample(self, channel, stds):
        if any(not math.isfinite(s) or s < 0 for s in stds):
            raise ValueError('noise standard deviations must be finite and nonnegative')
        return tuple(self.streams[channel].gauss(0., s) if s else 0. for s in stds)


def covariance(stds):
    return [stds[i//3]**2 if i in (0, 4, 8) else 0. for i in range(9)]


def noisy_orientation(q, errors):
    """Apply a Gaussian rotation vector in ENU, normalizing the result."""
    angle = math.sqrt(sum(v*v for v in errors))
    scale = math.sin(angle/2)/angle if angle else .5
    x, y, z = (v*scale for v in errors)
    w = math.cos(angle/2)
    a, b, c, d = q
    result = (w*a+x*d+y*c-z*b, w*b-x*c+y*d+z*a,
              w*c+x*b-y*a+z*d, w*d-x*a-y*b-z*c)
    norm = math.sqrt(sum(v*v for v in result))
    return tuple(v/norm for v in result)


def noisy_fix(latitude, longitude, altitude, errors):
    """Convert north/east/up metric errors to WGS84 geographic coordinates."""
    north, east, up = errors
    lat = math.radians(latitude)
    a, e2 = 6378137., 6.69437999014e-3
    den = 1.-e2*math.sin(lat)**2
    meridian, prime = a*(1.-e2)/den**1.5, a/math.sqrt(den)
    return (latitude + math.degrees(north/meridian),
            longitude + math.degrees(east/(prime*max(1e-6, math.cos(lat)))), altitude+up)

"""Ray-traced occupancy with explicit unknown cells and expiring observations."""
import math
import numpy as np


def grid_line(start, end):
    """Bresenham cells, including both endpoints; coordinates are (column, row)."""
    x, y = start
    x1, y1 = end
    dx, dy = abs(x1-x), -abs(y1-y)
    sx, sy = 1 if x < x1 else -1, 1 if y < y1 else -1
    err = dx + dy
    while True:
        yield x, y
        if (x, y) == (x1, y1):
            return
        twice = 2*err
        if twice >= dy:
            err += dy
            x += sx
        if twice <= dx:
            err += dx
            y += sy


class OccupancyMapper:
    def __init__(self, resolution=0.5, size_m=160.0, origin=(-40., -40.),
                 inflation_m=3.0, observation_ttl_s=5.0):
        if resolution <= 0 or size_m <= 0 or observation_ttl_s <= 0 or inflation_m < 0:
            raise ValueError("invalid map dimensions, inflation or observation lifetime")
        self.resolution = float(resolution)
        self.size = int(math.ceil(size_m / resolution))
        self.origin = np.asarray(origin, dtype=float)
        self.inflation = float(inflation_m)
        self.ttl = float(observation_ttl_s)
        self.values = np.full((self.size, self.size), -1, dtype=np.int8)
        self.observed = np.full(self.values.shape, -np.inf)
        self.last_stamp = None

    def cell(self, xy):
        return tuple(np.floor((np.asarray(xy)[:2]-self.origin) / self.resolution).astype(int))

    def inside(self, cell):
        return 0 <= cell[0] < self.size and 0 <= cell[1] < self.size

    def update(self, sensor_origin, endpoints, occupied, stamp):
        """Integrate already filtered world-space rays; occupied=False clears to endpoint.

        Endpoint hits win over free rays within a cloud. Out-of-order clouds are
        rejected. A clock reset must explicitly reset this instance in the adapter.
        """
        stamp = float(stamp)
        if self.last_stamp is not None and stamp < self.last_stamp:
            return False
        start = self.cell(sensor_origin)
        if not self.inside(start):
            return False
        free, hits = set(), set()
        rays = {(self.cell(point), bool(hit)) for point, hit in zip(endpoints, occupied) if np.isfinite(point).all()}
        for end, hit in rays:
            # Bound traversal even for corrupted or very distant endpoints.
            if max(abs(end[0]-start[0]), abs(end[1]-start[1])) > self.size*4:
                continue
            for cell in grid_line(start, end):
                if not self.inside(cell):
                    break
                if cell == end and hit:
                    hits.add(cell)
                else:
                    free.add(cell)
        for cells, value in ((free-hits, 0), (hits, 100)):
            if cells:
                xy = np.asarray(list(cells))
                if value == 0:
                    # A scan and cloud can arrive in either order. Recent cloud
                    # hits beat a sparse scan's free ray through the same cell.
                    keep = ~((self.values[xy[:, 1], xy[:, 0]] == 100) &
                             (stamp-self.observed[xy[:, 1], xy[:, 0]] <= 0.3))
                    xy = xy[keep]
                self.values[xy[:, 1], xy[:, 0]] = value
                self.observed[xy[:, 1], xy[:, 0]] = stamp
        self.last_stamp = stamp
        return True

    def grid(self, now):
        expired = float(now) - self.observed > self.ttl
        result = self.values.copy()
        result[expired] = -1
        # Only the output is inflated; observed raw occupancy remains recoverable.
        rows, cols = np.nonzero(result == 100)
        radius = int(math.ceil(self.inflation/self.resolution))
        for dr in range(-radius, radius+1):
            for dc in range(-radius, radius+1):
                if (dr*self.resolution)**2+(dc*self.resolution)**2 > self.inflation**2:
                    continue
                rr, cc = rows+dr, cols+dc
                valid = (rr >= 0)&(rr < self.size)&(cc >= 0)&(cc < self.size)
                result[rr[valid], cc[valid]] = 100
        return result

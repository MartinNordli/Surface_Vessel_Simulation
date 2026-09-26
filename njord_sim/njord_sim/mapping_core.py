"""Ray-traced occupancy with explicit unknown cells and expiring observations.

Pure-Python core of the ``mapper`` node. Lidar rays are traced in 2D (x, y of
the ENU ``map`` frame) from the sensor to each endpoint: cells along the ray
become observed free, the endpoint of a real return becomes occupied. Cells
never reached by a ray stay unknown. No area around the vessel is cleared
without a ray passing through it.

Cell values follow ``nav_msgs/OccupancyGrid``:
    -1   unknown (never observed, or last observation expired);
     0   observed free;
    100  occupied (and, in the published grid only, inflated around obstacles).
"""
import math
import numpy as np


def grid_line(start, end):
    """Bresenham cells, including both endpoints; coordinates are (column, row).

    Generator over the integer cells of a straight line from ``start`` to
    ``end``. Consecutive cells may touch only diagonally.
    """
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
    """Square occupancy grid with per-cell evidence timestamps.

    Args:
        resolution: cell edge length [m].
        size_m: grid edge length [m]; the grid has ``ceil(size_m /
            resolution)`` cells per side.
        origin: map (x, y) of the lower-left corner of cell (0, 0) [m].
        inflation_m: radius by which obstacles are grown in the published grid
            [m], usually the vessel's circumscribed radius plus a margin.
        observation_ttl_s: a cell reverts to unknown when the observation that
            decided its value is older than this [s].

    State (arrays are indexed ``[row, column]`` = ``[y, x]``, the row-major
    layout of ``OccupancyGrid.data``):
        ``values``: raw, uninflated cell values -1 / 0 / 100.
        ``free_observed``, ``hit_observed``: newest acquisition stamp [s] of
            free and of occupied evidence per cell (``-inf`` if none).
        ``observed``: stamp of the evidence that decided the current value;
            used for TTL aging.
        ``stream_stamps``: newest accepted stamp per input stream.
        ``last_stamp``: newest stamp over all streams.

    All stamps are sensor acquisition times in simulation seconds, never the
    time of arrival or processing.
    """

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
        self.free_observed = np.full(self.values.shape, -np.inf)
        self.hit_observed = np.full(self.values.shape, -np.inf)
        self.stream_stamps = {}
        self.last_stamp = None

    def cell(self, xy):
        """Return the (column, row) cell containing map point ``xy`` [m].

        Only x and y are used; the result may lie outside the grid.
        """
        return tuple(np.floor((np.asarray(xy)[:2]-self.origin) / self.resolution).astype(int))

    def inside(self, cell):
        """True if the (column, row) cell lies within the grid."""
        return 0 <= cell[0] < self.size and 0 <= cell[1] < self.size

    def update(self, sensor_origin, endpoints, occupied, stamp, stream="cloud"):
        """Integrate already filtered world-space rays; occupied=False clears to endpoint.

        Reject regressions within each input stream, not between scan and cloud.
        Each cell retains the acquisition time of its free and occupied evidence;
        hits win over free evidence up to 0.3 seconds newer, independent of arrival
        order. A clock reset explicitly resets this instance in the adapter.

        Args:
            sensor_origin: sensor position in map at acquisition time [m].
            endpoints: (N, >=2) ray endpoints in map [m].
            occupied: N booleans; True marks the endpoint cell as a hit,
                False means the ray only clears cells up to and including
                its endpoint (e.g. a no-return ray clipped at max range).
            stamp: acquisition time of the measurement [s, simulation time].
            stream: input name (``"cloud"`` or ``"scan"``); stamp ordering is
                checked per stream.

        Returns:
            True if the measurement was accepted (even if no cell changed);
            False if the stamp is non-finite, older than the stream's last
            accepted stamp, or the sensor lies outside the grid.

        Only cells crossed by a ray in this call are re-evaluated. Within one
        call, a cell that is a hit for any ray is not also counted as free.
        """
        stamp = float(stamp)
        if not math.isfinite(stamp) or stamp < self.stream_stamps.get(stream, -math.inf):
            return False
        start = self.cell(sensor_origin)
        if not self.inside(start):
            return False
        free, hits = set(), set()
        # Many rays end in the same cell; tracing each (cell, hit) pair once is enough.
        rays = {(self.cell(point), bool(hit)) for point, hit in zip(endpoints, occupied)
                if np.isfinite(point).all()}
        for end, hit in rays:
            # Bound traversal even for corrupted or very distant endpoints.
            if max(abs(end[0]-start[0]), abs(end[1]-start[1])) > self.size*4:
                continue
            for cell in grid_line(start, end):
                # Stop at the map edge: nothing beyond it is stored.
                if not self.inside(cell):
                    break
                if cell == end and hit:
                    hits.add(cell)
                else:
                    free.add(cell)
        # Record evidence times. np.maximum keeps the newest stamp per cell, so
        # a late-arriving older message cannot overwrite newer evidence.
        for cells, timestamps in ((free-hits, self.free_observed), (hits, self.hit_observed)):
            if cells:
                xy = np.asarray(list(cells))
                rows, cols = xy[:, 1], xy[:, 0]
                timestamps[rows, cols] = np.maximum(timestamps[rows, cols], stamp)
        if free or hits:
            xy = np.asarray(list(free | hits))
            rows, cols = xy[:, 1], xy[:, 0]
            free_times, hit_times = self.free_observed[rows, cols], self.hit_observed[rows, cols]
            # Occupied unless free evidence is more than 0.3 s newer than the
            # last hit: biased towards keeping obstacles.
            occupied_now = np.isfinite(hit_times) & (hit_times >= free_times-0.3)
            self.values[rows, cols] = np.where(occupied_now, 100, 0)
            self.observed[rows, cols] = np.where(occupied_now, hit_times, free_times)
        self.stream_stamps[stream] = stamp
        self.last_stamp = max(self.stream_stamps.values())
        return True

    def grid(self, now):
        """Return the grid to publish at simulation time ``now`` [s].

        Cells whose deciding observation is older than ``observation_ttl_s``
        become unknown (-1). Then every remaining occupied cell is grown by a
        disk of radius ``inflation_m``, marking cells within it as 100
        regardless of their value (free or unknown). Inflation is applied to
        this copy only; the stored raw values are unchanged.

        Returns:
            (size, size) int8 array indexed ``[row, column]``.
        """
        expired = float(now) - self.observed > self.ttl
        result = self.values.copy()
        result[expired] = -1
        # Only the output is inflated; observed raw occupancy remains recoverable.
        rows, cols = np.nonzero(result == 100)
        radius = int(math.ceil(self.inflation/self.resolution))
        # Shift the set of occupied cells by every offset inside the disk.
        for dr in range(-radius, radius+1):
            for dc in range(-radius, radius+1):
                if (dr*self.resolution)**2+(dc*self.resolution)**2 > self.inflation**2:
                    continue
                rr, cc = rows+dr, cols+dc
                valid = (rr >= 0)&(rr < self.size)&(cc >= 0)&(cc < self.size)
                result[rr[valid], cc[valid]] = 100
        return result

"""ROS-independent grid geometry and persistent D* Lite planning."""

from dataclasses import dataclass
import math

from njord_sim.dstar_lite import DStarLite, Grid


def fresh(now, stamp, timeout):
    """Use acquisition timestamps, rejecting clock regressions/future data."""
    return stamp is not None and math.isfinite(stamp) and 0 <= now - stamp <= timeout


@dataclass(frozen=True)
class Geometry:
    width: int
    height: int
    resolution: float
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    frame: str = "map"

    def __post_init__(self):
        if (self.width <= 0 or self.height <= 0 or not self.frame
                or self.resolution <= 0
                or not all(math.isfinite(v) for v in
                           (self.resolution, self.x, self.y, self.z, self.yaw))):
            raise ValueError("invalid occupancy grid geometry")

    @classmethod
    def from_message(cls, msg, frame):
        p, q = msg.info.origin.position, msg.info.origin.orientation
        values = (q.x, q.y, q.z, q.w)
        if (msg.header.frame_id != frame or not all(math.isfinite(v) for v in values)
                or abs(sum(v * v for v in values) - 1.0) > 1e-3
                or abs(q.x) > 1e-6 or abs(q.y) > 1e-6):
            raise ValueError("occupancy grid must have matching frame and planar unit quaternion")
        return cls(msg.info.width, msg.info.height, msg.info.resolution,
                   p.x, p.y, p.z, 2 * math.atan2(q.z, q.w), frame)

    def cell(self, point):
        dx, dy = point[0] - self.x, point[1] - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (math.floor((-s * dx + c * dy) / self.resolution),
                math.floor((c * dx + s * dy) / self.resolution))

    def world(self, cell):
        x, y = (cell[1] + 0.5) * self.resolution, (cell[0] + 0.5) * self.resolution
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * x - s * y, self.y + s * x + c * y

    def validate_data(self, data):
        if len(data) != self.width * self.height or any(v < -1 or v > 100 for v in data):
            raise ValueError("invalid occupancy data")


class IncrementalPlanner:
    def __init__(self):
        self.geometry = None
        self.search = None

    def plan(self, geometry, data, position, goal):
        geometry.validate_data(data)
        if not all(math.isfinite(v) for v in (*position, *goal)):
            raise ValueError("nonfinite planning endpoint")
        start, end = geometry.cell(position), geometry.cell(goal)
        blocked = {(i // geometry.width, i % geometry.width)
                   for i, value in enumerate(data) if value >= 50}
        if self.geometry != geometry or self.search is None or self.search.goal != end:
            grid = Grid(geometry.height, geometry.width)
            grid.blocked = blocked
            self.search = DStarLite(grid, start, end)
            self.geometry = geometry
        else:
            changed = blocked.symmetric_difference(self.search.grid.blocked)
            self.search.grid.blocked = blocked
            self.search.update_start(start)
            self.search.apply_changes(changed)
        if not self.search.grid.is_free(start) or not self.search.grid.is_free(end):
            return []
        self.search.compute_shortest_path()
        return [geometry.world(cell) for cell in self.search.path()]

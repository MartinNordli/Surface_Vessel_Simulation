"""ROS-independent grid geometry and persistent D* Lite planning.

``planner_node.py`` and ``guidance_node.py`` use these classes; they are kept
free of ROS so they can be unit tested on their own.
"""

from dataclasses import dataclass
import math

from njord_sim.dstar_lite import DStarLite, Grid


def fresh(now, stamp, timeout):
    """Use acquisition timestamps, rejecting clock regressions/future data.

    Args:
        now: current simulation time (/clock) in s.
        stamp: acquisition time of the data (its header stamp) in s, or None.
        timeout: maximum allowed age in s.

    Returns:
        True if the data exists and ``0 <= now - stamp <= timeout``. Data from
        the future (for example after the simulation clock was reset) is
        rejected rather than trusted.
    """
    return stamp is not None and math.isfinite(stamp) and 0 <= now - stamp <= timeout


@dataclass(frozen=True)
class Geometry:
    """Placement of an occupancy grid in the map frame.

    Attributes:
        width, height: grid size in cells (columns, rows).
        resolution: cell edge length in m.
        x, y, z: map-frame position of the grid origin (corner of cell (0, 0))
            in m.
        yaw: rotation of the grid about map z in rad.
        frame: frame the grid is expressed in (the map frame).

    Cells are addressed as (row, col); columns run along the grid's x axis and
    rows along its y axis. The dataclass is frozen and compares by value, so
    the planner can detect a changed grid layout with ``==``.
    """
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
        """Build a Geometry from a ``nav_msgs/OccupancyGrid`` message.

        Raises ValueError unless the grid is in ``frame`` and its origin
        orientation is a unit quaternion rotating only about z (a planar grid).
        """
        p, q = msg.info.origin.position, msg.info.origin.orientation
        values = (q.x, q.y, q.z, q.w)
        if (msg.header.frame_id != frame or not all(math.isfinite(v) for v in values)
                or abs(sum(v * v for v in values) - 1.0) > 1e-3
                or abs(q.x) > 1e-6 or abs(q.y) > 1e-6):
            raise ValueError("occupancy grid must have matching frame and planar unit quaternion")
        # For a pure z rotation, yaw = 2 * atan2(qz, qw).
        return cls(msg.info.width, msg.info.height, msg.info.resolution,
                   p.x, p.y, p.z, 2 * math.atan2(q.z, q.w), frame)

    def cell(self, point):
        """Return the (row, col) of the cell containing a map-frame (x, y) point.

        The result may lie outside the grid; callers check bounds.
        """
        dx, dy = point[0] - self.x, point[1] - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (math.floor((-s * dx + c * dy) / self.resolution),
                math.floor((c * dx + s * dy) / self.resolution))

    def world(self, cell):
        """Return the map-frame (x, y) in m of the centre of a (row, col) cell."""
        x, y = (cell[1] + 0.5) * self.resolution, (cell[0] + 0.5) * self.resolution
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * x - s * y, self.y + s * x + c * y

    def validate_data(self, data):
        """Raise ValueError unless ``data`` has one value in [-1, 100] per cell."""
        if len(data) != self.width * self.height or any(v < -1 or v > 100 for v in data):
            raise ValueError("invalid occupancy data")


class IncrementalPlanner:
    """Keeps one D* Lite search alive between calls so replanning is cheap.

    The search is rebuilt from scratch when the grid layout (``Geometry``) or
    the goal cell changes; otherwise only the changed cells are passed to D*
    Lite and the start is moved to the vessel's current cell.
    """

    def __init__(self):
        self.geometry = None
        self.search = None

    def plan(self, geometry, data, position, goal):
        """Plan from ``position`` to ``goal`` (map-frame (x, y) in m).

        Cells with value 50 or more are blocked for the search. Unknown cells
        (-1) are planned through, so the route can lead into unexplored water;
        guidance only follows the part of it that is observed free.

        Returns:
            List of map-frame (x, y) cell centres from start to goal, or an
            empty list if the start or goal cell is blocked/outside the grid or
            no path exists.

        Raises:
            ValueError: for invalid occupancy data or non-finite endpoints.
        """
        geometry.validate_data(data)
        if not all(math.isfinite(v) for v in (*position, *goal)):
            raise ValueError("nonfinite planning endpoint")
        start, end = geometry.cell(position), geometry.cell(goal)
        blocked = {(i // geometry.width, i % geometry.width)
                   for i, value in enumerate(data) if value >= 50}
        if self.geometry != geometry or self.search is None or self.search.goal != end:
            # New grid layout or new goal: the old search tree is meaningless.
            grid = Grid(geometry.height, geometry.width)
            grid.blocked = blocked
            self.search = DStarLite(grid, start, end)
            self.geometry = geometry
        else:
            # Same layout and goal: repair the existing search incrementally
            # with only the cells whose blocked state flipped.
            changed = blocked.symmetric_difference(self.search.grid.blocked)
            self.search.grid.blocked = blocked
            self.search.update_start(start)
            self.search.apply_changes(changed)
        if not self.search.grid.is_free(start) or not self.search.grid.is_free(end):
            return []
        self.search.compute_shortest_path()
        return [geometry.world(cell) for cell in self.search.path()]

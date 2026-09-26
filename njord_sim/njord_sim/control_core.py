"""Pure collision checks and differential-thrust guidance helpers.

This module has no ROS dependency; ``guidance_node.py`` wires it to topics.
It provides:

* corridor checks on the occupancy grid (``segment_is_free``,
  ``tracking_corridor``, ``clearance``),
* the speed cap used by guidance (``speed_limit``),
* two ways to turn a surge force (N) and yaw moment (N*m) into two thruster
  forces in newtons (``mix_thrusters`` and ``allocate_thrusters``).

Occupancy values follow ``nav_msgs/OccupancyGrid``: -1 unknown, 0 free,
100 occupied. Only cells that are exactly 0 (observed free) count as free
here; unknown and occupied cells, and anything outside the grid, are treated
as blocked. The input grid must already include the vessel footprint and
safety inflation, so the vessel can be treated as a point.

Braking/deceleration parameters are reference assumptions until dynamics
trials provide measurements; they are not a guarantee under arbitrary
wind/current.
"""

import math


def wrap(angle):
    """Wrap an angle in rad to the interval [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _local(geometry, point):
    """Convert a map-frame (x, y) point in m to continuous grid coordinates.

    Returns (x, y) in units of cells relative to the grid origin, rotated into
    the grid's own axes: x runs along columns and y along rows, so
    ``floor(x)`` is the column index and ``floor(y)`` the row index.
    """
    dx, dy = point[0] - geometry.x, point[1] - geometry.y
    c, s = math.cos(geometry.yaw), math.sin(geometry.yaw)
    return ((c * dx + s * dy) / geometry.resolution,
            (-s * dx + c * dy) / geometry.resolution)


def observed_free(geometry, data, cell):
    """True only if the (row, col) cell is inside the grid and observed free (0).

    Unknown (-1) and occupied cells are deliberately not free: guidance may
    only drive through water that the sensors have actually seen to be empty.
    """
    r, c = cell
    return 0 <= r < geometry.height and 0 <= c < geometry.width and data[r * geometry.width + c] == 0


def segment_is_free(geometry, data, start, end):
    """Supercover traversal: reject unknown cells and diagonal corner cutting.

    Walks every grid cell touched by the straight segment from ``start`` to
    ``end`` (both map-frame (x, y) in m) and returns True only if all of them
    are observed free. This is a grid traversal in the style of Amanatides and
    Woo, made conservative ("supercover") in two cases where the segment only
    touches cells at their edge or corner:

    * a segment passing exactly through a grid corner must have both cells
      beside that corner free, so the boat cannot squeeze diagonally between
      two blocked cells;
    * a segment lying exactly on a grid line touches the cells on both sides.
    """
    x, y = _local(geometry, start)
    ex, ey = _local(geometry, end)
    col, row = math.floor(x), math.floor(y)
    end_col, end_row = math.floor(ex), math.floor(ey)
    dx, dy = ex - x, ey - y
    sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    # tx/ty: segment parameter (0 at start, 1 at end) at which the segment next
    # crosses a vertical/horizontal cell boundary. dtx/dty: parameter step
    # between successive boundaries. Infinity means "never crosses".
    tx = ((col + (dx > 0) - x) / dx) if dx else math.inf
    ty = ((row + (dy > 0) - y) / dy) if dy else math.inf
    dtx, dty = (abs(1 / dx) if dx else math.inf), (abs(1 / dy) if dy else math.inf)
    # At most |d col| + |d row| + 1 cells are visited; one extra iteration is
    # slack. Running out of iterations is treated as "not free".
    for _ in range(abs(end_col - col) + abs(end_row - row) + 2):
        if not observed_free(geometry, data, (row, col)):
            return False
        # A segment lying on a grid boundary touches cells on both sides.
        if dx == 0 and x == math.floor(x) and not observed_free(geometry, data, (row, col - 1)):
            return False
        if dy == 0 and y == math.floor(y) and not observed_free(geometry, data, (row - 1, col)):
            return False
        if (row, col) == (end_row, end_col):
            return True
        if abs(tx - ty) < 1e-12:
            # The segment passes exactly through a cell corner: it touches
            # both side neighbours, so both must be free (no corner cutting).
            if not (observed_free(geometry, data, (row, col + sx))
                    and observed_free(geometry, data, (row + sy, col))):
                return False
            col, row, tx, ty = col + sx, row + sy, tx + dtx, ty + dty
        elif tx < ty:
            col, tx = col + sx, tx + dtx
        else:
            row, ty = row + sy, ty + dty
    return False


def tracking_corridor(geometry, data, path, position, lookahead):
    """Return a visible tracking target and free length along the chosen corridor.

    Args:
        geometry: grid ``Geometry`` (map frame).
        data: flat occupancy values, row-major.
        path: list of map-frame (x, y) waypoints in m, start to goal.
        position: vessel map-frame (x, y) in m.
        lookahead: preferred distance to the target in m.

    Starting at the path vertex nearest the vessel, the target advances along
    the path while the straight line from the vessel to the vertex stays
    observed free, and stops at the first vertex at least ``lookahead`` away
    (line-of-sight guidance).

    Returns:
        ``(target, free_length)``: the target point and the distance in m that
        is known to be free: vessel to target plus the following path segments
        until the first blocked or unknown one. ``(None, 0.0)`` if the path is
        empty, the vessel's own cell is not observed free, or no vertex is
        visible; the caller must then stop.
    """
    if not path or not observed_free(geometry, data, geometry.cell(position)):
        return None, 0.0
    nearest = min(range(len(path)), key=lambda i: math.dist(position, path[i]))
    target_index = None
    for i in range(nearest, len(path)):
        if not segment_is_free(geometry, data, position, path[i]):
            break
        target_index = i
        if math.dist(position, path[i]) >= lookahead:
            break
    if target_index is None:
        return None, 0.0
    target = path[target_index]
    free_length = math.dist(position, target)
    previous = target
    # Extend the known-free distance beyond the target so the braking limit in
    # speed_limit() does not slow the boat for a corridor that continues.
    for point in path[target_index + 1:]:
        if not segment_is_free(geometry, data, previous, point):
            break
        free_length += math.dist(previous, point)
        previous = point
    return target, free_length


def clearance(geometry, data, position, radius=5.0):
    """Conservative additional distance to unknown/occupied cell boundaries.

    Scans a square window of about ``radius`` m around the map-frame
    ``position`` and returns the smallest distance in m to any cell that is not
    observed free, capped at ``radius``. The distance is taken to the cell
    centre minus half the cell diagonal, so it never overestimates the gap.
    Because the grid is already inflated, this is clearance beyond the safety
    inflation, not distance to the physical obstacle.
    """
    row, col = geometry.cell(position)
    cells = math.ceil(radius / geometry.resolution) + 1
    best = radius
    half_diagonal = geometry.resolution / math.sqrt(2)
    for r in range(row - cells, row + cells + 1):
        for c in range(col - cells, col + cells + 1):
            if not observed_free(geometry, data, (r, c)):
                best = min(best, max(0.0, math.dist(position, geometry.world((r, c))) - half_diagonal))
    return best


def speed_limit(max_speed, heading_error, free_distance, clearance_m,
                current_speed, deceleration, reaction_s, margin_m):
    """Surge speed setpoint in m/s: the most restrictive of four limits.

    Args:
        max_speed: configured speed ceiling in m/s.
        heading_error: bearing to target minus vessel yaw, in rad.
        free_distance: observed-free distance ahead along the path, in m.
        clearance_m: extra clearance to non-free cells, in m (``clearance``).
        current_speed: measured surge speed in m/s.
        deceleration: assumed braking deceleration in m/s^2.
        reaction_s: assumed delay before braking starts, in s.
        margin_m: distance to keep free when stopped, in m.
    """
    # Braking: distance left after the stopping margin and the distance covered
    # at the current speed during the reaction time. v = sqrt(2 a d) is the
    # highest speed from which the boat can stop within that distance at
    # constant deceleration a.
    usable = max(0.0, free_distance - margin_m - abs(current_speed) * reaction_s)
    braking = math.sqrt(2.0 * deceleration * usable)
    # Turning: slow down smoothly with heading error; zero when the target is
    # 90 deg or more off the bow, so the boat turns before it accelerates.
    turn = max_speed * max(0.0, math.cos(heading_error)) ** 2
    # Near obstacles: linear ramp from zero at 0 m clearance to full speed at
    # 3 m or more.
    near_obstacles = max_speed * min(1.0, max(0.0, clearance_m) / 3.0)
    return min(max_speed, braking, turn, near_obstacles)


def mix_thrusters(force, moment, separation, max_thrust):
    """For thrusters at y=+/-separation/2, yaw moment = (R-L)*separation/2.

    Simple differential-thrust mixer for two parallel, forward-pointing
    thrusters placed symmetrically about the body x axis (body frame:
    x forward, y left, z up; left thruster at +y).

    Args:
        force: requested surge force in N (L + R).
        moment: requested yaw moment in N*m, positive counter-clockwise (turn
            left).
        separation: lateral distance between the thrusters in m.
        max_thrust: per-thruster magnitude limit in N (forward and reverse).

    Returns:
        ``(left, right)`` thrust in N. If either exceeds ``max_thrust`` both are
        scaled down by the same factor, which keeps the force/moment ratio (the
        turning intent) instead of clipping one side.

    Raises:
        ValueError: for non-finite input or non-positive separation/limit.
    """
    if not all(math.isfinite(v) for v in (force, moment, separation, max_thrust)) or separation <= 0 or max_thrust <= 0:
        raise ValueError("invalid physical thruster parameters")
    # Solve L + R = force and (R - L) * separation / 2 = moment.
    left, right = force / 2 - moment / separation, force / 2 + moment / separation
    scale = max(1.0, abs(left) / max_thrust, abs(right) / max_thrust)
    return left / scale, right / scale


def allocate_thrusters(force, moment, positions, axes, forward_limits, reverse_limits):
    """Solve the physical surge/yaw allocation, then uniformly saturate in N.

    Positions and axes are two flattened xyz vectors in the body frame, referred
    to the same origin as the controller wrench. Reverse limits are magnitudes.
    Any uncommanded sway from canted fixed thrusters remains physical.

    Args:
        force: requested surge force in N along body x.
        moment: requested yaw moment in N*m about body z.
        positions: ``[x0, y0, z0, x1, y1, z1]`` thruster positions in m
            (thruster 0 is left, thruster 1 is right).
        axes: ``[ax0, ay0, az0, ax1, ay1, az1]`` thrust directions (the
            direction of force for positive thrust).
        forward_limits: maximum positive thrust per thruster in N.
        reverse_limits: maximum reverse thrust magnitude per thruster in N.

    Returns:
        ``(thrust0, thrust1)`` in N. If either exceeds its limit for its sign,
        both are scaled by the same factor so the force/moment ratio is kept.

    Raises:
        ValueError: for wrong vector lengths, non-finite values, non-positive
            limits, or a geometry that cannot control surge and yaw
            independently (singular allocation matrix).
    """
    if len(positions) != 6 or len(axes) != 6 or len(forward_limits) != 2 or len(reverse_limits) != 2:
        raise ValueError('allocation requires two physical thrusters')
    if not all(math.isfinite(v) for v in (force, moment, *positions, *axes, *forward_limits, *reverse_limits)):
        raise ValueError('nonfinite thruster allocation')
    if min(*forward_limits, *reverse_limits) <= 0:
        raise ValueError('thruster limits must be positive')
    # Thrust T_i along axis u_i at position r_i gives surge u_i.x * T_i and yaw
    # moment (r_i x u_i).z * T_i = (r_x * u_y - r_y * u_x) * T_i. That is the
    # 2x2 system  [a b; c d] [T0; T1] = [force; moment]:
    a, b = axes[0], axes[3]
    c = positions[0] * axes[1] - positions[1] * axes[0]
    d = positions[3] * axes[4] - positions[4] * axes[3]
    determinant = a * d - b * c
    if abs(determinant) < 1e-9:
        raise ValueError('thruster geometry cannot independently control surge and yaw')
    # Cramer's rule for the 2x2 system above.
    forces = [(d * force - b * moment) / determinant, (a * moment - c * force) / determinant]
    # Uniform saturation: the limit depends on the sign of each thrust.
    scale = max(1.0, *(abs(value) / (forward_limits[i] if value >= 0 else reverse_limits[i])
                       for i, value in enumerate(forces)))
    return tuple(value / scale for value in forces)

"""Pure collision checks and differential-thrust guidance helpers.

The input grid must already include the vessel footprint and safety inflation.
Braking/deceleration parameters are reference assumptions until dynamics trials
provide measurements; they are not a guarantee under arbitrary wind/current.
"""

import math


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _local(geometry, point):
    dx, dy = point[0] - geometry.x, point[1] - geometry.y
    c, s = math.cos(geometry.yaw), math.sin(geometry.yaw)
    return ((c * dx + s * dy) / geometry.resolution,
            (-s * dx + c * dy) / geometry.resolution)


def observed_free(geometry, data, cell):
    r, c = cell
    return 0 <= r < geometry.height and 0 <= c < geometry.width and data[r * geometry.width + c] == 0


def segment_is_free(geometry, data, start, end):
    """Supercover traversal: reject unknown cells and diagonal corner cutting."""
    x, y = _local(geometry, start)
    ex, ey = _local(geometry, end)
    col, row = math.floor(x), math.floor(y)
    end_col, end_row = math.floor(ex), math.floor(ey)
    dx, dy = ex - x, ey - y
    sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    tx = ((col + (dx > 0) - x) / dx) if dx else math.inf
    ty = ((row + (dy > 0) - y) / dy) if dy else math.inf
    dtx, dty = (abs(1 / dx) if dx else math.inf), (abs(1 / dy) if dy else math.inf)
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
    """Return a visible tracking target and free length along the chosen corridor."""
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
    for point in path[target_index + 1:]:
        if not segment_is_free(geometry, data, previous, point):
            break
        free_length += math.dist(previous, point)
        previous = point
    return target, free_length


def clearance(geometry, data, position, radius=5.0):
    """Conservative additional distance to unknown/occupied cell boundaries."""
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
    usable = max(0.0, free_distance - margin_m - abs(current_speed) * reaction_s)
    braking = math.sqrt(2.0 * deceleration * usable)
    turn = max_speed * max(0.0, math.cos(heading_error)) ** 2
    near_obstacles = max_speed * min(1.0, max(0.0, clearance_m) / 3.0)
    return min(max_speed, braking, turn, near_obstacles)


def mix_thrusters(force, moment, separation, max_thrust):
    """For thrusters at y=+/-separation/2, yaw moment = (R-L)*separation/2."""
    if not all(math.isfinite(v) for v in (force, moment, separation, max_thrust)) or separation <= 0 or max_thrust <= 0:
        raise ValueError("invalid physical thruster parameters")
    left, right = force / 2 - moment / separation, force / 2 + moment / separation
    scale = max(1.0, abs(left) / max_thrust, abs(right) / max_thrust)
    return left / scale, right / scale


def allocate_thrusters(force, moment, positions, axes, forward_limits, reverse_limits):
    """Solve the physical surge/yaw allocation, then uniformly saturate in N.

    Positions and axes are two flattened xyz vectors in the body frame, referred
    to the same origin as the controller wrench. Reverse limits are magnitudes.
    Any uncommanded sway from canted fixed thrusters remains physical.
    """
    if len(positions) != 6 or len(axes) != 6 or len(forward_limits) != 2 or len(reverse_limits) != 2:
        raise ValueError('allocation requires two physical thrusters')
    if not all(math.isfinite(v) for v in (force, moment, *positions, *axes, *forward_limits, *reverse_limits)):
        raise ValueError('nonfinite thruster allocation')
    if min(*forward_limits, *reverse_limits) <= 0:
        raise ValueError('thruster limits must be positive')
    a, b = axes[0], axes[3]
    c = positions[0] * axes[1] - positions[1] * axes[0]
    d = positions[3] * axes[4] - positions[4] * axes[3]
    determinant = a * d - b * c
    if abs(determinant) < 1e-9:
        raise ValueError('thruster geometry cannot independently control surge and yaw')
    forces = [(d * force - b * moment) / determinant, (a * moment - c * force) / determinant]
    scale = max(1.0, *(abs(value) / (forward_limits[i] if value >= 0 else reverse_limits[i])
                       for i, value in enumerate(forces)))
    return tuple(value / scale for value in forces)

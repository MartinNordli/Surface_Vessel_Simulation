"""Validation of the D* Lite core against an independent A* reference.

The point of these tests is that they check the property that actually matters:
after the robot has moved and new obstacles have been discovered, the
incrementally repaired D* Lite solution must be identical to a from-scratch
optimal search on the updated grid. That is the guarantee an incremental
planner is supposed to give, and it is the one that silently breaks first.

Run: python3 -m pytest tests/ -v      (or: python3 tests/test_dstar_lite.py)
"""

import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "njord_sim"))

from njord_sim.dstar_lite import INF, DStarLite, Grid, astar

TOL = 1e-9


def random_grid(rng, height, width, fill):
    grid = Grid(height, width)
    for r in range(height):
        for c in range(width):
            if rng.random() < fill:
                grid.blocked.add((r, c))
    return grid


def path_is_valid(grid, path, start, goal):
    if not path:
        return False
    if path[0] != start or path[-1] != goal:
        return False
    for cell in path:
        if not grid.is_free(cell):
            return False
    for a, b in zip(path, path[1:]):
        if grid.cost(a, b) == INF:
            return False
    return True


def path_length(grid, path):
    return sum(grid.cost(a, b) for a, b in zip(path, path[1:]))


def test_matches_astar_on_static_grids():
    rng = random.Random(1)
    reachable = 0
    for _ in range(200):
        grid = random_grid(rng, 20, 20, 0.25)
        start, goal = (0, 0), (19, 19)
        grid.blocked.discard(start)
        grid.blocked.discard(goal)

        planner = DStarLite(grid, start, goal)
        planner.compute_shortest_path()
        expected = astar(grid, start, goal)

        assert abs(planner.path_cost() - expected) < TOL or (
            planner.path_cost() == INF and expected == INF
        ), f"D* Lite {planner.path_cost()} vs A* {expected}"

        if expected != INF:
            reachable += 1
            path = planner.path()
            assert path_is_valid(grid, path, start, goal)
            assert abs(path_length(grid, path) - expected) < TOL
    assert reachable > 100, "test grids were almost all unsolvable, weaken the fill rate"
    print(f"  static grids: 200 cases, {reachable} solvable, all match A*")


def test_incremental_replanning_matches_full_replan():
    """The core D* Lite guarantee: move the robot, reveal obstacles, repair."""
    rng = random.Random(7)
    cases = 0
    for trial in range(120):
        grid = random_grid(rng, 25, 25, 0.15)
        start, goal = (0, 0), (24, 24)
        grid.blocked.discard(start)
        grid.blocked.discard(goal)

        planner = DStarLite(grid, start, goal)
        planner.compute_shortest_path()
        if planner.path_cost() == INF:
            continue

        position = start
        for _ in range(6):
            path = planner.path()
            if len(path) < 3:
                break
            # advance the robot two cells along its current plan
            position = path[2]
            planner.update_start(position)

            # reveal a handful of previously unknown obstacles
            changed = []
            for _ in range(8):
                cell = (rng.randrange(25), rng.randrange(25))
                if cell in (position, goal):
                    continue
                if grid.set_blocked(cell, True):
                    changed.append(cell)
            if changed:
                planner.apply_changes(changed)
            planner.compute_shortest_path()

            expected = astar(grid, position, goal)
            got = planner.path_cost()
            assert abs(got - expected) < TOL or (got == INF and expected == INF), (
                f"trial {trial}: incremental {got} vs full replan {expected}"
            )
            cases += 1

            if expected == INF:
                break
            assert path_is_valid(grid, planner.path(), position, goal)
    assert cases > 200, f"only {cases} replan comparisons ran"
    print(f"  incremental replanning: {cases} repair steps, all match a full A* replan")


def test_mixed_block_and_clear_with_moving_start():
    """The case a real costmap produces: the boat moves, obstacles appear, and
    the footprint under the boat is cleared again every cycle.

    This mix is what exposed two float bugs in the priority queue. Both showed
    up as a g value that was too low rather than as a crash, so the planner
    reported a cost it could not actually achieve and path extraction died a
    few cycles later, far from the cause.
    """
    rng = random.Random(11)
    cases = 0
    for trial in range(80):
        grid = random_grid(rng, 30, 30, 0.12)
        start, goal = (0, 0), (29, 29)
        grid.blocked.discard(start)
        grid.blocked.discard(goal)

        planner = DStarLite(grid, start, goal)
        planner.compute_shortest_path()
        if planner.path_cost() == INF:
            continue

        position = start
        for _ in range(8):
            path = planner.path()
            if len(path) < 3:
                break
            position = path[2]
            planner.update_start(position)

            changed = []
            for _ in range(10):
                cell = (rng.randrange(30), rng.randrange(30))
                if cell in (position, goal):
                    continue
                if grid.set_blocked(cell, rng.random() < 0.6):
                    changed.append(cell)
            # clear the footprint under the robot, as a costmap would
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    cell = (position[0] + dr, position[1] + dc)
                    if grid.in_bounds(cell) and grid.set_blocked(cell, False):
                        changed.append(cell)
            if changed:
                planner.apply_changes(changed)
            planner.compute_shortest_path()

            expected = astar(grid, position, goal)
            got = planner.path_cost()
            assert abs(got - expected) < TOL or (got == INF and expected == INF), (
                f"trial {trial}: incremental {got} vs full replan {expected}"
            )
            cases += 1
            if expected == INF:
                break
            assert path_is_valid(grid, planner.path(), position, goal)
    assert cases > 150, f"only {cases} comparisons ran"
    print(f"  block + clear + moving start: {cases} repair steps, all match a full A* replan")


def test_obstacle_removal_is_handled():
    """Clearing a cell must reopen the shortcut through it."""
    grid = Grid(11, 11)
    for c in range(11):
        grid.blocked.add((5, c))
    grid.blocked.discard((5, 10))  # single gap far to the right

    start, goal = (0, 0), (10, 0)
    planner = DStarLite(grid, start, goal)
    planner.compute_shortest_path()
    detour = planner.path_cost()
    assert abs(detour - astar(grid, start, goal)) < TOL

    grid.set_blocked((5, 0), False)  # a much closer gap opens up
    planner.apply_changes([(5, 0)])
    planner.compute_shortest_path()
    direct = planner.path_cost()

    assert abs(direct - astar(grid, start, goal)) < TOL
    assert direct < detour
    print(f"  obstacle removal: cost dropped {detour:.2f} -> {direct:.2f}, matches A*")


def test_unreachable_goal():
    grid = Grid(9, 9)
    for c in range(9):
        grid.blocked.add((4, c))
    planner = DStarLite(grid, (0, 0), (8, 8))
    planner.compute_shortest_path()
    assert planner.path_cost() == INF
    assert planner.path() == []
    print("  walled-off goal: reported unreachable, no path returned")


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            print(name)
            fn()
    print("\nall D* Lite validation checks passed")

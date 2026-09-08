"""D* Lite (Koenig & Likhachev, 2002), optimized version.

Pure Python, no ROS and no Gazebo dependency, so it can be unit tested on its
own. The planner searches backwards from the goal, which is what makes the
incremental replanning cheap when the robot has moved and new obstacles have
appeared.

Grid convention: cells are (row, col) integer tuples. 8-connected. Diagonal
moves are not allowed to cut a corner, i.e. both orthogonally adjacent cells
must be free.
"""

import heapq
import math

SQRT2 = math.sqrt(2.0)
INF = float("inf")

# Costs are sums of 1.0 and sqrt(2), so keys that are equal in exact arithmetic
# routinely differ in the last bit. Two consequences, both of which produce a
# silently too-low g rather than a crash:
#
#   1. The queue terminates one expansion early on such a near-tie.
#   2. A tolerant key comparison disagrees with the heap's exact ordering, so
#      the cell that should be at the top is not the one the loop sees.
#
# Both are fixed by quantising keys to a fixed resolution, which makes exact
# comparison and heap ordering agree. g and rhs are compared with the same
# tolerance but are never heap-ordered, so they only need the helpers.
EPS = 1e-9
_DIGITS = 9


def _greater(a, b):
    """a > b, treating a difference below EPS as equality."""
    if a == INF:
        return b != INF
    if b == INF:
        return False
    return a > b + EPS


def _differs(a, b):
    if a == INF or b == INF:
        return a != b
    return abs(a - b) > EPS

_ORTHO = ((-1, 0), (1, 0), (0, -1), (0, 1))
_DIAG = ((-1, -1), (-1, 1), (1, -1), (1, 1))


class Grid:
    """Binary occupancy grid with 8-connected, non-corner-cutting motion."""

    def __init__(self, height, width):
        self.height = height
        self.width = width
        self.blocked = set()

    def in_bounds(self, cell):
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width

    def is_free(self, cell):
        return self.in_bounds(cell) and cell not in self.blocked

    def set_blocked(self, cell, blocked=True):
        """Returns True if the occupancy of the cell actually changed."""
        was = cell in self.blocked
        if blocked and not was:
            self.blocked.add(cell)
            return True
        if not blocked and was:
            self.blocked.discard(cell)
            return True
        return False

    def neighbors(self, cell):
        r, c = cell
        for dr, dc in _ORTHO:
            n = (r + dr, c + dc)
            if self.is_free(n):
                yield n
        for dr, dc in _DIAG:
            n = (r + dr, c + dc)
            if self.is_free(n) and self.is_free((r + dr, c)) and self.is_free((r, c + dc)):
                yield n

    def cost(self, a, b):
        """Traversal cost of the edge a -> b, INF if the move is not allowed."""
        if not self.is_free(a) or not self.is_free(b):
            return INF
        dr = abs(a[0] - b[0])
        dc = abs(a[1] - b[1])
        if dr > 1 or dc > 1 or (dr == 0 and dc == 0):
            return INF
        if dr == 1 and dc == 1:
            if not self.is_free((a[0] + (b[0] - a[0]), a[1])) or not self.is_free((a[0], a[1] + (b[1] - a[1]))):
                return INF
            return SQRT2
        return 1.0


def heuristic(a, b):
    """Euclidean distance. Admissible and consistent for this cost model."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


class DStarLite:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self._km = 0.0
        self._last_start = start
        self._g = {}
        self._rhs = {}
        self._queue = []
        self._counter = 0
        self._entries = {}  # cell -> [key, counter, cell or None if removed]
        self._rhs[goal] = 0.0
        self._insert(goal, self._key(goal))

    # ------------------------------------------------------------------ #
    # priority queue with lazy deletion
    # ------------------------------------------------------------------ #

    def _insert(self, cell, key):
        self._counter += 1
        entry = [key, self._counter, cell]
        self._entries[cell] = entry
        heapq.heappush(self._queue, entry)

    def _remove(self, cell):
        entry = self._entries.pop(cell, None)
        if entry is not None:
            entry[2] = None

    def _top(self):
        while self._queue:
            key, _, cell = self._queue[0]
            if cell is None:
                heapq.heappop(self._queue)
                continue
            return key, cell
        return None, None

    # ------------------------------------------------------------------ #
    # D* Lite proper
    # ------------------------------------------------------------------ #

    def g(self, cell):
        return self._g.get(cell, INF)

    def rhs(self, cell):
        return self._rhs.get(cell, INF)

    def _key(self, cell):
        m = min(self.g(cell), self.rhs(cell))
        if m == INF:
            return (INF, INF)
        return (round(m + heuristic(self.start, cell) + self._km, _DIGITS), round(m, _DIGITS))

    def _update_vertex(self, cell):
        if cell != self.goal:
            best = INF
            for n in self.grid.neighbors(cell):
                candidate = self.grid.cost(cell, n) + self.g(n)
                if candidate < best:
                    best = candidate
            self._rhs[cell] = best
        self._remove(cell)
        if _differs(self.g(cell), self.rhs(cell)):
            self._insert(cell, self._key(cell))

    def compute_shortest_path(self):
        while True:
            key, cell = self._top()
            if cell is None:
                break
            start_key = self._key(self.start)
            # The start vertex must be locally consistent before we stop. Using
            # "rhs > g" here instead of "rhs != g" leaves an overconsistent
            # start behind whenever it sits at the top of the queue, which
            # yields g(start) = inf even though a path exists.
            if not key < start_key and not _differs(self.rhs(self.start), self.g(self.start)):
                break
            new_key = self._key(cell)
            if key < new_key:
                self._remove(cell)
                self._insert(cell, new_key)
            elif _greater(self.g(cell), self.rhs(cell)):
                self._g[cell] = self.rhs(cell)
                self._remove(cell)
                for n in self.grid.neighbors(cell):
                    self._update_vertex(n)
            else:
                self._g[cell] = INF
                self._update_vertex(cell)
                for n in self.grid.neighbors(cell):
                    self._update_vertex(n)

    def update_start(self, start):
        """Tell the planner the robot has moved. Keeps the search tree valid."""
        if start == self.start:
            return
        self._km += heuristic(self._last_start, start)
        self._last_start = start
        self.start = start

    def apply_changes(self, changed_cells):
        """Re-evaluate vertices affected by occupancy changes.

        Blocking a cell invalidates the edges incident to it and the diagonal
        edges it corner-blocks. Every affected endpoint lies within Chebyshev
        distance 1 of the changed cell, so that neighbourhood is what we touch.
        """
        dirty = set()
        for cell in changed_cells:
            r, c = cell
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    dirty.add((r + dr, c + dc))
        for cell in dirty:
            if self.grid.in_bounds(cell):
                self._update_vertex(cell)

    def path(self, max_steps=100000):
        """Greedy descent of g from start to goal. Empty list if unreachable."""
        if not self.grid.is_free(self.start) or not self.grid.is_free(self.goal):
            return []
        if self.g(self.start) == INF and self.rhs(self.start) == INF:
            return []
        cell = self.start
        result = [cell]
        for _ in range(max_steps):
            if cell == self.goal:
                return result
            best, best_cost = None, INF
            for n in self.grid.neighbors(cell):
                candidate = self.grid.cost(cell, n) + self.g(n)
                if candidate < best_cost:
                    best, best_cost = n, candidate
            if best is None or best_cost == INF:
                return []
            cell = best
            result.append(cell)
        return []

    def path_cost(self):
        if not self.grid.is_free(self.start) or not self.grid.is_free(self.goal):
            return INF
        return self.g(self.start) if self.g(self.start) != INF else self.rhs(self.start)


def astar(grid, start, goal):
    """Reference implementation used to validate D* Lite. Returns cost or INF."""
    if not grid.is_free(start) or not grid.is_free(goal):
        return INF
    open_heap = [(heuristic(start, goal), 0.0, start)]
    best = {start: 0.0}
    closed = set()
    while open_heap:
        _, cost, cell = heapq.heappop(open_heap)
        if cell in closed:
            continue
        if cell == goal:
            return cost
        closed.add(cell)
        for n in grid.neighbors(cell):
            step = grid.cost(cell, n)
            new_cost = cost + step
            if new_cost < best.get(n, INF) - 1e-12:
                best[n] = new_cost
                heapq.heappush(open_heap, (new_cost + heuristic(n, goal), new_cost, n))
    return INF

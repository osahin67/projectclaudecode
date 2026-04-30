from __future__ import annotations
import heapq
from environment.grid import Position
from memory.belief import BeliefState
from memory.map import DroneMemory


# Movement cost per cell type.
# UNKNOWN is given a small cost so the drone prefers known-safe routes but
# will cross unknown space when no known path exists.
# THREAT is expensive but not infinite — the drone avoids danger zones but
# crosses them if there is no other way through.
_CELL_COST: dict[BeliefState, float] = {
    BeliefState.CLEAR:    1.0,
    BeliefState.UNKNOWN:  2.0,
    BeliefState.THREAT:   8.0,
    BeliefState.TARGET:   1.0,
    BeliefState.BASE:     1.0,
    BeliefState.OBSTACLE: float("inf"),
}

# All 8 directions with their movement cost multiplier.
_NEIGHBORS: list[tuple[int, int, float]] = [
    (-1,  0, 1.0), ( 1,  0, 1.0), ( 0, -1, 1.0), ( 0,  1, 1.0),  # cardinal
    (-1, -1, 1.414), ( 1, -1, 1.414), (-1,  1, 1.414), ( 1,  1, 1.414),  # diagonal
]


def heuristic(a: Position, b: Position) -> float:
    # Chebyshev distance — admissible for 8-directional movement.
    return max(abs(a.x - b.x), abs(a.y - b.y))


def astar(
    memory: DroneMemory,
    start: Position,
    goal: Position,
) -> list[Position] | None:
    """
    Find the lowest-cost path from start to goal on the drone's belief map.

    Returns the path as a list of Positions (start excluded, goal included),
    or None if the goal is unreachable.

    Uses A* with Chebyshev heuristic. Obstacles are impassable. Unknown cells
    are traversable at a cost penalty so the drone prefers known routes.
    """
    if start == goal:
        return []

    # Priority queue entries: (f_score, tie_breaker, position)
    counter   = 0
    open_heap: list[tuple[float, int, Position]] = []
    heapq.heappush(open_heap, (0.0, counter, start))

    g_score: dict[tuple[int,int], float] = {(start.x, start.y): 0.0}
    came_from: dict[tuple[int,int], Position] = {}

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        key = (current.x, current.y)

        if current == goal:
            return _reconstruct(came_from, current)

        for dx, dy, move_cost in _NEIGHBORS:
            nb = Position(current.x + dx, current.y + dy)
            if not memory.in_bounds(nb):
                continue

            cell_cost = _CELL_COST[memory.get_belief(nb)]
            if cell_cost == float("inf"):
                continue

            tentative_g = g_score[key] + move_cost * cell_cost
            nb_key = (nb.x, nb.y)

            if tentative_g < g_score.get(nb_key, float("inf")):
                g_score[nb_key]  = tentative_g
                came_from[nb_key] = current
                f = tentative_g + heuristic(nb, goal)
                counter += 1
                heapq.heappush(open_heap, (f, counter, nb))

    return None  # unreachable


def _reconstruct(came_from: dict[tuple[int,int], Position], end: Position) -> list[Position]:
    path: list[Position] = []
    current = end
    while (current.x, current.y) in came_from:
        path.append(current)
        current = came_from[(current.x, current.y)]
    path.reverse()
    return path

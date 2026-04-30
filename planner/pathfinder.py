from __future__ import annotations
import heapq
from dataclasses import dataclass
from environment.grid import Position
from memory.belief import BeliefState
from memory.map import DroneMemory
from constants import DIAGONAL_COST, CARDINAL_COST


# ---------------------------------------------------------------------------
# Cost profiles
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class CostProfile:
    """
    Encodes the drone's risk tolerance as per-cell traversal costs.

    How to read a cost value
    ------------------------
    A cell with cost C is equivalent to C clear cells.  THREAT=15 means
    the drone accepts a detour of up to 14 extra clear steps to avoid one
    threat cell.  Any shorter detour is cheaper than crossing the threat.

    Admissibility
    -------------
    A* is optimal only when h(n) ≤ true_cost(n, goal).  The minimum step
    cost is CARDINAL_COST × clear == 1.0, so the raw Chebyshev heuristic
    stays admissible for all profiles as long as clear == 1.0.
    """
    clear:    float = 1.0
    unknown:  float = 3.0    # penalise unmapped space; cross it only if needed
    threat:   float = 15.0   # strongly avoid; accept ~14-step detour per cell
    target:   float = 1.0
    base:     float = 1.0

    def for_belief(self, belief: BeliefState) -> float:
        match belief:
            case BeliefState.CLEAR:    return self.clear
            case BeliefState.UNKNOWN:  return self.unknown
            case BeliefState.THREAT:   return self.threat
            case BeliefState.TARGET:   return self.target
            case BeliefState.BASE:     return self.base
            case BeliefState.OBSTACLE: return float("inf")


# Named presets — pass to Planner() or astar() to set drone risk tolerance.
BALANCED   = CostProfile(unknown=3.0,  threat=15.0)
CAUTIOUS   = CostProfile(unknown=6.0,  threat=40.0)
AGGRESSIVE = CostProfile(unknown=1.2,  threat=5.0)


# ---------------------------------------------------------------------------
# Neighbour table
# ---------------------------------------------------------------------------

_NEIGHBORS: list[tuple[int, int, float]] = [
    (-1,  0, CARDINAL_COST), ( 1,  0, CARDINAL_COST),
    ( 0, -1, CARDINAL_COST), ( 0,  1, CARDINAL_COST),
    (-1, -1, DIAGONAL_COST), ( 1, -1, DIAGONAL_COST),
    (-1,  1, DIAGONAL_COST), ( 1,  1, DIAGONAL_COST),
]


# ---------------------------------------------------------------------------
# Heuristic
# ---------------------------------------------------------------------------

def heuristic(a: Position, b: Position) -> float:
    """
    Chebyshev distance — admissible for 8-directional movement.
    Tighter than Manhattan for diagonal moves, so A* expands fewer nodes.
    """
    return float(max(abs(a.x - b.x), abs(a.y - b.y)))


# ---------------------------------------------------------------------------
# A*
# ---------------------------------------------------------------------------

def astar(
    memory: DroneMemory,
    start: Position,
    goal: Position,
    profile: CostProfile = BALANCED,
) -> list[Position] | None:
    """
    Return the lowest-cost path from start to goal on the drone's belief map.

    Path is a list of Positions (start excluded, goal included), or None if
    the goal is unreachable under the given profile.

    Cost model
    ----------
    step_cost = base_move_cost × profile.for_belief(cell)
    base_move_cost is CARDINAL_COST for orthogonal steps, DIAGONAL_COST for
    diagonal steps.  Obstacles are always impassable.

    Lazy-deletion visited check
    ---------------------------
    Each heap entry stores the g-score at push time.  On pop, if the stored
    g-score is higher than the best known g-score, the entry is stale
    (a cheaper path was found later) and is skipped.  This avoids maintaining
    a separate closed set while still guaranteeing each node is settled once.
    """
    if start == goal:
        return []

    # Heap entries: (f_score, tie_breaker, g_at_push, position)
    counter = 0
    heap: list[tuple[float, int, float, Position]] = []
    heapq.heappush(heap, (heuristic(start, goal), counter, 0.0, start))

    g: dict[tuple[int, int], float] = {(start.x, start.y): 0.0}
    came_from: dict[tuple[int, int], Position] = {}

    while heap:
        _, _, g_at_push, current = heapq.heappop(heap)
        ck = (current.x, current.y)

        # Lazy deletion: skip if a cheaper route was found after this push.
        if g_at_push > g.get(ck, float("inf")):
            continue

        if current == goal:
            return _reconstruct(came_from, current)

        for dx, dy, move_cost in _NEIGHBORS:
            nb = Position(current.x + dx, current.y + dy)
            if not memory.in_bounds(nb):
                continue

            cell_cost = profile.for_belief(memory.get_belief(nb))
            if cell_cost == float("inf"):
                continue

            tentative_g = g[ck] + move_cost * cell_cost
            nk = (nb.x, nb.y)

            if tentative_g < g.get(nk, float("inf")):
                g[nk] = tentative_g
                came_from[nk] = current
                counter += 1
                f = tentative_g + heuristic(nb, goal)
                heapq.heappush(heap, (f, counter, tentative_g, nb))

    return None


# ---------------------------------------------------------------------------
# Path reconstruction
# ---------------------------------------------------------------------------

def _reconstruct(
    came_from: dict[tuple[int, int], Position],
    end: Position,
) -> list[Position]:
    path: list[Position] = []
    current = end
    while (current.x, current.y) in came_from:
        path.append(current)
        current = came_from[(current.x, current.y)]
    path.reverse()
    return path

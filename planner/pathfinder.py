from __future__ import annotations
import heapq
from dataclasses import dataclass
from environment.grid import Position
from memory.belief import BeliefState
from memory.map import DroneMemory


# ---------------------------------------------------------------------------
# Cost profiles
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class CostProfile:
    """
    Encodes the drone's risk tolerance as a set of per-cell traversal costs.

    How to reason about a cost value
    ---------------------------------
    A cell with cost C is equivalent to C clear cells.  So a drone using
    THREAT=15 will accept a detour of up to 14 extra clear-cell steps to
    avoid one threat cell — any detour shorter than 14 steps is cheaper
    than crossing the threat.

    Heuristic admissibility
    -----------------------
    A* is only guaranteed optimal when h(n) ≤ true_cost(n, goal).
    The minimum possible step cost is 1.0 × clear (one cardinal step on a
    clear cell).  As long as clear == 1.0 across all profiles, the raw
    Chebyshev heuristic stays admissible without scaling.
    """

    # Cost to enter a cell of each belief state.
    clear:    float = 1.0          # baseline — every other cost is relative to this
    unknown:  float = 3.0          # prefer mapped routes; cross unknown only if needed
    threat:   float = 15.0         # avoid strongly; accept ~14-step detour to skip one
    target:   float = 1.0          # goal cell — free to enter
    base:     float = 1.0          # home cell — free to enter

    def for_belief(self, belief: BeliefState) -> float:
        return {
            BeliefState.CLEAR:    self.clear,
            BeliefState.UNKNOWN:  self.unknown,
            BeliefState.THREAT:   self.threat,
            BeliefState.TARGET:   self.target,
            BeliefState.BASE:     self.base,
            BeliefState.OBSTACLE: float("inf"),
        }[belief]


# Preset profiles — passed into astar() or stored on the Planner.

# Standard operation: avoids unknown and threat, not paranoid.
BALANCED = CostProfile(unknown=3.0, threat=15.0)

# Post-damage or low-energy: strongly prefers mapped, safe corridors.
CAUTIOUS = CostProfile(unknown=6.0, threat=40.0)

# Scout mode: treats unknown as nearly free, still avoids hard threats.
AGGRESSIVE = CostProfile(unknown=1.2, threat=5.0)


# ---------------------------------------------------------------------------
# Neighbour table
# ---------------------------------------------------------------------------

# (dx, dy, base_move_cost)  — diagonal costs √2 ≈ 1.414 more than cardinal.
_NEIGHBORS: list[tuple[int, int, float]] = [
    (-1,  0, 1.0), ( 1,  0, 1.0), ( 0, -1, 1.0), ( 0,  1, 1.0),
    (-1, -1, 1.414), ( 1, -1, 1.414), (-1,  1, 1.414), ( 1,  1, 1.414),
]


# ---------------------------------------------------------------------------
# Heuristic
# ---------------------------------------------------------------------------

def heuristic(a: Position, b: Position) -> float:
    """
    Chebyshev distance — admissible for 8-directional movement when
    clear == 1.0 (the minimum possible step cost).

    Chebyshev is tighter than Manhattan for diagonal movement, which means
    A* expands fewer nodes and runs faster.
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
    Find the lowest-cost path from start to goal on the drone's belief map.

    Returns the path as a list of Positions (start excluded, goal included),
    or None if the goal is unreachable under the given profile.

    Cost model
    ----------
    total_step_cost = base_move_cost × profile.for_belief(cell)

    base_move_cost is 1.0 for cardinal steps and √2 for diagonal steps.
    profile.for_belief() maps belief state to a cost multiplier.
    Obstacles are always impassable regardless of profile.
    """
    if start == goal:
        return []

    # (f_score, tie_breaker, position)
    counter = 0
    heap: list[tuple[float, int, Position]] = []
    heapq.heappush(heap, (0.0, counter, start))

    g: dict[tuple[int, int], float] = {(start.x, start.y): 0.0}
    came_from: dict[tuple[int, int], Position] = {}

    while heap:
        _, _, current = heapq.heappop(heap)
        ck = (current.x, current.y)

        if current == goal:
            return _reconstruct(came_from, current)

        # Skip if we already found a cheaper route to this node.
        if g.get(ck, float("inf")) < g.get(ck, float("inf")):
            continue

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
                heapq.heappush(heap, (tentative_g + heuristic(nb, goal), counter, nb))

    return None  # goal unreachable under this profile


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

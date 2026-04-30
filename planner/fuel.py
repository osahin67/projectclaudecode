from __future__ import annotations
from constants import DIAGONAL_COST, CARDINAL_COST
from environment.grid import Position
from drone.drone import Drone
from memory.map import DroneMemory
from .pathfinder import CostProfile, astar


# ---------------------------------------------------------------------------
# Energy-only cost profile
# ---------------------------------------------------------------------------

# For fuel estimation we use only actual movement cost — 1.0 per cardinal
# step, √2 per diagonal — ignoring cell type.  Threat and unknown penalties
# are planning deterrents, not fuel burns.  Using them here would overstate
# the return cost and trigger false-positive abort decisions.
_ENERGY_PROFILE = CostProfile(
    clear=1.0, unknown=1.0, threat=1.0, target=1.0, base=1.0
)


# ---------------------------------------------------------------------------
# Path cost helper
# ---------------------------------------------------------------------------

def _path_cost(start: Position, path: list[Position]) -> float:
    """
    Sum actual movement cost for a path (no cell-type weighting).

    Uses true step cost: DIAGONAL_COST for diagonal moves, CARDINAL_COST
    for orthogonal moves — consistent with how Drone.move() spends energy.
    """
    cost = 0.0
    prev = start
    for pos in path:
        diagonal = abs(pos.x - prev.x) == 1 and abs(pos.y - prev.y) == 1
        cost += DIAGONAL_COST if diagonal else CARDINAL_COST
        prev = pos
    return cost


# ---------------------------------------------------------------------------
# FuelGuard
# ---------------------------------------------------------------------------

class FuelGuard:
    """
    Decides whether the drone has enough fuel to continue its mission.

    Why a fixed ratio is wrong
    --------------------------
    A drone near the map edge needs ~N steps home; one in the centre needs
    ~N/2.  A fixed 30% threshold is too aggressive in some positions and
    too lax in others.  FuelGuard replaces it with an explicit path-cost
    estimate: compute the cheapest physically possible route home via A*,
    multiply by a safety margin, compare to current fuel.

    Safety margin
    -------------
    _ENERGY_PROFILE gives the minimum possible cost (all cells = 1.0).
    The margin of 1.3 covers:
      - newly discovered obstacles that invalidate the estimated path
      - unknown cells that turn out to cost more than a clear step
      - diagonal shortcuts that may not exist at execution time

    Base unknown
    ------------
    If the drone has not yet observed base, the guard falls back to a
    conservative energy ratio until base is discovered.
    """

    SAFETY_MARGIN:      float = 1.3
    FALLBACK_THRESHOLD: float = 0.45

    # ------------------------------------------------------------------

    def must_return(self, drone: Drone, memory: DroneMemory) -> bool:
        """Return True if the drone must head home this tick."""
        base = memory.base_position
        if base is None:
            return drone.energy_ratio < self.FALLBACK_THRESHOLD

        cost = self.return_cost(drone.position, base, memory)
        if cost is None:
            return drone.energy_ratio < self.FALLBACK_THRESHOLD

        return drone.energy <= cost * self.SAFETY_MARGIN

    def can_reach_target_and_return(
        self,
        drone: Drone,
        memory: DroneMemory,
    ) -> bool:
        """
        Return True if the drone has enough fuel to reach the known target
        AND return to base afterward.

        Called before EXPLORING → NAVIGATING.  If False, the drone skips
        the target and returns directly rather than getting stranded.
        """
        target = memory.target_position
        base   = memory.base_position

        if target is None:
            return False

        path_to_target = astar(memory, drone.position, target, _ENERGY_PROFILE)
        if path_to_target is None:
            return False

        cost_to_target = _path_cost(drone.position, path_to_target)

        if base is None:
            return drone.energy >= cost_to_target * self.SAFETY_MARGIN

        path_to_base = astar(memory, target, base, _ENERGY_PROFILE)
        cost_to_base = _path_cost(target, path_to_base) if path_to_base else 0.0

        return drone.energy >= (cost_to_target + cost_to_base) * self.SAFETY_MARGIN

    # ------------------------------------------------------------------

    def return_cost(
        self,
        position: Position,
        base: Position,
        memory: DroneMemory,
    ) -> float | None:
        """
        Minimum fuel needed to travel from position to base.
        Returns None if no path exists in current memory.
        """
        path = astar(memory, position, base, _ENERGY_PROFILE)
        if path is None:
            return None
        return _path_cost(position, path)

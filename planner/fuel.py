from __future__ import annotations
from environment.grid import Position
from drone.drone import Drone
from memory.map import DroneMemory
from .pathfinder import CostProfile, astar


# ------------------------------------------------------------------
# Energy cost profile
# ------------------------------------------------------------------

# For fuel estimation we use actual movement cost only — 1.0 per cardinal
# step, √2 per diagonal — regardless of cell type.  Threat and unknown
# penalties are planning deterrents, not fuel burns.  Using them here
# would overestimate the energy needed to return and trigger early abort.
_ENERGY_PROFILE = CostProfile(
    clear=1.0, unknown=1.0, threat=1.0, target=1.0, base=1.0
)


# ------------------------------------------------------------------
# Path cost helper
# ------------------------------------------------------------------

def _path_cost(start: Position, path: list[Position]) -> float:
    """Sum actual movement cost for a path (no cell-type weighting)."""
    cost = 0.0
    prev = start
    for pos in path:
        diagonal = abs(pos.x - prev.x) == 1 and abs(pos.y - prev.y) == 1
        cost += 1.414 if diagonal else 1.0
        prev = pos
    return cost


# ------------------------------------------------------------------
# FuelGuard
# ------------------------------------------------------------------

class FuelGuard:
    """
    Decides whether the drone has enough fuel to continue its current
    mission phase or must turn back immediately.

    Why a fixed ratio threshold is wrong
    -------------------------------------
    A drone near the map edge needs ~N steps to reach base; one in the
    centre needs ~N/2.  A fixed 30% threshold is simultaneously too
    aggressive in some positions and too lax in others.  The guard
    replaces it with an explicit cost estimate: compute the cheapest
    physically possible path home, multiply by a safety margin, compare
    to current fuel.

    Safety margin
    -------------
    The estimate uses _ENERGY_PROFILE (all cell costs = 1.0), which gives
    the minimum possible energy needed.  The margin covers:
      - newly discovered obstacles that invalidate the estimated path
      - unknown cells that turn out to cost more than expected
      - diagonal shortcuts that may not exist mid-flight
    A margin of 1.3 means the drone starts home when it has 30% more
    fuel than the cheapest known route requires.

    Base unknown
    ------------
    If the drone has never observed base (first few steps), the guard
    falls back to a conservative ratio threshold until base is discovered.
    """

    SAFETY_MARGIN      = 1.3
    FALLBACK_THRESHOLD = 0.45   # used only before base is in memory

    # ------------------------------------------------------------------

    def must_return(self, drone: Drone, memory: DroneMemory) -> bool:
        """
        Return True if the drone must turn for home this tick.

        Called every tick from MissionController.  When True, the
        controller transitions to RETURNING regardless of current phase.
        """
        base = memory.base_position
        if base is None:
            return drone.energy_ratio < self.FALLBACK_THRESHOLD

        cost = self.return_cost(drone.position, base, memory)
        if cost is None:
            # No path to base found in current memory — be very conservative.
            return drone.energy_ratio < self.FALLBACK_THRESHOLD

        return drone.energy <= cost * self.SAFETY_MARGIN

    def can_reach_target_and_return(
        self,
        drone: Drone,
        memory: DroneMemory,
    ) -> bool:
        """
        Return True if the drone has enough fuel to reach the target AND
        return to base afterward.

        Called before EXPLORING → NAVIGATING transition.  If False, the
        drone skips the target and returns home instead of getting stranded
        mid-mission.
        """
        target = memory.target_position
        base   = memory.base_position

        if target is None:
            return False

        path_to_target = astar(memory, drone.position, target, _ENERGY_PROFILE)
        if path_to_target is None:
            return False

        cost_to_target = _path_cost(drone.position, path_to_target)

        # If base is unknown, only check that we can reach the target.
        # The controller will still guard the return leg via must_return().
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
        Estimate the minimum fuel needed to travel from position to base.
        Returns None if no path exists in current memory.
        """
        path = astar(memory, position, base, _ENERGY_PROFILE)
        if path is None:
            return None
        return _path_cost(position, path)

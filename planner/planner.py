from __future__ import annotations
from environment.grid import Position
from drone.mode import Direction, DroneMode
from memory.belief import BeliefState
from memory.map import DroneMemory
from .pathfinder import astar, CostProfile, BALANCED


# Built once at import time — maps every (dx, dy) delta to its Direction.
_DELTA_TO_DIRECTION: dict[tuple[int, int], Direction] = {
    (d.dx, d.dy): d for d in Direction
}


class Planner:
    """
    Converts the drone's current situation into a single Direction to move.

    Owns the active goal and the cached A* path to it.  Replans when:
      - the goal has been reached or is None
      - the mode changes (goal type changes)
      - the next step on the cached path is now an obstacle

    Goal selection by mode
    ----------------------
    EXPLORING  → closest frontier by Manhattan distance, routed via A*
    NAVIGATING → memory.target_position, routed via A*
    RETURNING  → memory.base_position, routed via A*

    Systematic exploration
    ----------------------
    Frontiers are cells on the boundary between known and unknown space.
    Moving toward the nearest frontier guarantees every step expands the
    map — the drone cannot wander in circles on already-known ground.
    """

    def __init__(self, profile: CostProfile = BALANCED) -> None:
        self._profile    = profile
        self._goal:      Position | None  = None
        self._path:      list[Position]   = []
        self._last_mode: DroneMode | None = None

    # ------------------------------------------------------------------
    # Main interface
    # ------------------------------------------------------------------

    def next_move(
        self,
        position: Position,
        mode: DroneMode,
        memory: DroneMemory,
    ) -> Direction | None:
        """Return the next Direction to move, or None if no goal is reachable."""
        if mode != self._last_mode:
            self._clear_plan()
            self._last_mode = mode

        if not self._path or position == self._goal:
            self._plan(position, mode, memory)

        if not self._path:
            return None

        next_pos = self._path[0]

        if memory.get_belief(next_pos) == BeliefState.OBSTACLE:
            self._clear_plan()
            self._plan(position, mode, memory)
            if not self._path:
                return None
            next_pos = self._path[0]

        direction = _DELTA_TO_DIRECTION.get((next_pos.x - position.x, next_pos.y - position.y))
        if direction is None:
            self._clear_plan()
            return None

        self._path.pop(0)
        return direction

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _plan(self, position: Position, mode: DroneMode, memory: DroneMemory) -> None:
        goal = self._select_goal(position, mode, memory)
        if goal is None:
            self._goal = None
            self._path = []
            return

        path = astar(memory, position, goal, self._profile)
        if path is None:
            self._goal = None
            self._path = []
        else:
            self._goal = goal
            self._path = path

    def _select_goal(
        self,
        position: Position,
        mode: DroneMode,
        memory: DroneMemory,
    ) -> Position | None:
        match mode:
            case DroneMode.NAVIGATING: return memory.target_position
            case DroneMode.RETURNING:  return memory.base_position
            case DroneMode.EXPLORING:  return _nearest_frontier(position, memory)
            case _:                    return None

    # ------------------------------------------------------------------

    def _clear_plan(self) -> None:
        self._goal = None
        self._path = []

    @property
    def current_goal(self) -> Position | None:
        return self._goal


# ---------------------------------------------------------------------------
# Frontier selection
# ---------------------------------------------------------------------------

def _nearest_frontier(position: Position, memory: DroneMemory) -> Position | None:
    """
    Return the frontier cell closest to position by Manhattan distance.

    Manhattan is an O(1) proxy that avoids running A* to every frontier
    on every tick.  The route to the chosen frontier is still computed with
    A*, so it correctly routes around obstacles.
    """
    frontiers = memory.frontiers()
    if not frontiers:
        return None
    return min(frontiers, key=lambda f: position.manhattan_distance(f))

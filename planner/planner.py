from __future__ import annotations
from environment.grid import Position
from drone.mode import Direction, DroneMode
from memory.map import DroneMemory
from .pathfinder import astar, CostProfile, BALANCED


# Map (dx, dy) deltas back to Direction enum values for path → move conversion.
_DELTA_TO_DIRECTION: dict[tuple[int,int], Direction] = {
    (d.dx, d.dy): d for d in Direction
}


def _direction_to(frm: Position, to: Position) -> Direction | None:
    delta = (to.x - frm.x, to.y - frm.y)
    return _DELTA_TO_DIRECTION.get(delta)


class Planner:
    """
    Converts the drone's current situation into a single Direction to move.

    Owns the active goal and cached A* path.  Replans automatically when:
      - the current goal has been reached
      - no path exists yet
      - the mode changes (goal type changes)

    Goal selection by mode
    ----------------------
    EXPLORING  → closest frontier by Manhattan distance, navigated via A*
    NAVIGATING → target_position from memory, navigated via A*
    RETURNING  → base_position from memory, navigated via A*

    Why this avoids random wandering
    ---------------------------------
    The drone always has a concrete goal and follows the A* shortest path
    toward it.  Exploration is systematic because frontiers are the
    boundary of the known map — visiting any frontier is guaranteed to
    expand knowledge.  The drone never moves randomly; every step is on
    a path toward a chosen goal.
    """

    def __init__(self, profile: CostProfile = BALANCED) -> None:
        self._profile     = profile
        self._goal:       Position | None       = None
        self._path:       list[Position]        = []
        self._last_mode:  DroneMode | None      = None

    # ------------------------------------------------------------------
    # Main interface
    # ------------------------------------------------------------------

    def next_move(
        self,
        position: Position,
        mode: DroneMode,
        memory: DroneMemory,
    ) -> Direction | None:
        """
        Return the Direction the drone should move this tick, or None if
        the drone is stuck (no reachable goal).
        """
        if mode != self._last_mode:
            self._clear_plan()
            self._last_mode = mode

        # If the current path is stale or exhausted, replan.
        if not self._path or position == self._goal:
            self._plan(position, mode, memory)

        if not self._path:
            return None

        # Advance along the cached path.
        next_pos = self._path[0]

        # If the next step is now an obstacle (newly discovered), replan.
        from memory.belief import BeliefState
        if memory.get_belief(next_pos) == BeliefState.OBSTACLE:
            self._clear_plan()
            self._plan(position, mode, memory)
            if not self._path:
                return None
            next_pos = self._path[0]

        direction = _direction_to(position, next_pos)
        if direction is None:
            # Path is broken (shouldn't happen, but be safe).
            self._clear_plan()
            return None

        self._path.pop(0)
        return direction

    # ------------------------------------------------------------------
    # Goal selection
    # ------------------------------------------------------------------

    def _plan(self, position: Position, mode: DroneMode, memory: DroneMemory) -> None:
        goal = self._select_goal(position, mode, memory)
        if goal is None:
            self._goal = None
            self._path = []
            return

        path = astar(memory, position, goal, self._profile)
        if path is None:
            # Goal unreachable — drop it so next tick tries a different one.
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
        if mode == DroneMode.NAVIGATING:
            return memory.target_position

        if mode == DroneMode.RETURNING:
            return memory.base_position

        if mode == DroneMode.EXPLORING:
            return self._closest_frontier(position, memory)

        return None

    def _closest_frontier(
        self,
        position: Position,
        memory: DroneMemory,
    ) -> Position | None:
        """
        Pick the frontier cell nearest to the drone by Manhattan distance.

        Manhattan distance is a cheap proxy — it avoids running A* to every
        frontier on every tick while still pointing the drone toward nearby
        unexplored space.  The actual path to the chosen frontier is still
        computed with A*, so the route respects obstacles.
        """
        frontiers = memory.frontiers()
        if not frontiers:
            return None
        return min(frontiers, key=lambda f: position.manhattan_distance(f))

    # ------------------------------------------------------------------

    def _clear_plan(self) -> None:
        self._goal = None
        self._path = []

    @property
    def current_goal(self) -> Position | None:
        return self._goal

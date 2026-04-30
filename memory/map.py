from __future__ import annotations
from environment.grid import Grid, Position, ObservedCell
from .belief import BeliefState, CellRecord, CELLTYPE_TO_BELIEF
from .sensor import Sensor


class DroneMemory:
    """
    The drone's internal occupancy grid — its only model of the world.

    Starts entirely UNKNOWN.  Cells transition to known states as the
    sensor sweeps them.  The drone plans exclusively from this map;
    it never inspects the ground-truth Grid directly.

    In real robotics this is called an occupancy grid (Moravec & Elfes, 1985).
    ROS (Robot Operating System) exposes this as nav_msgs/OccupancyGrid.
    """

    def __init__(self, width: int, height: int) -> None:
        self.width  = width
        self.height = height
        self._grid: list[list[CellRecord]] = [
            [CellRecord() for _ in range(width)]
            for _ in range(height)
        ]
        self._step = 0

        # Cached special positions — set the first time they are observed.
        self._target_pos: Position | None = None
        self._base_pos:   Position | None = None

    # ------------------------------------------------------------------
    # Updating from sensor observations
    # ------------------------------------------------------------------

    def update(self, observations: list[ObservedCell]) -> None:
        """
        Ingest a batch of sensor readings and advance the internal clock.

        Called once per simulator tick after the sensor has swept the
        drone's surroundings.
        """
        self._step += 1
        for obs in observations:
            self._apply(obs)

    def _apply(self, obs: ObservedCell) -> None:
        record = self._record(obs.position)
        record.belief      = CELLTYPE_TO_BELIEF[obs.cell_type]
        record.visit_count += 1
        record.last_seen   = self._step

        if record.belief == BeliefState.TARGET and self._target_pos is None:
            self._target_pos = obs.position
        if record.belief == BeliefState.BASE and self._base_pos is None:
            self._base_pos = obs.position

    # ------------------------------------------------------------------
    # Querying
    # ------------------------------------------------------------------

    def get_belief(self, pos: Position) -> BeliefState:
        return self._record(pos).belief

    def get_record(self, pos: Position) -> CellRecord:
        return self._record(pos)

    def in_bounds(self, pos: Position) -> bool:
        return 0 <= pos.x < self.width and 0 <= pos.y < self.height

    @property
    def target_position(self) -> Position | None:
        return self._target_pos

    @property
    def base_position(self) -> Position | None:
        return self._base_pos

    @property
    def step(self) -> int:
        return self._step

    # ------------------------------------------------------------------
    # Frontier detection
    # ------------------------------------------------------------------

    def frontiers(self) -> list[Position]:
        """
        Return all UNKNOWN cells that are adjacent to at least one known,
        passable cell.

        This is the core of frontier-based exploration (Yamauchi, 1997):
        the drone should move toward these cells to expand its knowledge.
        Any position on this list is a candidate next goal for the Planner.
        """
        result: list[Position] = []
        for y in range(self.height):
            for x in range(self.width):
                pos = Position(x, y)
                if self._record(pos).belief != BeliefState.UNKNOWN:
                    continue
                if self._has_known_passable_neighbour(pos):
                    result.append(pos)
        return result

    def _has_known_passable_neighbour(self, pos: Position) -> bool:
        for dx, dy in ((-1,0),(1,0),(0,-1),(0,1)):
            nb = Position(pos.x + dx, pos.y + dy)
            if not self.in_bounds(nb):
                continue
            belief = self._record(nb).belief
            if belief.is_known() and belief.is_passable():
                return True
        return False

    # ------------------------------------------------------------------
    # Coverage statistics
    # ------------------------------------------------------------------

    def coverage(self) -> float:
        """Fraction of cells that have been observed at least once."""
        total  = self.width * self.height
        known  = sum(
            1
            for y in range(self.height)
            for x in range(self.width)
            if self._grid[y][x].belief.is_known()
        )
        return known / total

    # ------------------------------------------------------------------
    # Debug rendering
    # ------------------------------------------------------------------

    _SYMBOLS = {
        BeliefState.UNKNOWN:  "?",
        BeliefState.CLEAR:    ".",
        BeliefState.OBSTACLE: "#",
        BeliefState.THREAT:   "!",
        BeliefState.TARGET:   "T",
        BeliefState.BASE:     "B",
    }

    def render(self, drone_pos: Position | None = None) -> str:
        rows = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                pos = Position(x, y)
                if drone_pos and pos.x == drone_pos.x and pos.y == drone_pos.y:
                    row.append("D")
                else:
                    row.append(self._SYMBOLS[self._grid[y][x].belief])
            rows.append(" ".join(row))
        return "\n".join(rows)

    # ------------------------------------------------------------------

    def _record(self, pos: Position) -> CellRecord:
        if not self.in_bounds(pos):
            raise IndexError(f"{pos} is out of bounds ({self.width}x{self.height})")
        return self._grid[pos.y][pos.x]

    # ------------------------------------------------------------------
    # Factory
    # ------------------------------------------------------------------

    @classmethod
    def from_grid(cls, grid: Grid) -> DroneMemory:
        """Convenience constructor that copies dimensions from a Grid."""
        return cls(grid.width, grid.height)

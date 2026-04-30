from __future__ import annotations
from dataclasses import dataclass, field
from environment.grid import Grid, Position
from .mode import Direction, DroneMode


@dataclass
class MoveResult:
    success: bool
    position: Position
    reason: str = ""


class Drone:
    """
    Represents the drone's physical state.

    Owns position, mode, energy, and movement history.
    Has no knowledge of obstacles or the map — the Planner is
    responsible for sending only valid move commands.
    """

    # Energy consumed per step (cardinal vs diagonal)
    _CARDINAL_COST  = 1.0
    _DIAGONAL_COST  = 1.414  # ≈ √2

    def __init__(
        self,
        start: Position,
        grid: Grid,
        max_energy: float = 200.0,
    ) -> None:
        # position: where the drone currently is
        self._position  = start
        # grid: needed only for bounds checking on move; drone is not map-aware
        self._grid      = grid
        # mode: drives what the Planner should compute next
        self._mode      = DroneMode.IDLE
        # energy: finite resource; planner uses it to decide when to return
        self._energy    = max_energy
        self._max_energy = max_energy
        # heading: last direction moved; useful for sensor FOV later
        self._heading: Direction | None = None
        # history: ordered list of every position visited
        self._history: list[Position] = [start]
        # steps: total moves attempted (including failed ones)
        self._steps     = 0

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    def move(self, direction: Direction) -> MoveResult:
        """
        Attempt one step in `direction`.
        Returns a MoveResult so the caller knows whether the move succeeded
        without needing to compare positions before and after.
        """
        if self._mode == DroneMode.CRASHED:
            return MoveResult(False, self._position, "drone is crashed")

        if self._energy <= 0:
            return MoveResult(False, self._position, "out of energy")

        candidate = Position(
            self._position.x + direction.dx,
            self._position.y + direction.dy,
        )

        if not self._grid.in_bounds(candidate):
            self._steps += 1
            return MoveResult(False, self._position, "out of bounds")

        cost = self._DIAGONAL_COST if direction.dx != 0 and direction.dy != 0 \
               else self._CARDINAL_COST

        self._position  = candidate
        self._heading   = direction
        self._energy    = max(0.0, self._energy - cost)
        self._history.append(candidate)
        self._steps    += 1

        return MoveResult(True, self._position)

    # ------------------------------------------------------------------
    # Mode transitions
    # ------------------------------------------------------------------

    def set_mode(self, mode: DroneMode) -> None:
        if self._mode == DroneMode.CRASHED:
            raise RuntimeError("Cannot change mode of a crashed drone")
        self._mode = mode

    def crash(self) -> None:
        """Terminal state — called by the simulator when the drone hits an obstacle."""
        self._mode = DroneMode.CRASHED

    # ------------------------------------------------------------------
    # Read-only state (other modules read these, never write directly)
    # ------------------------------------------------------------------

    @property
    def position(self) -> Position:
        return self._position

    @property
    def mode(self) -> DroneMode:
        return self._mode

    @property
    def heading(self) -> Direction | None:
        return self._heading

    @property
    def energy(self) -> float:
        return self._energy

    @property
    def energy_ratio(self) -> float:
        return self._energy / self._max_energy

    @property
    def steps(self) -> int:
        return self._steps

    @property
    def history(self) -> list[Position]:
        return list(self._history)  # copy — callers cannot mutate internal history

    @property
    def is_alive(self) -> bool:
        return self._mode != DroneMode.CRASHED and self._energy > 0

    # ------------------------------------------------------------------
    # Debug
    # ------------------------------------------------------------------

    def status(self, return_cost: float | None = None) -> str:
        fuel_str = ""
        if return_cost is not None:
            margin = self._energy - return_cost
            fuel_str = f"  return≈{return_cost:.1f}  margin={margin:+.1f}"
        return (
            f"pos=({self._position.x},{self._position.y})  "
            f"mode={self._mode.name:<11} "
            f"energy={self._energy:6.1f}/{self._max_energy:.0f}"
            f"{fuel_str}  steps={self._steps}"
        )

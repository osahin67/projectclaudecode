from __future__ import annotations
from dataclasses import dataclass
from constants import DIAGONAL_COST, CARDINAL_COST
from environment.grid import Grid, Position
from .mode import Direction, DroneMode


@dataclass
class MoveResult:
    success:  bool
    position: Position
    reason:   str = ""


class Drone:
    """
    The drone's physical state.

    Owns position, mode, energy, heading, and movement history.
    Has no knowledge of map contents — the Planner sends only valid moves
    and is solely responsible for obstacle avoidance.
    """

    def __init__(
        self,
        start: Position,
        grid: Grid,
        max_energy: float = 200.0,
    ) -> None:
        if max_energy <= 0:
            raise ValueError("max_energy must be positive")
        self._position   = start
        self._grid       = grid          # used only for in_bounds checks
        self._mode       = DroneMode.IDLE
        self._energy     = max_energy
        self._max_energy = max_energy
        self._heading:   Direction | None      = None
        self._history:   list[Position]        = [start]
        self._steps      = 0

    # ------------------------------------------------------------------
    # Movement
    # ------------------------------------------------------------------

    def move(self, direction: Direction) -> MoveResult:
        """
        Attempt one step in direction.
        Returns MoveResult so callers get structured feedback without
        needing to compare positions before and after.
        """
        if self._mode == DroneMode.CRASHED:
            return MoveResult(False, self._position, "drone is crashed")
        if self._energy <= 0:
            return MoveResult(False, self._position, "out of energy")

        candidate = Position(self._position.x + direction.dx,
                             self._position.y + direction.dy)

        if not self._grid.in_bounds(candidate):
            self._steps += 1
            return MoveResult(False, self._position, "out of bounds")

        cost = (DIAGONAL_COST if direction.dx != 0 and direction.dy != 0
                else CARDINAL_COST)

        self._position = candidate
        self._heading  = direction
        self._energy   = max(0.0, self._energy - cost)
        self._history.append(candidate)
        self._steps   += 1

        return MoveResult(True, self._position)

    # ------------------------------------------------------------------
    # Mode transitions
    # ------------------------------------------------------------------

    def set_mode(self, mode: DroneMode) -> None:
        if self._mode == DroneMode.CRASHED:
            raise RuntimeError("Cannot change mode of a crashed drone")
        self._mode = mode

    def crash(self) -> None:
        """Terminal state — called by the simulator on obstacle collision."""
        self._mode = DroneMode.CRASHED

    # ------------------------------------------------------------------
    # Read-only state
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
        return list(self._history)

    @property
    def is_alive(self) -> bool:
        return self._mode != DroneMode.CRASHED and self._energy > 0

    # ------------------------------------------------------------------
    # Debug
    # ------------------------------------------------------------------

    def status(self, return_cost: float | None = None) -> str:
        fuel = ""
        if return_cost is not None:
            fuel = f"  return≈{return_cost:.1f}  margin={self._energy - return_cost:+.1f}"
        return (
            f"pos=({self._position.x},{self._position.y})  "
            f"mode={self._mode.name:<11} "
            f"energy={self._energy:6.1f}/{self._max_energy:.0f}"
            f"{fuel}  steps={self._steps}"
        )

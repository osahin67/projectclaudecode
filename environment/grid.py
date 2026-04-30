from __future__ import annotations
from dataclasses import dataclass, field
from typing import Iterator
from .cell import CellType


@dataclass
class Position:
    x: int
    y: int

    def __add__(self, other: Position) -> Position:
        return Position(self.x + other.x, self.y + other.y)

    def manhattan_distance(self, other: Position) -> int:
        return abs(self.x - other.x) + abs(self.y - other.y)


@dataclass
class ObservedCell:
    position: Position
    cell_type: CellType


class Grid:
    def __init__(self, width: int, height: int) -> None:
        if width < 3 or height < 3:
            raise ValueError("Grid must be at least 3x3")
        self.width = width
        self.height = height
        self._cells: list[list[CellType]] = [
            [CellType.EMPTY] * width for _ in range(height)
        ]

    # ------------------------------------------------------------------
    # Cell access
    # ------------------------------------------------------------------

    def get(self, pos: Position) -> CellType:
        self._require_in_bounds(pos)
        return self._cells[pos.y][pos.x]

    def set(self, pos: Position, cell_type: CellType) -> None:
        self._require_in_bounds(pos)
        self._cells[pos.y][pos.x] = cell_type

    def in_bounds(self, pos: Position) -> bool:
        return 0 <= pos.x < self.width and 0 <= pos.y < self.height

    # ------------------------------------------------------------------
    # Sensor query — the only interface the Sensor module calls
    # ------------------------------------------------------------------

    def query_observable(self, origin: Position, radius: int) -> list[ObservedCell]:
        """Return all cells within Chebyshev distance `radius` of `origin`."""
        results: list[ObservedCell] = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                pos = Position(origin.x + dx, origin.y + dy)
                if self.in_bounds(pos):
                    results.append(ObservedCell(pos, self.get(pos)))
        return results

    # ------------------------------------------------------------------
    # Iteration helpers
    # ------------------------------------------------------------------

    def all_positions(self) -> Iterator[Position]:
        for y in range(self.height):
            for x in range(self.width):
                yield Position(x, y)

    def find(self, cell_type: CellType) -> list[Position]:
        return [p for p in self.all_positions() if self.get(p) == cell_type]

    # ------------------------------------------------------------------
    # Debug rendering
    # ------------------------------------------------------------------

    _SYMBOLS = {
        CellType.EMPTY:    ".",
        CellType.OBSTACLE: "#",
        CellType.THREAT:   "!",
        CellType.TARGET:   "T",
        CellType.BASE:     "B",
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
                    row.append(self._SYMBOLS[self._cells[y][x]])
            rows.append(" ".join(row))
        return "\n".join(rows)

    # ------------------------------------------------------------------

    def _require_in_bounds(self, pos: Position) -> None:
        if not self.in_bounds(pos):
            raise IndexError(f"Position {pos} is out of bounds ({self.width}x{self.height})")

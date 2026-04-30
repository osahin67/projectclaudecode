from __future__ import annotations
import random
from .cell import CellType
from .grid import Grid, Position


def random_map(
    width: int,
    height: int,
    obstacle_density: float = 0.15,
    threat_density: float = 0.08,
    seed: int | None = None,
) -> Grid:
    """Generate a random map with guaranteed-clear base and target cells."""
    if not (0.0 <= obstacle_density <= 1.0) or not (0.0 <= threat_density <= 1.0):
        raise ValueError("Densities must be between 0.0 and 1.0")
    if obstacle_density + threat_density > 0.6:
        raise ValueError("Combined density too high — map may be unsolvable")

    rng = random.Random(seed)
    grid = Grid(width, height)

    base_pos = Position(1, 1)
    target_pos = Position(width - 2, height - 2)

    protected = {(base_pos.x, base_pos.y), (target_pos.x, target_pos.y)}

    for pos in grid.all_positions():
        key = (pos.x, pos.y)
        if key in protected:
            continue
        roll = rng.random()
        if roll < obstacle_density:
            grid.set(pos, CellType.OBSTACLE)
        elif roll < obstacle_density + threat_density:
            grid.set(pos, CellType.THREAT)

    grid.set(base_pos, CellType.BASE)
    grid.set(target_pos, CellType.TARGET)

    return grid


class ManualMapBuilder:
    """Fluent builder for hand-crafting maps in tests or scenarios."""

    def __init__(self, width: int, height: int) -> None:
        self._grid = Grid(width, height)

    def obstacle(self, x: int, y: int) -> ManualMapBuilder:
        self._grid.set(Position(x, y), CellType.OBSTACLE)
        return self

    def threat(self, x: int, y: int) -> ManualMapBuilder:
        self._grid.set(Position(x, y), CellType.THREAT)
        return self

    def base(self, x: int, y: int) -> ManualMapBuilder:
        self._grid.set(Position(x, y), CellType.BASE)
        return self

    def target(self, x: int, y: int) -> ManualMapBuilder:
        self._grid.set(Position(x, y), CellType.TARGET)
        return self

    def row(self, y: int, pattern: str) -> ManualMapBuilder:
        """
        Place an entire row from a compact string.
        Characters: '.' empty  '#' obstacle  '!' threat  'T' target  'B' base
        """
        char_map = {
            ".": CellType.EMPTY,
            "#": CellType.OBSTACLE,
            "!": CellType.THREAT,
            "T": CellType.TARGET,
            "B": CellType.BASE,
        }
        for x, ch in enumerate(pattern.replace(" ", "")):
            if ch not in char_map:
                raise ValueError(f"Unknown character '{ch}' in pattern")
            self._grid.set(Position(x, y), char_map[ch])
        return self

    def build(self) -> Grid:
        return self._grid

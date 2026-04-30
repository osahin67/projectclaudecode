from __future__ import annotations
import random
from environment.grid import Grid, Position, ObservedCell
from environment.cell import CellType


# Cells the sensor can misread, and what it might report instead.
# An obstacle is never misread as clear — that would be a safety violation.
_NOISE_TABLE: dict[CellType, list[CellType]] = {
    CellType.EMPTY:  [CellType.THREAT],
    CellType.THREAT: [CellType.EMPTY],
}


class Sensor:
    """
    Translates ground-truth Environment observations into potentially
    imperfect readings that the drone's memory can ingest.

    In real robotics this layer models:
      - limited range (LiDAR, sonar, camera FOV)
      - sensor noise (false positives, missed detections)
      - occlusion (not implemented here, but the interface supports it)

    The drone never calls grid.get() directly — all information about the
    world arrives through here, enforcing the information barrier.
    """

    def __init__(self, vision_radius: int, noise_chance: float = 0.0) -> None:
        if vision_radius < 1:
            raise ValueError("vision_radius must be at least 1")
        if not (0.0 <= noise_chance < 1.0):
            raise ValueError("noise_chance must be in [0.0, 1.0)")

        self.vision_radius = vision_radius
        self.noise_chance  = noise_chance
        self._rng          = random.Random()

    def seed(self, value: int) -> None:
        self._rng.seed(value)

    def observe(self, grid: Grid, position: Position) -> list[ObservedCell]:
        """
        Return sensor readings for all cells within vision_radius of position.

        Each reading is either accurate (noise_chance == 0) or possibly
        corrupted according to _NOISE_TABLE.
        """
        raw = grid.query_observable(position, self.vision_radius)

        if self.noise_chance == 0.0:
            return raw

        results: list[ObservedCell] = []
        for obs in raw:
            cell_type = obs.cell_type
            if cell_type in _NOISE_TABLE and self._rng.random() < self.noise_chance:
                cell_type = self._rng.choice(_NOISE_TABLE[cell_type])
            results.append(ObservedCell(obs.position, cell_type))
        return results

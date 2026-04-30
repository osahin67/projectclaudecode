from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
from environment.cell import CellType


class BeliefState(Enum):
    """
    The drone's belief about a single cell.

    Mirrors CellType but adds UNKNOWN — the drone starts knowing nothing,
    and only updates cells it has directly observed.  This separation matters:
    CellType is ground truth; BeliefState is the drone's internal model.
    """
    UNKNOWN  = auto()   # never observed
    CLEAR    = auto()   # observed and passable
    OBSTACLE = auto()   # observed and impassable
    THREAT   = auto()   # observed and dangerous (passable but costly)
    TARGET   = auto()   # observed target location
    BASE     = auto()   # observed base location

    def is_passable(self) -> bool:
        # UNKNOWN is treated as potentially passable during exploration
        return self not in (BeliefState.OBSTACLE,)

    def is_known(self) -> bool:
        return self != BeliefState.UNKNOWN


# Maps ground-truth CellType → the BeliefState a perfect sensor would record.
# Noisy sensors may produce a different BeliefState — see sensor.py.
CELLTYPE_TO_BELIEF: dict[CellType, BeliefState] = {
    CellType.EMPTY:    BeliefState.CLEAR,
    CellType.OBSTACLE: BeliefState.OBSTACLE,
    CellType.THREAT:   BeliefState.THREAT,
    CellType.TARGET:   BeliefState.TARGET,
    CellType.BASE:     BeliefState.BASE,
}


@dataclass
class CellRecord:
    """
    Everything the drone remembers about one cell.

    - belief:      current best guess about cell type
    - visit_count: how many times the sensor has swept this cell
    - last_seen:   simulator step when it was last observed (None = never)
    """
    belief:      BeliefState = BeliefState.UNKNOWN
    visit_count: int         = 0
    last_seen:   int | None  = None

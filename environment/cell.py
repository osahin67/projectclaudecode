from enum import Enum, auto


class CellType(Enum):
    EMPTY = auto()
    OBSTACLE = auto()
    THREAT = auto()
    TARGET = auto()
    BASE = auto()

    def is_passable(self) -> bool:
        return self not in (CellType.OBSTACLE,)

    def is_dangerous(self) -> bool:
        return self == CellType.THREAT

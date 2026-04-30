from enum import Enum, auto


class DroneMode(Enum):
    EXPLORING  = auto()   # building the map; no known target yet
    NAVIGATING = auto()   # target located; moving toward it
    RETURNING  = auto()   # target reached; heading back to base
    IDLE       = auto()   # waiting for a command (start state)
    CRASHED    = auto()   # hit an obstacle; terminal state


class Direction(Enum):
    NORTH      = ( 0, -1)
    SOUTH      = ( 0,  1)
    EAST       = ( 1,  0)
    WEST       = (-1,  0)
    NORTH_EAST = ( 1, -1)
    NORTH_WEST = (-1, -1)
    SOUTH_EAST = ( 1,  1)
    SOUTH_WEST = (-1,  1)

    @property
    def dx(self) -> int:
        return self.value[0]

    @property
    def dy(self) -> int:
        return self.value[1]

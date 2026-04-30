"""
Shared physical constants used across simulation modules.

Keeping movement costs in one place ensures drone.py, pathfinder.py,
and fuel.py all agree on what one step costs.
"""
import math

DIAGONAL_COST: float = math.sqrt(2)   # cost of one diagonal step  ≈ 1.414
CARDINAL_COST: float = 1.0            # cost of one cardinal step

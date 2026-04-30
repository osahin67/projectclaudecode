from __future__ import annotations
from dataclasses import dataclass, field
from drone.drone import Drone
from drone.mode import DroneMode
from memory.map import DroneMemory


# The drone starts returning home when energy drops below this fraction,
# regardless of whether it has found the target.  Set conservatively so the
# drone can still reach base from anywhere on the map.
_ENERGY_RETURN_THRESHOLD = 0.30


@dataclass
class MissionStatus:
    complete: bool  = False
    success:  bool  = False
    reason:   str   = ""


class MissionController:
    """
    Top-level phase state machine.  Evaluates transition conditions every tick
    and issues mode changes to the drone.

    Transition table
    ----------------
    IDLE       → EXPLORING  : always, at first tick
    EXPLORING  → NAVIGATING : target seen in memory
    EXPLORING  → RETURNING  : energy below threshold (target never found)
    NAVIGATING → RETURNING  : drone is standing on target cell
    NAVIGATING → RETURNING  : energy below threshold mid-navigation
    RETURNING  → complete   : drone is standing on base cell

    The controller never touches the map or pathfinder — it only reads
    high-level facts (mode, position, energy, what memory knows) and
    issues set_mode() calls.  This keeps transitions legible and testable
    in isolation.
    """

    def __init__(self) -> None:
        self._status = MissionStatus()

    def tick(self, drone: Drone, memory: DroneMemory) -> MissionStatus:
        if self._status.complete:
            return self._status

        mode     = drone.mode
        position = drone.position

        # ── IDLE → EXPLORING ────────────────────────────────────────────
        if mode == DroneMode.IDLE:
            drone.set_mode(DroneMode.EXPLORING)
            return self._status

        # ── Terminal: energy exhausted before reaching base ──────────────
        if not drone.is_alive:
            return self._finish(False, "out of energy before reaching base")

        # ── EXPLORING ───────────────────────────────────────────────────
        if mode == DroneMode.EXPLORING:
            if memory.target_position is not None:
                drone.set_mode(DroneMode.NAVIGATING)
            elif drone.energy_ratio < _ENERGY_RETURN_THRESHOLD:
                drone.set_mode(DroneMode.RETURNING)

        # ── NAVIGATING ──────────────────────────────────────────────────
        elif mode == DroneMode.NAVIGATING:
            target = memory.target_position
            if target and position == target:
                drone.set_mode(DroneMode.RETURNING)
            elif drone.energy_ratio < _ENERGY_RETURN_THRESHOLD:
                drone.set_mode(DroneMode.RETURNING)

        # ── RETURNING ───────────────────────────────────────────────────
        elif mode == DroneMode.RETURNING:
            base = memory.base_position
            if base and position == base:
                return self._finish(True, "reached base")

        return self._status

    def _finish(self, success: bool, reason: str) -> MissionStatus:
        self._status.complete = True
        self._status.success  = success
        self._status.reason   = reason
        return self._status

    @property
    def status(self) -> MissionStatus:
        return self._status

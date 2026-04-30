from __future__ import annotations
from dataclasses import dataclass
from drone.drone import Drone
from drone.mode import DroneMode
from memory.map import DroneMemory
from .fuel import FuelGuard


@dataclass
class MissionStatus:
    complete: bool = False
    success:  bool = False
    reason:   str  = ""


class MissionController:
    """
    Top-level phase state machine.

    Transition table
    ----------------
    IDLE       → EXPLORING  : always, at first tick
    EXPLORING  → NAVIGATING : target in memory AND FuelGuard confirms
                              enough fuel to reach target + return to base
    EXPLORING  → RETURNING  : FuelGuard says turn back now
    NAVIGATING → RETURNING  : drone reached target cell
    NAVIGATING → RETURNING  : FuelGuard says turn back now
    RETURNING  → complete   : drone reached base cell
    any        → failure    : drone is no longer alive (energy == 0)

    The fuel check uses an actual A* path estimate (via FuelGuard) rather
    than a fixed energy ratio.  This means the drone turns back based on
    distance to base, not a blind percentage.
    """

    def __init__(self, fuel_guard: FuelGuard | None = None) -> None:
        self._fuel   = fuel_guard or FuelGuard()
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

        # ── Terminal: energy exhausted ───────────────────────────────────
        if not drone.is_alive:
            return self._finish(False, "out of energy before reaching base")

        # ── Fuel check (runs every tick in active phases) ────────────────
        # Check before phase-specific logic so an emergency return can
        # override even a freshly triggered NAVIGATING transition.
        if mode in (DroneMode.EXPLORING, DroneMode.NAVIGATING):
            if self._fuel.must_return(drone, memory):
                drone.set_mode(DroneMode.RETURNING)
                return self._status

        # ── EXPLORING ───────────────────────────────────────────────────
        if mode == DroneMode.EXPLORING:
            if memory.target_position is not None:
                if self._fuel.can_reach_target_and_return(drone, memory):
                    drone.set_mode(DroneMode.NAVIGATING)
                else:
                    # Target found but not enough fuel — head home.
                    drone.set_mode(DroneMode.RETURNING)

        # ── NAVIGATING ──────────────────────────────────────────────────
        elif mode == DroneMode.NAVIGATING:
            target = memory.target_position
            if target and position == target:
                drone.set_mode(DroneMode.RETURNING)

        # ── RETURNING ───────────────────────────────────────────────────
        elif mode == DroneMode.RETURNING:
            base = memory.base_position
            if base and position == base:
                return self._finish(True, "reached base")

        return self._status

    # ------------------------------------------------------------------

    def _finish(self, success: bool, reason: str) -> MissionStatus:
        self._status.complete = True
        self._status.success  = success
        self._status.reason   = reason
        return self._status

    @property
    def status(self) -> MissionStatus:
        return self._status

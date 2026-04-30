"""
Drone simulator — visual runner.

Controls
--------
  SPACE       pause / resume
  R           restart with the same seed
  +  /  -     speed up / slow down
  Q  / ESC    quit
"""
from __future__ import annotations
import sys
from dataclasses import dataclass
import pygame

import config
from environment import random_map, Position
from drone import Drone, DroneMode
from memory import DroneMemory, Sensor
from planner import Planner, MissionController, FuelGuard, BALANCED, MissionStatus
from visualization import Renderer
from visualization import colors


# ---------------------------------------------------------------------------
# Simulation container
# ---------------------------------------------------------------------------

@dataclass
class Simulation:
    """Holds every object that makes up one running mission."""
    grid:       object          # Grid — typed as object to avoid circular import at top
    drone:      Drone
    memory:     DroneMemory
    sensor:     Sensor
    planner:    Planner
    guard:      FuelGuard
    controller: MissionController


def build_simulation(seed: int) -> Simulation:
    from environment.grid import Grid   # local import avoids re-exporting Grid publicly
    grid = random_map(
        config.GRID_W, config.GRID_H,
        seed=seed,
        obstacle_density=config.OBSTACLE_DENSITY,
        threat_density=config.THREAT_DENSITY,
    )
    drone      = Drone(Position(1, 1), grid, max_energy=config.MAX_ENERGY)
    memory     = DroneMemory.from_grid(grid)
    sensor     = Sensor(vision_radius=config.VISION_RADIUS)
    planner    = Planner(profile=BALANCED)
    guard      = FuelGuard()
    controller = MissionController(fuel_guard=guard)
    return Simulation(grid, drone, memory, sensor, planner, guard, controller)


def step(sim: Simulation) -> MissionStatus:
    """Advance simulation by one tick. Returns current MissionStatus."""
    sim.memory.update(sim.sensor.observe(sim.grid, sim.drone.position))
    status = sim.controller.tick(sim.drone, sim.memory)
    if not status.complete:
        direction = sim.planner.next_move(sim.drone.position, sim.drone.mode, sim.memory)
        if direction is not None:
            sim.drone.move(direction)
    return status


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(seed: int = config.SEED) -> None:
    sim      = build_simulation(seed)
    renderer = Renderer(
        config.GRID_W, config.GRID_H,
        cell_size=config.CELL_SIZE,
        hud_height=config.HUD_HEIGHT,
        caption=f"Drone Simulator  |  seed={seed}",
    )
    renderer.snap_to(sim.drone.position)

    paused     = False
    speed_idx  = 0
    step_timer = 0.0
    status     = sim.controller.status

    while True:
        dt = renderer.tick(config.FPS)

        # ── Events ──────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                _quit()
            if event.type == pygame.KEYDOWN:
                match event.key:
                    case pygame.K_q | pygame.K_ESCAPE:
                        _quit()
                    case pygame.K_SPACE:
                        paused = not paused
                    case pygame.K_r:
                        sim        = build_simulation(seed)
                        status     = sim.controller.status
                        step_timer = 0.0
                        paused     = False
                        renderer.snap_to(sim.drone.position)
                    case pygame.K_PLUS | pygame.K_EQUALS | pygame.K_KP_PLUS:
                        speed_idx = min(speed_idx + 1, len(config.SPEED_LEVELS) - 1)
                    case pygame.K_MINUS | pygame.K_KP_MINUS:
                        speed_idx = max(speed_idx - 1, 0)

        # ── Simulation ticks ────────────────────────────────────────────
        if not paused and not status.complete:
            interval = 1.0 / (config.STEPS_PER_SECOND * config.SPEED_LEVELS[speed_idx])
            step_timer += dt
            while step_timer >= interval:
                step_timer -= interval
                status = step(sim)

        # ── Smooth movement ─────────────────────────────────────────────
        renderer.move_toward(sim.drone.position)

        # ── Render ──────────────────────────────────────────────────────
        renderer.draw(sim.memory, sim.drone, fuel_guard=sim.guard)

        if paused or status.complete:
            _draw_overlay(renderer, paused, status)
            pygame.display.flip()


def _draw_overlay(
    renderer: Renderer,
    paused: bool,
    status: MissionStatus,
) -> None:
    if paused:
        text  = "PAUSED"
        color = (200, 200, 80)
    elif status.success:
        text  = "MISSION SUCCESS"
        color = (80, 180, 80)
    else:
        text  = f"MISSION FAILED  —  {status.reason}"
        color = (200, 80, 80)

    font   = pygame.font.SysFont("monospace", 22, bold=True)
    surf   = font.render(text, True, color)
    width  = renderer.grid_w * renderer.cell_size
    centre = renderer.grid_h * renderer.cell_size // 2

    bg = pygame.Surface((width, 44), pygame.SRCALPHA)
    bg.fill((0, 0, 0, 180))
    renderer.screen.blit(bg,   (0,     centre - 22))
    renderer.screen.blit(surf, (width // 2 - surf.get_width() // 2, centre - 11))


def _quit() -> None:
    pygame.quit()
    sys.exit()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else config.SEED
    run(seed)

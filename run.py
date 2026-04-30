"""
Drone simulator — visual runner.

Controls
--------
  SPACE       pause / resume
  R           restart with the same seed
  +  /  -     speed up / slow down simulation
  Q  / ESC    quit
"""
import sys
import pygame

from environment import random_map, Position
from drone import Drone, DroneMode
from memory import DroneMemory, Sensor
from planner import Planner, MissionController, FuelGuard, BALANCED
from visualization import Renderer


# ── Configuration ────────────────────────────────────────────────────────────

GRID_W      = 18
GRID_H      = 13
CELL_SIZE   = 46
SEED        = 42
MAX_ENERGY  = 250.0
VISION_R    = 2

STEPS_PER_SECOND = 4   # simulation ticks/second at normal speed
FPS              = 60
SPEED_LEVELS     = [1, 2, 4, 8, 16]  # multipliers for steps/second


# ── Simulation factory ───────────────────────────────────────────────────────

def make_simulation(seed: int):
    grid       = random_map(GRID_W, GRID_H, seed=seed,
                            obstacle_density=0.12, threat_density=0.07)
    drone      = Drone(start=Position(1, 1), grid=grid, max_energy=MAX_ENERGY)
    memory     = DroneMemory.from_grid(grid)
    sensor     = Sensor(vision_radius=VISION_R)
    planner    = Planner(profile=BALANCED)
    guard      = FuelGuard()
    controller = MissionController(fuel_guard=guard)
    return grid, drone, memory, sensor, planner, guard, controller


def step_simulation(drone, memory, sensor, planner, guard, controller):
    """Run one simulation tick. Returns MissionStatus."""
    memory.update(sensor.observe(grid, drone.position))
    status = controller.tick(drone, memory)
    if not status.complete:
        direction = planner.next_move(drone.position, drone.mode, memory)
        if direction is not None:
            drone.move(direction)
    return status


# ── Main loop ────────────────────────────────────────────────────────────────

def run(seed: int = SEED) -> None:
    global grid  # step_simulation needs grid for sensor.observe

    grid, drone, memory, sensor, planner, guard, controller = make_simulation(seed)

    renderer = Renderer(GRID_W, GRID_H, cell_size=CELL_SIZE,
                        caption=f"Drone Simulator  |  seed={seed}")
    renderer.snap_to(drone.position)

    paused        = False
    speed_idx     = 0
    step_timer    = 0.0
    status        = controller.status

    while True:
        dt = renderer.tick(FPS)

        # ── Events ──────────────────────────────────────────────────────
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    pygame.quit()
                    sys.exit()
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_r:
                    # Restart — rebuild simulation, keep renderer
                    grid, drone, memory, sensor, planner, guard, controller = make_simulation(seed)
                    renderer.snap_to(drone.position)
                    status     = controller.status
                    step_timer = 0.0
                    paused     = False
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    speed_idx = min(speed_idx + 1, len(SPEED_LEVELS) - 1)
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    speed_idx = max(speed_idx - 1, 0)

        # ── Simulation ticks ────────────────────────────────────────────
        if not paused and not status.complete:
            step_interval = 1.0 / (STEPS_PER_SECOND * SPEED_LEVELS[speed_idx])
            step_timer += dt
            while step_timer >= step_interval:
                step_timer -= step_interval
                status = step_simulation(drone, memory, sensor, planner, guard, controller)

        # ── Smooth movement ─────────────────────────────────────────────
        renderer.move_toward(drone.position)

        # ── Render ──────────────────────────────────────────────────────
        renderer.draw(memory, drone, fuel_guard=guard)

        # Overlay: paused / mission complete banner
        if paused or status.complete:
            _draw_banner(renderer.screen, renderer.grid_h * renderer.cell_size,
                         renderer.grid_w * renderer.cell_size,
                         "PAUSED" if paused else
                         ("MISSION SUCCESS" if status.success else f"MISSION FAILED: {status.reason}"),
                         (80, 180, 80) if (status.complete and status.success) else
                         (200, 80, 80) if status.complete else (200, 200, 80))
            pygame.display.flip()


def _draw_banner(screen, y_offset: int, width: int, text: str, color) -> None:
    font   = pygame.font.SysFont("monospace", 22, bold=True)
    surf   = font.render(text, True, color)
    bg     = pygame.Surface((width, 40), pygame.SRCALPHA)
    bg.fill((0, 0, 0, 180))
    screen.blit(bg, (0, y_offset // 2 - 20))
    screen.blit(surf, (width // 2 - surf.get_width() // 2, y_offset // 2 - 14))


# ── Entry point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else SEED
    run(seed)

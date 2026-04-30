"""
Simulation and renderer configuration.

All tuneable parameters live here.  Import this module anywhere you need
a setting rather than scattering magic numbers across files.

  python run.py          → uses these defaults
  python run.py 99       → overrides SEED from CLI
"""

# ── World ────────────────────────────────────────────────────────────────────
GRID_W:           int   = 18
GRID_H:           int   = 13
SEED:             int   = 42
OBSTACLE_DENSITY: float = 0.12
THREAT_DENSITY:   float = 0.07

# ── Drone ────────────────────────────────────────────────────────────────────
MAX_ENERGY:    float = 250.0
VISION_RADIUS: int   = 2

# ── Renderer ─────────────────────────────────────────────────────────────────
CELL_SIZE:  int = 46
HUD_HEIGHT: int = 90
FPS:        int = 60

# ── Simulation speed ─────────────────────────────────────────────────────────
STEPS_PER_SECOND: int       = 4
SPEED_LEVELS:     list[int] = [1, 2, 4, 8, 16]

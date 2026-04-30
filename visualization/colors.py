"""
Color palette for the drone simulator renderer.

All values are RGB tuples (0-255).  Keeping them in one place means
changing the visual theme is a single-file edit.
"""
from memory.belief import BeliefState


# ── Cell backgrounds ─────────────────────────────────────────────────────────

CELL: dict[BeliefState, tuple[int, int, int]] = {
    BeliefState.UNKNOWN:  ( 30,  30,  40),   # near-black — unexplored void
    BeliefState.CLEAR:    (180, 190, 200),   # light grey  — safe open space
    BeliefState.OBSTACLE: ( 50,  50,  60),   # dark grey   — impassable wall
    BeliefState.THREAT:   (180,  60,  60),   # red         — danger zone
    BeliefState.TARGET:   ( 60, 180,  80),   # green       — mission goal
    BeliefState.BASE:     ( 60, 120, 200),   # blue        — home
}

# Grid line drawn between cells
GRID_LINE       = ( 20,  20,  30)

# ── Drone ────────────────────────────────────────────────────────────────────

DRONE_BODY      = (240, 220,  60)   # yellow
DRONE_HEADING   = (255, 255, 255)   # white arrow tip

# ── Path trail ───────────────────────────────────────────────────────────────

PATH_RECENT     = (240, 200,  60, 200)   # bright gold, mostly opaque
PATH_OLD        = ( 80,  80, 120,  60)   # dim blue-grey, nearly transparent
PATH_MAX_ALPHA  = 200
PATH_MIN_ALPHA  = 30

# ── HUD ──────────────────────────────────────────────────────────────────────

HUD_BG          = (  0,   0,   0, 160)   # semi-transparent black panel
HUD_TEXT        = (220, 220, 220)
HUD_LABEL       = (140, 140, 160)

MODE_COLOR = {
    "IDLE":       (160, 160, 160),
    "EXPLORING":  ( 80, 180, 240),
    "NAVIGATING": ( 80, 240, 120),
    "RETURNING":  (240, 160,  60),
    "CRASHED":    (240,  60,  60),
}

# ── Background ───────────────────────────────────────────────────────────────

BACKGROUND      = ( 15,  15,  20)

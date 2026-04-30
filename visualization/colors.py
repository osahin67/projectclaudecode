"""
Color palette for the drone simulator renderer.

All RGB/RGBA values live here.  Changing the visual theme is a single-file
edit with no grep required.
"""
from memory.belief import BeliefState


# ── Cell backgrounds ─────────────────────────────────────────────────────────

CELL: dict[BeliefState, tuple[int, int, int]] = {
    BeliefState.UNKNOWN:  ( 30,  30,  40),
    BeliefState.CLEAR:    (180, 190, 200),
    BeliefState.OBSTACLE: ( 50,  50,  60),
    BeliefState.THREAT:   (180,  60,  60),
    BeliefState.TARGET:   ( 60, 180,  80),
    BeliefState.BASE:     ( 60, 120, 200),
}

GRID_LINE = (20, 20, 30)

# ── Drone ────────────────────────────────────────────────────────────────────

DRONE_BODY    = (240, 220,  60)
DRONE_HEADING = (255, 255, 255)

# ── Path trail (alpha channel added dynamically in renderer) ─────────────────

PATH_MAX_ALPHA = 200
PATH_MIN_ALPHA =  30

# ── HUD ──────────────────────────────────────────────────────────────────────

HUD_BG    = (  0,   0,   0, 160)
HUD_TEXT  = (220, 220, 220)
HUD_LABEL = (140, 140, 160)

MODE_COLOR: dict[str, tuple[int, int, int]] = {
    "IDLE":       (160, 160, 160),
    "EXPLORING":  ( 80, 180, 240),
    "NAVIGATING": ( 80, 240, 120),
    "RETURNING":  (240, 160,  60),
    "CRASHED":    (240,  60,  60),
}

# ── Window ───────────────────────────────────────────────────────────────────

BACKGROUND = (15, 15, 20)

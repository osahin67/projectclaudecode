"""
Renderer — translates simulation state into pygame draw calls.

Rendering is strictly read-only: it receives data, never mutates it.
All mutable render state (smooth position, path surface cache) is
owned here, not in the simulation modules.

Coordinate systems
------------------
  grid  : integer (col, row) — simulation units
  pixel : float (x, y)       — screen pixels, top-left origin
  Conversion: pixel = grid * cell_size + cell_size / 2  (cell centre)
"""
from __future__ import annotations
import math
import pygame
from drone.drone import Drone
from drone.mode import Direction
from environment.grid import Position
from memory.map import DroneMemory
from planner.fuel import FuelGuard
from . import colors


class Renderer:
    FONT_SIZE   = 14
    HUD_PADDING = 8
    HUD_LINE_H  = 20

    def __init__(
        self,
        grid_w: int,
        grid_h: int,
        cell_size: int = 44,
        hud_height: int = 90,
        caption: str = "Drone Simulator",
    ) -> None:
        pygame.init()
        pygame.display.set_caption(caption)

        self.cell_size  = cell_size
        self.grid_w     = grid_w
        self.grid_h     = grid_h
        self.hud_height = hud_height

        win_w = grid_w * cell_size
        win_h = grid_h * cell_size + hud_height
        self.screen = pygame.display.set_mode((win_w, win_h))
        self.clock  = pygame.time.Clock()
        self._font  = pygame.font.SysFont("monospace", self.FONT_SIZE)

        # Smooth-movement state: render position in grid units (floats).
        self._rx: float = 0.0
        self._ry: float = 0.0

        # Pre-allocate a surface for the semi-transparent path trail.
        self._trail_surf = pygame.Surface((win_w, grid_h * cell_size), pygame.SRCALPHA)

    # ------------------------------------------------------------------
    # Main entry point — call once per render frame
    # ------------------------------------------------------------------

    def draw(
        self,
        memory: DroneMemory,
        drone: Drone,
        fuel_guard: FuelGuard | None = None,
    ) -> None:
        self.screen.fill(colors.BACKGROUND)

        self._draw_cells(memory)
        self._draw_trail(drone.history)
        self._draw_drone(drone)
        self._draw_hud(drone, memory, fuel_guard)

        pygame.display.flip()

    def tick(self, fps: int = 60) -> float:
        """Advance clock; returns delta-time in seconds."""
        return self.clock.tick(fps) / 1000.0

    # ------------------------------------------------------------------
    # Smooth movement
    # ------------------------------------------------------------------

    def move_toward(self, target: Position, alpha: float = 0.18) -> None:
        """
        Exponential lerp toward target grid position.

        alpha controls speed: 0.18 covers ~95% of distance in 16 frames.
        At 60 fps this takes ~270 ms — visually one smooth step between
        simulation ticks at 3–5 steps/second.
        """
        self._rx += (target.x - self._rx) * alpha
        self._ry += (target.y - self._ry) * alpha

    def snap_to(self, pos: Position) -> None:
        """Instantly place the render position (used at init)."""
        self._rx = float(pos.x)
        self._ry = float(pos.y)

    # ------------------------------------------------------------------
    # Cell grid
    # ------------------------------------------------------------------

    def _draw_cells(self, memory: DroneMemory) -> None:
        cs = self.cell_size
        for y in range(self.grid_h):
            for x in range(self.grid_w):
                belief = memory.get_belief(Position(x, y))
                color  = colors.CELL[belief]
                rect   = pygame.Rect(x * cs, y * cs, cs, cs)
                pygame.draw.rect(self.screen, color, rect)
                pygame.draw.rect(self.screen, colors.GRID_LINE, rect, 1)

    # ------------------------------------------------------------------
    # Path trail
    # ------------------------------------------------------------------

    def _draw_trail(self, history: list[Position]) -> None:
        if len(history) < 2:
            return

        self._trail_surf.fill((0, 0, 0, 0))
        cs   = self.cell_size
        half = cs // 2
        n    = len(history)

        # Draw oldest → newest; alpha increases toward most recent.
        for i in range(1, n):
            t     = i / n                          # 0.0 (oldest) … 1.0 (newest)
            alpha = int(colors.PATH_MIN_ALPHA + t * (colors.PATH_MAX_ALPHA - colors.PATH_MIN_ALPHA))
            r     = max(2, int(cs * 0.08 + t * cs * 0.08))

            prev = history[i - 1]
            curr = history[i]
            x0   = prev.x * cs + half
            y0   = prev.y * cs + half
            x1   = curr.x * cs + half
            y1   = curr.y * cs + half

            # Line segment
            pygame.draw.line(self._trail_surf, (*colors.DRONE_BODY, alpha), (x0, y0), (x1, y1), max(1, r))
            # Dot at each waypoint
            pygame.draw.circle(self._trail_surf, (*colors.DRONE_BODY, alpha), (x1, y1), r)

        self.screen.blit(self._trail_surf, (0, 0))

    # ------------------------------------------------------------------
    # Drone body and heading arrow
    # ------------------------------------------------------------------

    def _draw_drone(self, drone: Drone) -> None:
        cs   = self.cell_size
        half = cs // 2
        px   = self._rx * cs + half
        py   = self._ry * cs + half

        radius = max(4, int(cs * 0.28))
        pygame.draw.circle(self.screen, colors.DRONE_BODY, (int(px), int(py)), radius)
        pygame.draw.circle(self.screen, (0, 0, 0), (int(px), int(py)), radius, 2)

        # Heading arrow — drawn only when heading is known
        if drone.heading is not None:
            self._draw_heading_arrow(px, py, drone.heading, radius)

    def _draw_heading_arrow(
        self,
        cx: float,
        cy: float,
        heading: Direction,
        radius: int,
    ) -> None:
        angle = math.atan2(heading.dy, heading.dx)
        tip_dist   = radius + self.cell_size * 0.22
        wing_dist  = radius * 0.6
        wing_angle = math.pi * 0.45

        tip  = (cx + math.cos(angle) * tip_dist,
                cy + math.sin(angle) * tip_dist)
        left = (cx + math.cos(angle + wing_angle) * wing_dist,
                cy + math.sin(angle + wing_angle) * wing_dist)
        right= (cx + math.cos(angle - wing_angle) * wing_dist,
                cy + math.sin(angle - wing_angle) * wing_dist)

        pygame.draw.polygon(self.screen, colors.DRONE_HEADING, [tip, left, right])

    # ------------------------------------------------------------------
    # HUD — status bar below the grid
    # ------------------------------------------------------------------

    def _draw_hud(
        self,
        drone: Drone,
        memory: DroneMemory,
        fuel_guard: FuelGuard | None,
    ) -> None:
        cs     = self.cell_size
        top    = self.grid_h * cs
        width  = self.grid_w * cs
        hud_r  = pygame.Rect(0, top, width, self.hud_height)

        # Background
        hud_surf = pygame.Surface((width, self.hud_height), pygame.SRCALPHA)
        hud_surf.fill(colors.HUD_BG)
        self.screen.blit(hud_surf, (0, top))

        p  = self.HUD_PADDING
        lh = self.HUD_LINE_H

        mode_str   = drone.mode.name
        mode_color = colors.MODE_COLOR.get(mode_str, colors.HUD_TEXT)

        # Row 1: mode | step counter | coverage
        self._hud_text(f"MODE", top + p,         x=p,           color=colors.HUD_LABEL)
        self._hud_text(mode_str, top + p,         x=p + 48,      color=mode_color)
        self._hud_text(f"STEP", top + p,          x=160,         color=colors.HUD_LABEL)
        self._hud_text(str(drone.steps), top + p, x=210,         color=colors.HUD_TEXT)
        self._hud_text(f"MAP",  top + p,          x=290,         color=colors.HUD_LABEL)
        self._hud_text(f"{memory.coverage():.0%}", top + p, x=325, color=colors.HUD_TEXT)

        # Row 2: energy bar + numeric
        bar_y    = top + p + lh
        bar_x    = p
        bar_w    = 200
        bar_h    = 12
        ratio    = drone.energy_ratio
        bar_fill = int(bar_w * ratio)
        bar_color = (
            (80, 220, 80) if ratio > 0.5 else
            (220, 200, 50) if ratio > 0.25 else
            (220, 60, 60)
        )
        pygame.draw.rect(self.screen, (60, 60, 70), (bar_x, bar_y, bar_w, bar_h))
        pygame.draw.rect(self.screen, bar_color,    (bar_x, bar_y, bar_fill, bar_h))
        pygame.draw.rect(self.screen, (120, 120, 140), (bar_x, bar_y, bar_w, bar_h), 1)

        energy_str = f"{drone.energy:.0f} / {drone.energy:.0f}"
        self._hud_text(f"{drone.energy:.0f}", bar_y - 1, x=bar_x + bar_w + 6, color=colors.HUD_TEXT)

        # Row 3: return cost + target/base discovery
        row3_y = top + p + lh * 2
        if fuel_guard is not None and memory.base_position is not None:
            rc = fuel_guard.return_cost(drone.position, memory.base_position, memory)
            if rc is not None:
                margin = drone.energy - rc * fuel_guard.SAFETY_MARGIN
                m_color = (80, 220, 80) if margin > 10 else (220, 60, 60)
                self._hud_text("RETURN EST", row3_y, x=p,       color=colors.HUD_LABEL)
                self._hud_text(f"{rc:.1f}",  row3_y, x=p + 90,  color=colors.HUD_TEXT)
                self._hud_text("MARGIN",     row3_y, x=p + 140,  color=colors.HUD_LABEL)
                self._hud_text(f"{margin:+.1f}", row3_y, x=p+200, color=m_color)

        tgt = memory.target_position
        base= memory.base_position
        icons = []
        if base: icons.append(("BASE", colors.CELL[__import__('memory').BeliefState.BASE]))
        if tgt:  icons.append(("TARGET", colors.CELL[__import__('memory').BeliefState.TARGET]))
        ix = width - p
        for label, c in reversed(icons):
            w = self._font.size(label)[0] + 14
            ix -= w
            pygame.draw.rect(self.screen, c, (ix, row3_y, w, lh - 2), border_radius=3)
            self._hud_text(label, row3_y, x=ix + 4, color=(255, 255, 255))

    def _hud_text(
        self,
        text: str,
        y: int,
        x: int = 0,
        color: tuple[int, int, int] = colors.HUD_TEXT,
    ) -> None:
        surf = self._font.render(text, True, color)
        self.screen.blit(surf, (x, y))

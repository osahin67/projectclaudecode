"""
Renderer — translates simulation state into pygame draw calls.

The renderer is strictly read-only: it receives data, never mutates it.
All mutable render state (smooth position, trail surface) lives here,
not in simulation modules.

Coordinate systems
------------------
  grid  : integer (col, row) — simulation units
  pixel : float (x, y)       — screen pixels, top-left origin
  Conversion: pixel_centre = grid * cell_size + cell_size / 2
"""
from __future__ import annotations
import math
import pygame
from drone.drone import Drone
from drone.mode import Direction
from environment.grid import Position
from memory.belief import BeliefState
from memory.map import DroneMemory
from planner.fuel import FuelGuard
from . import colors


class Renderer:
    _FONT_SIZE    = 14
    _HUD_PADDING  = 8
    _HUD_LINE_H   = 20
    _LERP_ALPHA   = 0.18   # exponential lerp speed; ~95% of distance in 16 frames
    _ARROW_TIP    = 0.22   # tip reach as fraction of cell_size beyond drone radius
    _ARROW_WINGS  = 0.45   # wing half-angle in radians (π × 0.45)

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

        grid_pixel_h = grid_h * cell_size
        win_w        = grid_w  * cell_size
        win_h        = grid_pixel_h + hud_height

        self.screen = pygame.display.set_mode((win_w, win_h))
        self.clock  = pygame.time.Clock()
        self._font  = pygame.font.SysFont("monospace", self._FONT_SIZE)

        # Smooth-movement position in grid units (floats).
        self._rx: float = 0.0
        self._ry: float = 0.0

        # Reusable SRCALPHA surface for the transparent trail.
        self._trail_surf = pygame.Surface((win_w, grid_pixel_h), pygame.SRCALPHA)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def draw(
        self,
        memory: DroneMemory,
        drone: Drone,
        fuel_guard: FuelGuard | None = None,
    ) -> None:
        """Draw one complete frame."""
        self.screen.fill(colors.BACKGROUND)
        self._draw_cells(memory)
        self._draw_trail(drone.history)
        self._draw_drone(drone)
        self._draw_hud(drone, memory, fuel_guard)
        pygame.display.flip()

    def tick(self, fps: int = 60) -> float:
        """Advance clock; returns delta-time in seconds."""
        return self.clock.tick(fps) / 1000.0

    def move_toward(self, target: Position) -> None:
        """
        Exponential lerp of the render position toward target.
        At 60 fps the drone covers ~95% of one cell in 16 frames (≈270 ms),
        giving smooth motion at 3–5 simulation steps per second.
        """
        self._rx += (target.x - self._rx) * self._LERP_ALPHA
        self._ry += (target.y - self._ry) * self._LERP_ALPHA

    def snap_to(self, pos: Position) -> None:
        """Instantly place render position — use on init and restart."""
        self._rx = float(pos.x)
        self._ry = float(pos.y)

    # ------------------------------------------------------------------
    # Cell grid
    # ------------------------------------------------------------------

    def _draw_cells(self, memory: DroneMemory) -> None:
        cs = self.cell_size
        for y in range(self.grid_h):
            for x in range(self.grid_w):
                color = colors.CELL[memory.get_belief(Position(x, y))]
                rect  = pygame.Rect(x * cs, y * cs, cs, cs)
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

        for i in range(1, n):
            t     = i / n
            alpha = int(colors.PATH_MIN_ALPHA
                        + t * (colors.PATH_MAX_ALPHA - colors.PATH_MIN_ALPHA))
            r     = max(2, int(cs * 0.08 + t * cs * 0.08))
            color = (*colors.DRONE_BODY, alpha)

            prev = history[i - 1]
            curr = history[i]
            x0, y0 = prev.x * cs + half, prev.y * cs + half
            x1, y1 = curr.x * cs + half, curr.y * cs + half

            pygame.draw.line(self._trail_surf, color, (x0, y0), (x1, y1), max(1, r))
            pygame.draw.circle(self._trail_surf, color, (x1, y1), r)

        self.screen.blit(self._trail_surf, (0, 0))

    # ------------------------------------------------------------------
    # Drone body and heading arrow
    # ------------------------------------------------------------------

    def _draw_drone(self, drone: Drone) -> None:
        cs   = self.cell_size
        half = cs // 2
        px   = self._rx * cs + half
        py   = self._ry * cs + half
        r    = max(4, int(cs * 0.28))

        pygame.draw.circle(self.screen, colors.DRONE_BODY,  (int(px), int(py)), r)
        pygame.draw.circle(self.screen, (0, 0, 0),          (int(px), int(py)), r, 2)

        if drone.heading is not None:
            self._draw_heading_arrow(px, py, drone.heading, r)

    def _draw_heading_arrow(
        self, cx: float, cy: float, heading: Direction, radius: int
    ) -> None:
        angle     = math.atan2(heading.dy, heading.dx)
        tip_reach = radius + self.cell_size * self._ARROW_TIP
        wing_r    = radius * 0.6
        wing_a    = math.pi * self._ARROW_WINGS

        tip   = (cx + math.cos(angle)          * tip_reach,
                 cy + math.sin(angle)          * tip_reach)
        left  = (cx + math.cos(angle + wing_a) * wing_r,
                 cy + math.sin(angle + wing_a) * wing_r)
        right = (cx + math.cos(angle - wing_a) * wing_r,
                 cy + math.sin(angle - wing_a) * wing_r)

        pygame.draw.polygon(self.screen, colors.DRONE_HEADING, [tip, left, right])

    # ------------------------------------------------------------------
    # HUD
    # ------------------------------------------------------------------

    def _draw_hud(
        self,
        drone: Drone,
        memory: DroneMemory,
        fuel_guard: FuelGuard | None,
    ) -> None:
        cs    = self.cell_size
        top   = self.grid_h * cs
        width = self.grid_w * cs

        hud_surf = pygame.Surface((width, self.hud_height), pygame.SRCALPHA)
        hud_surf.fill(colors.HUD_BG)
        self.screen.blit(hud_surf, (0, top))

        p  = self._HUD_PADDING
        lh = self._HUD_LINE_H

        # Row 1: mode, step, coverage
        mode_str = drone.mode.name
        self._text(f"MODE",           top + p,          p,         colors.HUD_LABEL)
        self._text(mode_str,          top + p,          p + 48,    colors.MODE_COLOR.get(mode_str, colors.HUD_TEXT))
        self._text("STEP",            top + p,          160,       colors.HUD_LABEL)
        self._text(str(drone.steps),  top + p,          210,       colors.HUD_TEXT)
        self._text("MAP",             top + p,          290,       colors.HUD_LABEL)
        self._text(f"{memory.coverage():.0%}", top + p, 325,       colors.HUD_TEXT)

        # Row 2: energy bar
        bar_y, bar_x, bar_w, bar_h = top + p + lh, p, 200, 12
        pygame.draw.rect(self.screen, (60, 60, 70),                   (bar_x, bar_y, bar_w, bar_h))
        pygame.draw.rect(self.screen, self._energy_color(drone),      (bar_x, bar_y, int(bar_w * drone.energy_ratio), bar_h))
        pygame.draw.rect(self.screen, (120, 120, 140),                (bar_x, bar_y, bar_w, bar_h), 1)
        self._text(f"{drone.energy:.0f}", bar_y - 1, bar_x + bar_w + 6, colors.HUD_TEXT)

        # Row 3: return cost estimate + discovery badges
        row3_y = top + p + lh * 2
        if fuel_guard is not None and memory.base_position is not None:
            rc = fuel_guard.return_cost(drone.position, memory.base_position, memory)
            if rc is not None:
                margin      = drone.energy - rc * fuel_guard.SAFETY_MARGIN
                margin_color = (80, 220, 80) if margin > 10 else (220, 60, 60)
                self._text("RETURN EST",         row3_y, p,       colors.HUD_LABEL)
                self._text(f"{rc:.1f}",          row3_y, p + 90,  colors.HUD_TEXT)
                self._text("MARGIN",             row3_y, p + 140, colors.HUD_LABEL)
                self._text(f"{margin:+.1f}",     row3_y, p + 200, margin_color)

        self._draw_discovery_badges(memory, row3_y, width, p, lh)

    def _draw_discovery_badges(
        self,
        memory: DroneMemory,
        y: int,
        width: int,
        padding: int,
        line_h: int,
    ) -> None:
        badges = []
        if memory.base_position:
            badges.append(("BASE",   colors.CELL[BeliefState.BASE]))
        if memory.target_position:
            badges.append(("TARGET", colors.CELL[BeliefState.TARGET]))

        x = width - padding
        for label, color in reversed(badges):
            w = self._font.size(label)[0] + 14
            x -= w
            pygame.draw.rect(self.screen, color, (x, y, w, line_h - 2), border_radius=3)
            self._text(label, y, x + 4, (255, 255, 255))

    @staticmethod
    def _energy_color(drone: Drone) -> tuple[int, int, int]:
        r = drone.energy_ratio
        if r > 0.5:
            return (80, 220, 80)
        if r > 0.25:
            return (220, 200, 50)
        return (220, 60, 60)

    def _text(
        self,
        text: str,
        y: int,
        x: int = 0,
        color: tuple[int, int, int] = colors.HUD_TEXT,
    ) -> None:
        self.screen.blit(self._font.render(text, True, color), (x, y))

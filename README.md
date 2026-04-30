# Autonomous Drone Simulator

A Python simulation of an autonomous drone operating in an unknown environment. The drone explores a grid-based world, builds its own map from sensor observations, locates a target, and returns to base — all while managing limited fuel. Every decision the drone makes is based solely on what its sensor has seen; it never reads the ground truth.

Built with Python 3.11 and pygame. Implements real robotics algorithms: frontier-based exploration, A\* pathfinding on an occupancy grid, and position-aware fuel management.

---

## Key Features

- **Incremental map building** — the drone starts with a blank map and fills it in as its sensor sweeps cells. Unknown territory is tracked explicitly, not assumed empty.
- **Frontier-based exploration** — systematic coverage using Yamauchi's 1997 algorithm. The drone always moves toward the nearest boundary between known and unknown space, guaranteeing progress without random wandering.
- **A\* pathfinding with configurable risk profiles** — three named profiles (BALANCED, CAUTIOUS, AGGRESSIVE) tune how strongly the drone avoids unknown cells and threat zones. Cost is an exchange rate: THREAT=15 means the drone accepts up to 14 extra clear steps to avoid one threat cell.
- **Position-aware fuel management** — the drone estimates its return trip cost by running A\* to base at every tick, then aborts its mission when `current_fuel ≤ return_cost × 1.3`. No fixed percentage threshold; the decision scales with actual distance.
- **Information barrier** — the drone never accesses the ground-truth `Grid` directly. All world knowledge arrives through `Sensor.observe()`, which can be configured with a noise model. This is the same constraint real robots operate under.
- **Pygame visualisation** — 60 fps rendering with smooth drone movement (exponential lerp), a fading path trail, heading arrow, and a live HUD showing mode, coverage, energy bar, and return-cost estimate.

---

## Demo

```
python run.py          # default map (seed 42)
python run.py 99       # different map
```

| Control | Action |
|---|---|
| `SPACE` | Pause / resume |
| `R` | Restart with same seed |
| `+` / `-` | Speed up / slow down |
| `Q` / `ESC` | Quit |

---

## Architecture

The system is split into six modules with strict one-way dependencies. Each module owns its data and exposes a narrow interface.

```
Environment ──sensor query──► Sensor ──observations──► DroneMemory
                                                            │
                                                      map state│
                                                            ▼
                              DroneState ◄──move cmd── Planner
                                  │                        ▲
                            position/energy                │ phase
                                  ▼                        │
                          MissionController ───────────────┘
                                  │
                            FuelGuard (A* return estimate)
```

### `environment/`
Ground truth. The `Grid` holds the authoritative cell layout. `Sensor` wraps `grid.query_observable()` — the only path through which environment data reaches the drone. `ManualMapBuilder` and `random_map()` provide two ways to create a world.

### `drone/`
Physical state. Owns position, mode (`IDLE → EXPLORING → NAVIGATING → RETURNING`), energy, heading, and movement history. Validates bounds on `move()` but is completely obstacle-unaware — that is the Planner's responsibility.

### `memory/`
The drone's belief about the world. `DroneMemory` is an occupancy grid that starts fully `UNKNOWN` and updates cell-by-cell from `Sensor` observations. Tracks `BeliefState` per cell (UNKNOWN, CLEAR, OBSTACLE, THREAT, TARGET, BASE), visit count, and the step each cell was last seen. `frontiers()` returns the boundary of explored space for the exploration algorithm.

### `planner/`
Decision-making. Four components:

| File | Responsibility |
|---|---|
| `pathfinder.py` | A\* on the belief map with lazy-deletion visited check and configurable `CostProfile` |
| `planner.py` | Goal selection (nearest frontier / target / base) and path caching with obstacle-triggered replan |
| `controller.py` | Phase state machine — evaluates transition conditions and calls `drone.set_mode()` |
| `fuel.py` | `FuelGuard` — A\*-based return-trip estimation with 1.3× safety margin |

### `visualization/`
Rendering only. `Renderer` receives state, draws pixels, owns no simulation data. Smooth movement uses exponential lerp (`render_pos += (target - render_pos) × 0.18`). The trail fades both alpha and dot size from the oldest position to the most recent.

### `config.py` / `constants.py`
All tuneable parameters and shared physics constants live here. `DIAGONAL_COST = math.sqrt(2)` appears once and is imported by `drone.py`, `pathfinder.py`, and `fuel.py`.

---

## Algorithms

### Frontier-Based Exploration
Implemented in `DroneMemory.frontiers()`. A frontier cell is any `UNKNOWN` cell adjacent to at least one known passable cell. The Planner picks the closest frontier by Manhattan distance each tick, then routes to it with A\*. This is the dominant exploration strategy in real ROS navigation stacks.

### A\* on an Occupancy Grid
`planner/pathfinder.py` runs A\* over the belief map with a Chebyshev heuristic (admissible for 8-directional movement). Cell costs:

| Belief state | BALANCED cost | Meaning |
|---|---|---|
| CLEAR | 1.0 | Baseline |
| UNKNOWN | 3.0 | Prefer mapped routes; cross unknown if needed |
| THREAT | 15.0 | Detour up to 14 clear steps to avoid |
| OBSTACLE | ∞ | Impassable |

Uses lazy deletion for the visited-node check: the g-score is stored in each heap entry and checked on pop, avoiding a separate closed set.

### Position-Aware Fuel Management
`FuelGuard.must_return()` runs A\* with an energy-only profile (all cell costs = 1.0, matching actual fuel burn) to compute the minimum possible return trip cost. The drone turns back when:

```
current_energy  ≤  return_cost × 1.3
```

The 1.3 margin covers newly discovered obstacles that may invalidate the estimated path. Before base is located in memory, a conservative 0.45 ratio is used as a fallback. `can_reach_target_and_return()` runs the same check over both legs before allowing a NAVIGATING transition.

---

## Real-World Relevance

This simulator implements the same algorithmic stack used in mobile robotics:

| Simulator | Real-world equivalent |
|---|---|
| `DroneMemory` occupancy grid | `nav_msgs/OccupancyGrid` in ROS |
| `BeliefState.UNKNOWN` | The -1 (unknown) cell in ROS occupancy grids |
| `Sensor` with noise model | Sensor fusion noise models in Kalman / particle filters |
| `frontiers()` | Frontier Exploration (Yamauchi, 1997) — baseline in most ROS navigation stacks |
| `CostProfile` on A\* | Costmap inflation layers in `nav2` |
| `FuelGuard` return estimate | Battery-aware mission planning in commercial UAV firmware |

The key property the system preserves: **the drone reasons only from `DroneMemory`**. The ground-truth `Grid` is never imported into planning modules. If you deleted the `environment/` package, the drone's planning logic would still compile.

---

## Installation

Requires Python 3.11+.

```bash
git clone https://github.com/osahin67/projectclaudecode
cd projectclaudecode
pip install -r requirements.txt
python run.py
```

Or install as a package:

```bash
pip install -e .
drone-simulator
```

---

## Project Structure

```
├── config.py               # all tuneable parameters
├── constants.py            # shared physics constants (DIAGONAL_COST, etc.)
├── run.py                  # entry point and simulation loop
│
├── environment/
│   ├── cell.py             # CellType enum
│   ├── grid.py             # Grid, Position, ObservedCell
│   └── generator.py        # random_map(), ManualMapBuilder
│
├── drone/
│   ├── mode.py             # DroneMode, Direction enums
│   └── drone.py            # Drone class
│
├── memory/
│   ├── belief.py           # BeliefState, CellRecord
│   ├── sensor.py           # Sensor (environment → belief)
│   └── map.py              # DroneMemory (occupancy grid + frontiers)
│
├── planner/
│   ├── pathfinder.py       # A* with CostProfile presets
│   ├── planner.py          # goal selection and path following
│   ├── controller.py       # mission phase state machine
│   └── fuel.py             # FuelGuard (position-aware fuel decisions)
│
└── visualization/
    ├── colors.py            # color palette
    └── renderer.py          # Renderer (pygame, smooth movement, HUD)
```

---

## Configuration

Edit `config.py` to change any simulation parameter without touching module code.

```python
GRID_W            = 18      # map width in cells
GRID_H            = 13      # map height in cells
OBSTACLE_DENSITY  = 0.12    # fraction of cells that are obstacles
THREAT_DENSITY    = 0.07    # fraction of cells that are threat zones
MAX_ENERGY        = 250.0   # total fuel
VISION_RADIUS     = 2       # sensor range in cells (Chebyshev)
STEPS_PER_SECOND  = 4       # simulation speed at 1× (adjustable with +/-)
```

To change the drone's risk tolerance, pass a different profile to `Planner`:

```python
from planner import Planner, CAUTIOUS
planner = Planner(profile=CAUTIOUS)   # avoids unknown cells and threats more strongly
```

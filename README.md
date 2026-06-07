# VCES-D* Lite

**VCES-D* Lite** stands for **Visibility-Constrained Edge-State D* Lite Replanning**.

This is a compact Python project for online path planning in a `9x9` maze where obstacles are placed on **edges between adjacent cells** rather than inside cells. The robot does not know the full map in advance. It only scans a limited front-facing local region, updates the state of visible edges, and repairs its path incrementally with a D* Lite-style replanning core.

![VCES-D* Lite overview](./overview.png)

## Project Goal

The project studies a simple but meaningful online planning setting:

- The environment is a `9x9` grid with `81` cells.
- Obstacles lie on **edges** between cells.
- Each edge has one of three states: `UNKNOWN`, `FREE`, or `BLOCKED`.
- The robot can plan through unknown edges as a temporary hypothesis.
- The robot is only allowed to **move through edges that have been certified `FREE`**.
- Before moving into an unknown direction, the robot must scan its **front `3x3` visible area**.

The main idea is to combine:

- local visibility-constrained sensing,
- edge-state map updating,
- and D* Lite incremental replanning.

## Core Workflow

The navigation loop follows this pattern:

1. Compute or repair a candidate path to the current goal.
2. Check the next edge on that path.
3. If the next edge is `UNKNOWN`, rotate toward it and scan the front `3x3` local region.
4. Update observed edge states as `FREE` or `BLOCKED`.
5. If any edge cost changed, repair the path with D* Lite-style affected-vertex updates.
6. If the next edge is confirmed `FREE`, move one cell forward.
7. Repeat until the goal is reached.

In short:

```text
PLAN -> SCAN -> UPDATE EDGE STATES -> REPAIR PATH -> MOVE
```

## Why This Project Is Interesting

This project is small, but it captures several nontrivial planning ideas:

- **Partial observability**: the robot cannot see the full maze.
- **Edge-based obstacle modeling**: obstacles are attached to transitions, not cells.
- **Safe execution policy**: candidate paths may cross unknown edges, but motion may not.
- **Incremental replanning**: only changed edge costs trigger local repair instead of full recomputation.

It is therefore a useful educational example for:

- D* Lite style replanning,
- local sensing under uncertainty,
- and robot decision loops that alternate between sensing and acting.

## Repository Structure

```text
Visibility-Constrained Edge-State D Lite Replanning/
├── README.md
├── ChatGPT Image 2026年6月7日 21_33_23.png
├── maze_dstar_lite.py
├── vces-dstar-lite/
│   ├── README.md
│   ├── pyproject.toml
│   ├── docs/
│   │   └── METHOD.md
│   ├── examples/
│   │   └── run_demo.py
│   ├── ros1/
│   │   ├── README.md
│   │   └── maze_planner_node.py
│   ├── src/
│   │   └── vces_dstar_lite/
│   │       ├── __init__.py
│   │       ├── __main__.py
│   │       └── core.py
│   └── tests/
│       └── test_core.py
```

## Main Files

- `maze_dstar_lite.py`
  A standalone single-file version of the planner.

- `vces-dstar-lite/src/vces_dstar_lite/core.py`
  The main packaged implementation. This is the most important source file.

- `vces-dstar-lite/tests/test_core.py`
  Basic tests for topology, obstacle generation, and end-to-end navigation.

- `vces-dstar-lite/examples/run_demo.py`
  A small simulation entry point.

## Key Components

Inside `core.py`, the main classes are:

- `EdgeGrid`
  Builds the `9x9` edge-based maze topology and stores edge states and costs.

- `VisibilityModel`
  Simulates front-facing `3x3` local sensing with simple occlusion logic.

- `DStarLitePlanner`
  Maintains `g`, `rhs`, and the open list for incremental shortest-path repair.

- `MazeNavigator`
  Implements the high-level decision loop:
  `PLAN`, `SCAN`, `MOVE`, `DONE`, or `NO_PATH`.

## Edge States and Costs

Each internal edge is assigned a state and traversal cost:

| State | Meaning | Cost |
|---|---|---|
| `UNKNOWN` | not scanned yet | `1.2` |
| `FREE` | observed traversable | `1.0` |
| `BLOCKED` | observed obstacle | `infinity` |

This cost design encourages the planner to use unknown edges when necessary, while still preferring already confirmed free edges.

## How to Run

From the packaged project directory:

```bash
cd vces-dstar-lite
python -m vces_dstar_lite --start 1 --goals 14,44,68,38 --seed 11
```

Or run the example script:

```bash
cd vces-dstar-lite
python examples/run_demo.py
```

If the package is not installed, you can also run with:

```bash
cd vces-dstar-lite
set PYTHONPATH=src
python -m vces_dstar_lite --start 1 --goals 14,44,68,38 --seed 11
```

## Example Output Behavior

During simulation, the planner prints step-by-step actions such as:

- `SCAN`: the next edge is unknown, so the robot scans locally.
- `MOVE`: the next edge is certified free, so the robot advances.
- `PLAN`: the path is repaired or switched to a new goal.
- `DONE`: all goals have been reached.

This makes it easy to observe how sensing and replanning interact over time.

## ROS1 Note

The project itself is pure Python and has no ROS dependency in the core planner. A minimal ROS1 wrapper skeleton is included in:

- `vces-dstar-lite/ros1/maze_planner_node.py`

This wrapper shows how the planner could be connected to:

- a perception node that produces edge observations,
- a motion controller that executes cell-to-cell moves,
- and a callback that confirms robot arrival at the next cell.

## Educational Value

This project is especially suitable for:

- robotics course projects,
- path planning demonstrations,
- D* Lite learning exercises,
- and small research prototypes on local sensing and online replanning.

It is not intended as a large-scale benchmark system. Its value is in presenting a clean and understandable combination of:

- limited visibility,
- edge-state uncertainty,
- and incremental path repair.

## Related Folder

If you want the original package-style documentation and code layout details, see:

- [vces-dstar-lite/README.md](./vces-dstar-lite/README.md)
- [vces-dstar-lite/docs/METHOD.md](./vces-dstar-lite/docs/METHOD.md)

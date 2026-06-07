# VCES-D* Lite

**VCES-D* Lite** stands for **Visibility-Constrained Edge-State D* Lite Replanning**.

This project implements online path planning for a `9x9` maze in which obstacles are placed on **edges between adjacent cells**. The robot does not know the full map in advance. It scans a front-facing local region, updates edge states, and repairs its path with a D* Lite-style incremental replanning core.

![VCES-D* Lite overview](./overview.png)

## Problem Setting

- The environment is a `9x9` grid with `81` cells.
- Obstacles lie on **edges** between cells.
- Each edge has one of three states: `UNKNOWN`, `FREE`, or `BLOCKED`.
- The robot may plan through unknown edges as a temporary hypothesis.
- The robot may move only through edges that have been confirmed `FREE`.
- Before moving toward an unknown edge, the robot scans its front `3x3` visible area.

## Core Workflow

1. Compute or repair a candidate path to the current goal.
2. Check the next edge on that path.
3. If the next edge is `UNKNOWN`, rotate toward it and scan the front `3x3` local region.
4. Update observed edge states as `FREE` or `BLOCKED`.
5. If any edge cost changed, repair the path with D* Lite-style affected-vertex updates.
6. If the next edge is confirmed `FREE`, move one cell forward.
7. Repeat until the goal is reached.

```text
PLAN -> SCAN -> UPDATE EDGE STATES -> REPAIR PATH -> MOVE
```

## Repository Structure

```text
Visibility-Constrained Edge-State D Lite Replanning/
|-- README.md
|-- overview.png
|-- maze_dstar_lite.py
`-- vces-dstar-lite/
    |-- README.md
    |-- pyproject.toml
    |-- docs/
    |   `-- METHOD.md
    |-- examples/
    |   `-- run_demo.py
    |-- ros1/
    |   |-- README.md
    |   `-- maze_planner_node.py
    |-- src/
    |   `-- vces_dstar_lite/
    |       |-- __init__.py
    |       |-- __main__.py
    |       `-- core.py
    `-- tests/
        `-- test_core.py
```

## Main Files

- `maze_dstar_lite.py`
  Standalone single-file version of the planner.

- `vces-dstar-lite/src/vces_dstar_lite/core.py`
  Main packaged implementation.

- `vces-dstar-lite/tests/test_core.py`
  Tests for topology, obstacle generation, and end-to-end navigation.

- `vces-dstar-lite/examples/run_demo.py`
  Simulation entry point.

## Key Components

- `EdgeGrid`
  Builds the `9x9` edge-based maze topology and stores edge states and costs.

- `VisibilityModel`
  Simulates front-facing `3x3` local sensing with occlusion checks.

- `DStarLitePlanner`
  Maintains `g`, `rhs`, and the open list for incremental shortest-path repair.

- `MazeNavigator`
  Implements the high-level decision loop: `PLAN`, `SCAN`, `MOVE`, `DONE`, or `NO_PATH`.

## Edge States and Costs

| State | Meaning | Cost |
|---|---|---|
| `UNKNOWN` | not scanned yet | `1.2` |
| `FREE` | observed traversable | `1.0` |
| `BLOCKED` | observed obstacle | `infinity` |

`FREE` edges are cheaper than `UNKNOWN` edges, and `BLOCKED` edges are non-traversable.

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

If the package is not installed, run with:

```bash
cd vces-dstar-lite
set PYTHONPATH=src
python -m vces_dstar_lite --start 1 --goals 14,44,68,38 --seed 11
```

## Example Output

During simulation, the planner prints step-by-step actions such as:

- `SCAN`
- `MOVE`
- `PLAN`
- `DONE`

## ROS1 Note

The core planner has no ROS dependency. A minimal ROS1 wrapper skeleton is included in:

- `vces-dstar-lite/ros1/maze_planner_node.py`

## Additional Documentation

- [vces-dstar-lite/README.md](./vces-dstar-lite/README.md)
- [vces-dstar-lite/docs/METHOD.md](./vces-dstar-lite/docs/METHOD.md)

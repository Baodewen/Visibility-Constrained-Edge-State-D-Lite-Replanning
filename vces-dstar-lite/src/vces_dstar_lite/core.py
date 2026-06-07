#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vces_dstar_lite/core.py

Python 3.9 implementation of a visibility-constrained edge-state D* Lite planner
for a 9×9 grid maze.

Problem model
-------------
1. Cells are numbered from 1 to 81, left-to-right and top-to-bottom.
2. Obstacles are not placed inside cells. Obstacles are placed on edges between
   adjacent cells.
3. The robot maintains an edge-state map:
       UNKNOWN = not observed yet
       FREE    = observed traversable
       BLOCKED = observed obstacle
4. Unknown edges are allowed in the candidate path, but the robot must scan and
   certify the next edge before moving.
5. Sensor model:
       - scan only the near-field 3×3 area in front of the robot;
       - a candidate edge is observed only if line-of-sight is not occluded by a
         true blocked edge;
       - rear side is not scanned.
6. D* Lite is used as the replanning core:
       - when an edge cost changes, only affected vertices are updated;
       - the shortest path is repaired incrementally rather than recomputed
         from scratch.

This file is pure Python and has no ROS dependency. In ROS1, wrap MazeNavigator
inside a rospy node and connect:
    - sensor output -> navigator.apply_observations(...)
    - planner output -> motion controller command
    - cell arrival callback -> navigator.move_to(...)
"""

from __future__ import annotations

import argparse
import heapq
import math
import random
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


INF = 1.0e12
EPS = 1.0e-9


class EdgeState(IntEnum):
    """State of an internal grid edge."""
    UNKNOWN = 0
    FREE = 1
    BLOCKED = 2


class Heading(IntEnum):
    """Cardinal headings. Values are chosen only for compact representation."""
    N = 0
    E = 1
    S = 2
    W = 3


DIR_DELTA: Dict[Heading, Tuple[int, int]] = {
    Heading.N: (-1, 0),
    Heading.E: (0, 1),
    Heading.S: (1, 0),
    Heading.W: (0, -1),
}

DIR_ARROW: Dict[Heading, str] = {
    Heading.N: "↑",
    Heading.E: "→",
    Heading.S: "↓",
    Heading.W: "←",
}


def left_of(h: Heading) -> Heading:
    return {
        Heading.N: Heading.W,
        Heading.W: Heading.S,
        Heading.S: Heading.E,
        Heading.E: Heading.N,
    }[h]


def right_of(h: Heading) -> Heading:
    return {
        Heading.N: Heading.E,
        Heading.E: Heading.S,
        Heading.S: Heading.W,
        Heading.W: Heading.N,
    }[h]


@dataclass(frozen=True)
class EdgeSpec:
    """Static geometry and topology of one internal edge."""
    edge_id: int
    u: int
    v: int
    kind: str              # 'V' for vertical wall segment, 'H' for horizontal
    row: int               # 0-based row of the segment anchor
    col: int               # 0-based column of the segment anchor


@dataclass
class StepResult:
    """Result returned by MazeNavigator.step_simulation()."""
    action: str
    current: int
    goal: int
    heading: Heading
    path: List[int]
    changed_edges: List[int]
    affected_vertices: List[int]
    message: str


class EdgeGrid:
    """
    Fixed 9×9 edge-based grid.

    Internal cells are numbered 1..81.
    Internal edges are numbered 0..143.
    """

    def __init__(
        self,
        rows: int = 9,
        cols: int = 9,
        free_cost: float = 1.0,
        unknown_cost: float = 1.2,
    ) -> None:
        if rows != 9 or cols != 9:
            raise ValueError("This implementation is specialized for a 9×9 grid.")
        self.rows = rows
        self.cols = cols
        self.free_cost = float(free_cost)
        self.unknown_cost = float(unknown_cost)

        self.edges: List[EdgeSpec] = []
        self.neighbors: Dict[int, List[Tuple[int, int, Heading]]] = {
            c: [] for c in range(1, rows * cols + 1)
        }
        self.edge_between: Dict[Tuple[int, int], int] = {}

        self._build_topology()
        self.state: List[EdgeState] = [EdgeState.UNKNOWN for _ in self.edges]

    @staticmethod
    def cell_to_rc(cell: int) -> Tuple[int, int]:
        """Convert 1..81 cell id to 0-based (row, col)."""
        if cell < 1 or cell > 81:
            raise ValueError(f"cell must be in 1..81, got {cell}")
        cell0 = cell - 1
        return cell0 // 9, cell0 % 9

    @staticmethod
    def rc_to_cell(row: int, col: int) -> int:
        """Convert 0-based (row, col) to 1..81 cell id."""
        if not (0 <= row < 9 and 0 <= col < 9):
            raise ValueError(f"row/col out of range: {(row, col)}")
        return row * 9 + col + 1

    def _add_edge(self, u: int, v: int, kind: str, row: int, col: int) -> None:
        eid = len(self.edges)
        self.edges.append(EdgeSpec(eid, u, v, kind, row, col))
        key = (min(u, v), max(u, v))
        self.edge_between[key] = eid

        # Add undirected adjacency.
        hu = self.direction_between(u, v)
        hv = self.direction_between(v, u)
        self.neighbors[u].append((v, eid, hu))
        self.neighbors[v].append((u, eid, hv))

    def _build_topology(self) -> None:
        # Vertical wall segments: between (r,c) and (r,c+1)
        for r in range(self.rows):
            for c in range(self.cols - 1):
                u = self.rc_to_cell(r, c)
                v = self.rc_to_cell(r, c + 1)
                self._add_edge(u, v, "V", r, c)

        # Horizontal wall segments: between (r,c) and (r+1,c)
        for r in range(self.rows - 1):
            for c in range(self.cols):
                u = self.rc_to_cell(r, c)
                v = self.rc_to_cell(r + 1, c)
                self._add_edge(u, v, "H", r, c)

    def reset_states(self, state: EdgeState = EdgeState.UNKNOWN) -> None:
        self.state = [state for _ in self.edges]

    def validate_cell(self, cell: int) -> int:
        if cell < 1 or cell > self.rows * self.cols:
            raise ValueError(f"cell must be in 1..81, got {cell}")
        return cell

    def edge_id_between(self, a: int, b: int) -> int:
        key = (min(a, b), max(a, b))
        if key not in self.edge_between:
            raise ValueError(f"cells {a} and {b} are not adjacent")
        return self.edge_between[key]

    def edge_cells(self, edge_id: int) -> Tuple[int, int]:
        e = self.edges[edge_id]
        return e.u, e.v

    def set_edge_state(self, edge_id: int, new_state: EdgeState) -> Tuple[float, float]:
        """Set edge state and return (old_cost, new_cost)."""
        old_cost = self.edge_cost(edge_id)
        self.state[edge_id] = new_state
        new_cost = self.edge_cost(edge_id)
        return old_cost, new_cost

    def edge_cost(self, edge_id: int) -> float:
        st = self.state[edge_id]
        if st == EdgeState.BLOCKED:
            return INF
        if st == EdgeState.FREE:
            return self.free_cost
        return self.unknown_cost

    def cost(self, a: int, b: int) -> float:
        return self.edge_cost(self.edge_id_between(a, b))

    def direction_between(self, a: int, b: int) -> Heading:
        ra, ca = self.cell_to_rc(a)
        rb, cb = self.cell_to_rc(b)
        if rb == ra - 1 and cb == ca:
            return Heading.N
        if rb == ra + 1 and cb == ca:
            return Heading.S
        if rb == ra and cb == ca + 1:
            return Heading.E
        if rb == ra and cb == ca - 1:
            return Heading.W
        raise ValueError(f"cells {a} and {b} are not adjacent")

    def step_neighbor(self, cell: int, heading: Heading) -> Optional[int]:
        r, c = self.cell_to_rc(cell)
        dr, dc = DIR_DELTA[heading]
        nr, nc = r + dr, c + dc
        if 0 <= nr < self.rows and 0 <= nc < self.cols:
            return self.rc_to_cell(nr, nc)
        return None

    def incident_edges(self, cell: int) -> List[int]:
        return [eid for _, eid, _ in self.neighbors[cell]]

    def manhattan(self, a: int, b: int) -> float:
        ra, ca = self.cell_to_rc(a)
        rb, cb = self.cell_to_rc(b)
        return float(abs(ra - rb) + abs(ca - cb))


class LazyPriorityQueue:
    """
    Priority queue for D* Lite with lazy deletion.

    Python heapq has no decrease-key operation. We store the latest key per node
    in entry_finder and discard stale heap entries when they are popped.
    """

    def __init__(self) -> None:
        self.heap: List[Tuple[float, float, int, int]] = []
        self.entry_finder: Dict[int, Tuple[float, float]] = {}
        self.counter = 0

    def __len__(self) -> int:
        return len(self.entry_finder)

    @staticmethod
    def key_less(a: Tuple[float, float], b: Tuple[float, float]) -> bool:
        if a[0] < b[0] - EPS:
            return True
        if a[0] > b[0] + EPS:
            return False
        return a[1] < b[1] - EPS

    def push(self, node: int, key: Tuple[float, float]) -> None:
        self.entry_finder[node] = key
        self.counter += 1
        heapq.heappush(self.heap, (key[0], key[1], self.counter, node))

    def remove(self, node: int) -> None:
        self.entry_finder.pop(node, None)

    def contains(self, node: int) -> bool:
        return node in self.entry_finder

    def top_key(self) -> Tuple[float, float]:
        self._discard_stale()
        if not self.heap:
            return (INF, INF)
        k1, k2, _, _ = self.heap[0]
        return (k1, k2)

    def pop(self) -> Tuple[int, Tuple[float, float]]:
        self._discard_stale()
        if not self.heap:
            raise IndexError("pop from empty priority queue")
        k1, k2, _, node = heapq.heappop(self.heap)
        key = self.entry_finder.pop(node)
        return node, key

    def _discard_stale(self) -> None:
        while self.heap:
            k1, k2, _, node = self.heap[0]
            latest = self.entry_finder.get(node)
            if latest is not None and abs(latest[0] - k1) < EPS and abs(latest[1] - k2) < EPS:
                return
            heapq.heappop(self.heap)


class DStarLitePlanner:
    """
    D* Lite planner over EdgeGrid.

    The planner is goal-directed. It repairs the path whenever edge costs change.
    """

    def __init__(self, grid: EdgeGrid, start: int, goal: int) -> None:
        self.grid = grid
        self.start = grid.validate_cell(start)
        self.goal = grid.validate_cell(goal)
        self.km = 0.0

        self.g: List[float] = [INF] * (self.grid.rows * self.grid.cols + 1)
        self.rhs: List[float] = [INF] * (self.grid.rows * self.grid.cols + 1)
        self.open = LazyPriorityQueue()

        self.queue_pops = 0
        self.vertex_updates = 0
        self.affected_vertices: Set[int] = set()

        self.initialize()

    def initialize(self) -> None:
        n = self.grid.rows * self.grid.cols
        self.g = [INF] * (n + 1)
        self.rhs = [INF] * (n + 1)
        self.open = LazyPriorityQueue()
        self.km = 0.0
        self.rhs[self.goal] = 0.0
        self.open.push(self.goal, self.calculate_key(self.goal))
        self.compute_shortest_path()

    def reset_goal(self, start: int, goal: int) -> None:
        """Change start/goal. Known edge states are preserved; D* state is reset."""
        self.start = self.grid.validate_cell(start)
        self.goal = self.grid.validate_cell(goal)
        self.initialize()

    def heuristic(self, a: int, b: int) -> float:
        return self.grid.manhattan(a, b)

    def calculate_key(self, s: int) -> Tuple[float, float]:
        m = min(self.g[s], self.rhs[s])
        return (m + self.heuristic(self.start, s) + self.km, m)

    def update_vertex(self, u: int) -> None:
        self.affected_vertices.add(u)
        self.vertex_updates += 1

        if u != self.goal:
            best = INF
            for v, _, _ in self.grid.neighbors[u]:
                best = min(best, self.grid.cost(u, v) + self.g[v])
            self.rhs[u] = best

        if self.open.contains(u):
            self.open.remove(u)

        if abs(self.g[u] - self.rhs[u]) > EPS:
            self.open.push(u, self.calculate_key(u))

    def compute_shortest_path(self) -> None:
        """
        Repair shortest path from current start to goal.

        Complexity is proportional to the number of inconsistent vertices that
        need processing after edge-cost updates.
        """
        self.queue_pops = 0
        self.vertex_updates = 0
        guard = 0

        while LazyPriorityQueue.key_less(self.open.top_key(), self.calculate_key(self.start)) or (
            abs(self.rhs[self.start] - self.g[self.start]) > EPS
        ):
            guard += 1
            if guard > 10000:
                raise RuntimeError("D* Lite loop guard triggered; check edge costs or topology.")

            u, k_old = self.open.pop()
            k_new = self.calculate_key(u)

            if LazyPriorityQueue.key_less(k_old, k_new):
                self.open.push(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                self.queue_pops += 1
                for pred, _, _ in self.grid.neighbors[u]:
                    self.update_vertex(pred)
            else:
                self.g[u] = INF
                self.queue_pops += 1
                self.update_vertex(u)
                for pred, _, _ in self.grid.neighbors[u]:
                    self.update_vertex(pred)

    def notify_edge_cost_changed(self, edge_id: int) -> None:
        """
        Notify D* Lite that one undirected edge changed cost.

        For an undirected graph, both endpoint vertices can be affected.
        """
        u, v = self.grid.edge_cells(edge_id)
        self.update_vertex(u)
        self.update_vertex(v)

    def repair_after_edge_changes(self, changed_edges: Iterable[int]) -> None:
        self.affected_vertices.clear()
        for eid in changed_edges:
            self.notify_edge_cost_changed(eid)
        self.compute_shortest_path()

    def move_start(self, new_start: int) -> None:
        """Update current robot node after a successful movement."""
        old = self.start
        self.start = self.grid.validate_cell(new_start)
        self.km += self.heuristic(old, self.start)
        self.compute_shortest_path()

    def get_path(self, max_len: int = 200) -> List[int]:
        """
        Extract a greedy path from current start to goal using current g-values.

        Returns an empty list if no path is known under current edge costs.
        """
        if self.rhs[self.start] >= INF / 2 and self.g[self.start] >= INF / 2:
            return []

        path = [self.start]
        current = self.start
        seen = {current}

        for _ in range(max_len):
            if current == self.goal:
                return path

            best_next = None
            best_value = INF
            for nxt, _, _ in self.grid.neighbors[current]:
                value = self.grid.cost(current, nxt) + self.g[nxt]
                if value < best_value - EPS:
                    best_value = value
                    best_next = nxt

            if best_next is None or best_value >= INF / 2 or best_next in seen:
                return []

            current = best_next
            path.append(current)
            seen.add(current)

        return []


class VisibilityModel:
    """
    Precomputed front 3×3 visibility model.

    A candidate edge is in the front near-field if its midpoint is:
        forward in (0, 3.5]
        lateral within ±1.5 cells
    relative to the robot heading.

    Occlusion is evaluated by precomputed line-segment intersections. In real ROS
    use, your sensor node can skip this geometry and simply pass observed edges
    into MazeNavigator.apply_observations().
    """

    def __init__(self, grid: EdgeGrid) -> None:
        self.grid = grid
        self.candidates: Dict[Tuple[int, Heading], List[int]] = {}
        self.occluders: Dict[Tuple[int, Heading, int], Tuple[int, ...]] = {}
        self._precompute()

    def _precompute(self) -> None:
        for cell in range(1, 82):
            for heading in Heading:
                cands = []
                for edge_id in range(len(self.grid.edges)):
                    if self._in_front_3x3(cell, heading, edge_id):
                        cands.append(edge_id)
                        occ = self._compute_occluders(cell, edge_id)
                        self.occluders[(cell, heading, edge_id)] = tuple(occ)
                self.candidates[(cell, heading)] = cands

    @staticmethod
    def _cell_center(cell: int) -> Tuple[float, float]:
        r, c = EdgeGrid.cell_to_rc(cell)
        return c + 0.5, r + 0.5

    def _edge_segment(self, edge_id: int) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
        e = self.grid.edges[edge_id]
        if e.kind == "V":
            a = (e.col + 1.0, e.row + 0.0)
            b = (e.col + 1.0, e.row + 1.0)
            mid = (e.col + 1.0, e.row + 0.5)
            return a, b, mid
        a = (e.col + 0.0, e.row + 1.0)
        b = (e.col + 1.0, e.row + 1.0)
        mid = (e.col + 0.5, e.row + 1.0)
        return a, b, mid

    def _front_coordinates(self, cell: int, heading: Heading, point: Tuple[float, float]) -> Tuple[float, float]:
        ox, oy = self._cell_center(cell)
        dx, dy = point[0] - ox, point[1] - oy
        if heading == Heading.E:
            return dx, dy
        if heading == Heading.W:
            return -dx, -dy
        if heading == Heading.S:
            return dy, -dx
        return -dy, dx  # Heading.N

    def _in_front_3x3(self, cell: int, heading: Heading, edge_id: int) -> bool:
        _, _, mid = self._edge_segment(edge_id)
        forward, lateral = self._front_coordinates(cell, heading, mid)
        return 0.05 < forward <= 3.5 and abs(lateral) <= 1.5

    @staticmethod
    def _orient(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> int:
        v = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if abs(v) < 1e-10:
            return 0
        return 1 if v > 0 else -1

    @staticmethod
    def _on_segment(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> bool:
        return (
            min(a[0], b[0]) - 1e-10 <= c[0] <= max(a[0], b[0]) + 1e-10
            and min(a[1], b[1]) - 1e-10 <= c[1] <= max(a[1], b[1]) + 1e-10
        )

    @classmethod
    def _segments_intersect(
        cls,
        a: Tuple[float, float],
        b: Tuple[float, float],
        c: Tuple[float, float],
        d: Tuple[float, float],
    ) -> bool:
        o1 = cls._orient(a, b, c)
        o2 = cls._orient(a, b, d)
        o3 = cls._orient(c, d, a)
        o4 = cls._orient(c, d, b)

        if o1 != o2 and o3 != o4:
            return True
        if o1 == 0 and cls._on_segment(a, b, c):
            return True
        if o2 == 0 and cls._on_segment(a, b, d):
            return True
        if o3 == 0 and cls._on_segment(c, d, a):
            return True
        if o4 == 0 and cls._on_segment(c, d, b):
            return True
        return False

    def _compute_occluders(self, cell: int, target_edge: int) -> List[int]:
        origin = self._cell_center(cell)
        _, _, target_mid = self._edge_segment(target_edge)
        occluders = []

        for eid in range(len(self.grid.edges)):
            if eid == target_edge:
                continue
            a, b, _ = self._edge_segment(eid)
            if self._segments_intersect(origin, target_mid, a, b):
                occluders.append(eid)
        return occluders

    def observe_with_truth(
        self,
        cell: int,
        heading: Heading,
        true_blocked_edges: Set[int],
        force_edges: Optional[Iterable[int]] = None,
    ) -> List[Tuple[int, EdgeState]]:
        """
        Simulate sensor observation from true blocked edges.

        Returns a list of (edge_id, observed_state) pairs.
        """
        result: List[Tuple[int, EdgeState]] = []

        def visible(edge_id: int) -> bool:
            occ = self.occluders.get((cell, heading, edge_id), ())
            return all(o not in true_blocked_edges for o in occ)

        candidates = list(self.candidates[(cell, heading)])
        if force_edges is not None:
            for eid in force_edges:
                if eid not in candidates:
                    candidates.append(eid)

        for eid in candidates:
            if visible(eid):
                st = EdgeState.BLOCKED if eid in true_blocked_edges else EdgeState.FREE
                result.append((eid, st))
        return result


class MazeNavigator:
    """
    High-level navigation policy for the user's task.

    It combines:
        - EdgeGrid: edge-state map
        - VisibilityModel: front 3×3 scan model
        - DStarLitePlanner: incremental path repair

    For ROS:
        - call decide_next_action() to decide SCAN/MOVE/REPAIR/DONE;
        - call apply_observations() after sensor processing;
        - call confirm_move() after the robot reaches the next cell center.
    """

    def __init__(
        self,
        start: int = 1,
        goals: Sequence[int] = (38,),
        unknown_cost: float = 1.2,
    ) -> None:
        if not goals:
            raise ValueError("goals must not be empty")

        self.grid = EdgeGrid(unknown_cost=unknown_cost)
        self.visibility = VisibilityModel(self.grid)

        self.start = self.grid.validate_cell(start)
        self.current = self.start
        self.goals = [self.grid.validate_cell(g) for g in goals]
        self.goal_index = 0
        self.heading = Heading.E

        self.planner = DStarLitePlanner(self.grid, self.current, self.current_goal)
        self.changed_edges: Set[int] = set()
        self.last_observed_edges: Set[int] = set()

    @property
    def current_goal(self) -> int:
        return self.goals[self.goal_index]

    @property
    def done(self) -> bool:
        return self.current == self.current_goal and self.goal_index == len(self.goals) - 1

    def reset_known_map(self) -> None:
        self.grid.reset_states(EdgeState.UNKNOWN)
        self.current = self.start
        self.goal_index = 0
        self.heading = Heading.E
        self.changed_edges.clear()
        self.last_observed_edges.clear()
        self.planner.reset_goal(self.current, self.current_goal)

    def set_task(self, start: int, goals: Sequence[int]) -> None:
        if not goals:
            raise ValueError("goals must not be empty")
        self.start = self.grid.validate_cell(start)
        self.current = self.start
        self.goals = [self.grid.validate_cell(g) for g in goals]
        self.goal_index = 0
        self.heading = Heading.E
        self.reset_known_map()

    def _advance_goal_if_reached(self) -> bool:
        if self.current == self.current_goal and self.goal_index < len(self.goals) - 1:
            self.goal_index += 1
            self.planner.reset_goal(self.current, self.current_goal)
            return True
        return False

    def current_path(self) -> List[int]:
        return self.planner.get_path()

    def decide_next_action(self) -> Tuple[str, Optional[int], str]:
        """
        Return (action, next_cell, message).

        action ∈ {"DONE", "PLAN", "SCAN", "MOVE", "NO_PATH"}.
        """
        if self.done:
            return "DONE", None, "All goals reached."

        if self._advance_goal_if_reached():
            return "PLAN", None, f"Switched to next goal {self.current_goal}; D* Lite reset for new goal."

        path = self.current_path()
        if len(path) < 2:
            # D* Lite has already computed after initialization/updates, but this
            # call is cheap and makes the method robust after external changes.
            self.planner.compute_shortest_path()
            path = self.current_path()
            if len(path) < 2:
                return "NO_PATH", None, "No path under current known blocked edges."
            return "PLAN", path[1], "Candidate path repaired/available."

        next_cell = path[1]
        eid = self.grid.edge_id_between(self.current, next_cell)
        st = self.grid.state[eid]

        if st == EdgeState.UNKNOWN:
            self.heading = self.grid.direction_between(self.current, next_cell)
            return "SCAN", next_cell, "Next edge is unknown; rotate toward it and scan front 3×3."

        if st == EdgeState.BLOCKED:
            self.planner.repair_after_edge_changes([eid])
            return "PLAN", None, "Next edge is blocked; D* Lite repaired the path."

        return "MOVE", next_cell, "Next edge is certified free; move one cell."

    def apply_observations(self, observations: Iterable[Tuple[int, EdgeState]]) -> Set[int]:
        """
        Apply sensor observations.

        Returns the set of edges whose traversal cost actually changed.
        """
        changed: Set[int] = set()
        observed: Set[int] = set()

        for eid, new_state in observations:
            if eid < 0 or eid >= len(self.grid.edges):
                raise ValueError(f"invalid edge id: {eid}")
            if new_state not in (EdgeState.FREE, EdgeState.BLOCKED):
                raise ValueError("observations must be FREE or BLOCKED")

            old_cost, new_cost = self.grid.set_edge_state(eid, new_state)
            observed.add(eid)
            if abs(old_cost - new_cost) > EPS:
                changed.add(eid)

        self.last_observed_edges = observed
        self.changed_edges = changed

        if changed:
            self.planner.repair_after_edge_changes(changed)
        return changed

    def scan_with_truth(self, true_blocked_edges: Set[int]) -> Set[int]:
        """
        Simulation-only scan using the true obstacle set.

        In real ROS use, replace this with actual sensor observations and call
        apply_observations().
        """
        path = self.current_path()
        force_edges: List[int] = []
        if len(path) >= 2:
            force_edges.append(self.grid.edge_id_between(path[0], path[1]))

        observations = self.visibility.observe_with_truth(
            self.current,
            self.heading,
            true_blocked_edges,
            force_edges=force_edges,
        )
        return self.apply_observations(observations)

    def confirm_move(self, next_cell: int) -> None:
        """Confirm that the robot physically moved into next_cell."""
        self.grid.validate_cell(next_cell)
        eid = self.grid.edge_id_between(self.current, next_cell)

        if self.grid.state[eid] != EdgeState.FREE:
            raise RuntimeError("cannot move through an edge that is not certified FREE")

        old = self.current
        self.current = next_cell
        self.heading = self.grid.direction_between(old, next_cell)
        self.planner.move_start(self.current)
        self._advance_goal_if_reached()

    def step_simulation(self, true_blocked_edges: Set[int]) -> StepResult:
        """
        Unified one-step simulation:
            PLAN/SCAN/MOVE/DONE are chosen automatically.
        """
        action, next_cell, msg = self.decide_next_action()

        if action == "SCAN":
            self.scan_with_truth(true_blocked_edges)
            path = self.current_path()
            return StepResult(
                action="SCAN",
                current=self.current,
                goal=self.current_goal,
                heading=self.heading,
                path=path,
                changed_edges=sorted(self.changed_edges),
                affected_vertices=sorted(self.planner.affected_vertices),
                message=f"{msg} Observed {len(self.last_observed_edges)} edges; changed {len(self.changed_edges)} costs.",
            )

        if action == "MOVE" and next_cell is not None:
            self.confirm_move(next_cell)
            return StepResult(
                action="MOVE",
                current=self.current,
                goal=self.current_goal,
                heading=self.heading,
                path=self.current_path(),
                changed_edges=[],
                affected_vertices=sorted(self.planner.affected_vertices),
                message=f"Moved to {self.current}.",
            )

        if action in ("PLAN", "NO_PATH", "DONE"):
            return StepResult(
                action=action,
                current=self.current,
                goal=self.current_goal,
                heading=self.heading,
                path=self.current_path(),
                changed_edges=sorted(self.changed_edges),
                affected_vertices=sorted(self.planner.affected_vertices),
                message=msg,
            )

        raise RuntimeError(f"unexpected action: {action}")


def is_connected_under_truth(grid: EdgeGrid, start: int, goal: int, blocked: Set[int]) -> bool:
    q = [start]
    seen = {start}
    while q:
        u = q.pop(0)
        if u == goal:
            return True
        for v, eid, _ in grid.neighbors[u]:
            if eid in blocked or v in seen:
                continue
            seen.add(v)
            q.append(v)
    return False


def mission_connected_under_truth(grid: EdgeGrid, start: int, goals: Sequence[int], blocked: Set[int]) -> bool:
    pts = [start] + list(goals)
    for a, b in zip(pts[:-1], pts[1:]):
        if not is_connected_under_truth(grid, a, b, blocked):
            return False
    return True


def generate_random_obstacles(
    grid: EdgeGrid,
    start: int,
    goals: Sequence[int],
    total: int = 20,
    seed: Optional[int] = None,
    protect_cells: Sequence[int] = (1, 9),
) -> Set[int]:
    """
    Generate a sparse random true-obstacle set.

    Constraints:
        - about `total` internal blocked edges;
        - no protected-cell incident edge is blocked;
        - no cell has more than two incident blocked edges;
        - prefer non-touching obstacles in the early fill phase;
        - ensure start→goals are truly connected.
    """
    rng = random.Random(seed)
    all_eids = list(range(len(grid.edges)))

    banned: Set[int] = set()
    for c in protect_cells:
        if 1 <= c <= 81:
            banned.update(grid.incident_edges(c))

    for _attempt in range(500):
        chosen: Set[int] = set()
        incident_count: Dict[int, int] = {}

        shuffled = all_eids[:]
        rng.shuffle(shuffled)

        for eid in shuffled:
            if len(chosen) >= total:
                break
            if eid in banned:
                continue

            u, v = grid.edge_cells(eid)
            if incident_count.get(u, 0) >= 2 or incident_count.get(v, 0) >= 2:
                continue

            # During most of the generation process, avoid obstacles sharing a
            # cell with existing obstacles. This makes obstacles less continuous.
            touches_existing = False
            for old in chosen:
                a, b = grid.edge_cells(old)
                if u in (a, b) or v in (a, b):
                    touches_existing = True
                    break
            if touches_existing and len(chosen) < max(0, total - 3):
                continue

            chosen.add(eid)
            incident_count[u] = incident_count.get(u, 0) + 1
            incident_count[v] = incident_count.get(v, 0) + 1

        if len(chosen) == total and mission_connected_under_truth(grid, start, goals, chosen):
            return chosen

    raise RuntimeError("failed to generate a valid sparse obstacle map; try another seed")


def run_demo(start: int, goals: Sequence[int], seed: Optional[int], max_steps: int = 200) -> None:
    nav = MazeNavigator(start=start, goals=goals)
    true_blocked = generate_random_obstacles(nav.grid, start, goals, seed=seed)

    print(f"Task: {start} -> {' -> '.join(map(str, goals))}")
    print(f"True blocked edges: {len(true_blocked)}")
    print("Running unified simulation...\n")

    for i in range(max_steps):
        res = nav.step_simulation(true_blocked)
        path_preview = "->".join(map(str, res.path[:12]))
        if len(res.path) > 12:
            path_preview += "->..."
        print(
            f"[{i:03d}] {res.action:6s} "
            f"cell={res.current:2d} goal={res.goal:2d} "
            f"head={DIR_ARROW[res.heading]} "
            f"changed={len(res.changed_edges):2d} "
            f"affected={len(res.affected_vertices):2d} "
            f"path={path_preview or '-'} | {res.message}"
        )
        if res.action == "DONE":
            return

    print("\nStopped by max_steps; check obstacle map or parameters.")


def parse_goals(text: str) -> List[int]:
    goals = [int(x.strip()) for x in text.split(",") if x.strip()]
    if not goals:
        raise argparse.ArgumentTypeError("goals must not be empty")
    for g in goals:
        if g < 1 or g > 81:
            raise argparse.ArgumentTypeError("each goal must be in 1..81")
    return goals


def main() -> None:
    parser = argparse.ArgumentParser(description="9×9 edge-state D* Lite maze demo")
    parser.add_argument("--start", type=int, default=1, help="start cell in 1..81")
    parser.add_argument("--goals", type=parse_goals, default=[38], help="comma-separated goals, e.g. 14,44,68,38")
    parser.add_argument("--seed", type=int, default=7, help="random seed for simulated obstacle map")
    parser.add_argument("--max-steps", type=int, default=200)
    args = parser.parse_args()

    run_demo(args.start, args.goals, seed=args.seed, max_steps=args.max_steps)


if __name__ == "__main__":
    main()

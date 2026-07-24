from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Callable, Optional

from warehouse.model.grid import Coord, Grid
from warehouse.planning.reservation import ReservationTable


@dataclass(frozen=True)
class GoalSpec:
    is_goal: Callable[[Coord], bool]
    heuristic: Callable[[Coord], int]  # admissible: never overestimates remaining steps
    # The finite set of cells that could ever satisfy is_goal, when known and
    # small (adjacent_to: <=4; any_row: grid.width) -- lets plan() cheaply
    # detect a currently-hopeless goal (every candidate permanently blocked)
    # without exhausting max_expansions on a search that can't succeed at any
    # tick. None for goals without a small enumerable candidate set.
    candidates: Optional[tuple[Coord, ...]] = None


def adjacent_to(pos: Coord, grid: Grid, exclude_offsets: frozenset[Coord] = frozenset()) -> GoalSpec:
    """Goal: any in-bounds orthogonal neighbor of `pos` (used to get next to a
    pallet before picking/docking), optionally excluding specific approach
    directions (offset from `pos` to the candidate cell).

    `exclude_offsets` matters for replenishment specifically: docking from the
    cell north of a pallet (offset (0,-1)) attaches the pallet to the robot's
    *south* side -- and a pallet docked to a robot's south side can never
    reach the replenishment row (y = height-1), since the pallet would need
    to sit one row past it, out of bounds. The replenish-docking approach
    goal excludes that one direction; plain pick-adjacency doesn't need to."""
    excluded_cells = {pos + off for off in exclude_offsets}
    targets = frozenset(c for c in grid.neighbors4(pos) if c not in excluded_cells)

    def is_goal(c: Coord) -> bool:
        return c in targets

    def heuristic(c: Coord) -> int:
        return max(0, abs(c.x - pos.x) + abs(c.y - pos.y) - 1)

    return GoalSpec(is_goal=is_goal, heuristic=heuristic, candidates=tuple(targets))


def any_row(y: int, grid: Grid) -> GoalSpec:
    """Goal: reach the given row at any x (used for `y=0` fulfillment and
    `y=height-1` replenishment, whose coordinates are otherwise unconstrained)."""

    def is_goal(c: Coord) -> bool:
        return c.y == y

    def heuristic(c: Coord) -> int:
        return abs(c.y - y)

    candidates = tuple(Coord(x, y) for x in range(grid.width))
    return GoalSpec(is_goal=is_goal, heuristic=heuristic, candidates=candidates)


def exact_cell(target: Coord) -> GoalSpec:
    """Goal: reach one specific cell (used to drag a docked pallet back to
    exactly where it started once a replenishment trip finishes, so it never
    permanently clutters the replenishment row)."""

    def is_goal(c: Coord) -> bool:
        return c == target

    def heuristic(c: Coord) -> int:
        return abs(c.x - target.x) + abs(c.y - target.y)

    return GoalSpec(is_goal=is_goal, heuristic=heuristic, candidates=(target,))


def _footprint(coord: Coord, offsets: list[Coord]) -> list[Coord]:
    return [coord] + [coord + off for off in offsets]


def _node_valid(
    coord: Coord,
    t: int,
    offsets: list[Coord],
    reservation_table: ReservationTable,
    ignore_cells: frozenset[Coord],
    grid: Grid,
) -> bool:
    for cell in _footprint(coord, offsets):
        if not grid.in_bounds(cell):
            return False
        if not reservation_table.is_free(cell, t, ignore_cells):
            return False
    return True


def _edge_valid(
    old_coord: Coord,
    new_coord: Coord,
    t: int,
    offsets: list[Coord],
    reservation_table: ReservationTable,
) -> bool:
    old_footprint = _footprint(old_coord, offsets)
    new_footprint = _footprint(new_coord, offsets)
    for old_cell, new_cell in zip(old_footprint, new_footprint):
        if not reservation_table.is_edge_free(old_cell, new_cell, t):
            return False
    return True


def _has_any_reachable_candidate(
    candidates: tuple[Coord, ...],
    dock_offsets: list[Coord],
    reservation_table: ReservationTable,
    ignore_cells: frozenset[Coord],
    robot_id: int,
) -> bool:
    """Cheap, time-independent check: is there at least one candidate goal
    cell whose footprint isn't permanently blocked (static_holds/
    settled_holds -- neither is scoped to a specific tick, so if every
    candidate is blocked by one of these right now, no amount of searching or
    waiting will ever find a path; only another robot moving away can change
    that). This exists to avoid burning max_expansions, every single tick,
    on a goal that's provably hopeless right now -- e.g. a pallet whose only
    physically-reachable adjacent cell is currently settled by another robot
    for an extended pick session."""
    def cell_reachable(cell: Coord) -> bool:
        if cell in ignore_cells:
            return True
        if cell in reservation_table.static_holds:
            return False
        settled_by = reservation_table.settled_holds.get(cell)
        return settled_by is None or settled_by == robot_id

    for candidate in candidates:
        if all(cell_reachable(cell) for cell in _footprint(candidate, dock_offsets)):
            return True
    return False


def _reconstruct(came_from: dict[tuple[Coord, int], tuple[Coord, int]], end_key: tuple[Coord, int]) -> list[Coord]:
    path = []
    key = end_key
    while key in came_from:
        path.append(key[0])
        key = came_from[key]
    path.append(key[0])
    path.reverse()
    return path


def plan(
    robot_id: int,
    start: Coord,
    start_t: int,
    goal: GoalSpec,
    dock_offsets: list[Coord],
    reservation_table: ReservationTable,
    grid: Grid,
    max_expansions: int = 20_000,
) -> Optional[list[Coord]]:
    """Prioritized space-time A* for one robot (plus any pallets docked to
    it, riding along as a rigid multi-cell footprint). Returns the sequence of
    coordinates from `start` (inclusive) to a goal cell (inclusive) — one
    entry per tick, where a repeated coordinate is a wait — or None if no
    path is found within `max_expansions`.

    A candidate goal cell is only accepted once it's confirmed free not just
    at the arrival tick but indefinitely afterward (see
    ReservationTable.is_free_indefinitely): the robot intends to settle there
    for an open-ended stay (picking, waiting to dock, etc.), so accepting a
    cell that's merely free *at that instant* would let it later collide with
    a different robot's path that was reserved long before this one ever
    arrived. If the nearest candidate fails that check, the search simply
    keeps going — waiting, or trying a different candidate cell.

    This is a pure search: it does NOT write to `reservation_table`. The
    caller must call `reservation_table.reserve_path(...)` immediately after
    accepting the result, before any other robot is planned, to preserve the
    table's whole-fleet invariant.
    """
    ignore_cells = frozenset(_footprint(start, dock_offsets))

    if goal.candidates is not None and not _has_any_reachable_candidate(
        goal.candidates, dock_offsets, reservation_table, ignore_cells, robot_id
    ):
        return None

    counter = 0
    open_heap: list[tuple[int, int, int, Coord, int]] = [(goal.heuristic(start), 0, counter, start, start_t)]
    came_from: dict[tuple[Coord, int], tuple[Coord, int]] = {}
    best_g: dict[tuple[Coord, int], int] = {(start, start_t): 0}
    expansions = 0

    while open_heap:
        _, g, _, coord, t = heapq.heappop(open_heap)
        if best_g.get((coord, t)) != g:
            continue  # stale queue entry, a cheaper path to this node was already found
        if goal.is_goal(coord) and reservation_table.is_free_indefinitely(
            _footprint(coord, dock_offsets), t, ignore_cells, robot_id
        ):
            return _reconstruct(came_from, (coord, t))

        expansions += 1
        if expansions > max_expansions:
            return None

        for next_coord in [coord] + list(grid.neighbors4(coord)):
            next_t = t + 1
            if not _node_valid(next_coord, next_t, dock_offsets, reservation_table, ignore_cells, grid):
                continue
            if next_coord != coord and not _edge_valid(coord, next_coord, next_t, dock_offsets, reservation_table):
                continue
            next_g = g + 1
            key = (next_coord, next_t)
            if next_g < best_g.get(key, float("inf")):
                best_g[key] = next_g
                came_from[key] = (coord, t)
                counter += 1
                heapq.heappush(open_heap, (next_g + goal.heuristic(next_coord), next_g, counter, next_coord, next_t))

    return None

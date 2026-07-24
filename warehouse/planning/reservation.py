from __future__ import annotations

from typing import Optional

from warehouse.model.grid import Coord
from warehouse.model.world import EntityRef, WorldState


class ReservationTable:
    """The universal, shared spatio-temporal costmap every robot's planner
    reads from and writes to — there is exactly one instance for the whole
    fleet, never a per-robot view.

    Every entity is, at any instant, in one of three states, all tracked in
    this same shared table: "parked with no active plan" (idle robot, resting
    pallet — held in `static_holds`, blocking its cell for all future t,
    recomputed fresh every tick from real world occupancy); "in motion under
    an active committed plan" (its *entire* future path is written into
    `vertex`/`edge` the moment it's planned, however far ahead that is); or
    "settled at the end of an active committed plan" (`settled_holds` —  see
    below). This is what guarantees no two robots can ever end up planning to
    occupy the same cell at the same timestep, even when one plan was
    committed long before the other: every new `plan()` call queries this
    same table before committing, and writes its own path back in
    immediately, so no other planning call ever sees a stale view.

    `settled_holds` exists because a committed path only reserves the *exact*
    ticks explicitly in it — once a robot arrives and settles in for an
    open-ended stay (picking many times, waiting to dock), nothing reserves
    the ticks *after* arrival until `sync_static_holds` notices it next tick,
    from real occupancy. That leaves a window: a robot planned long before
    (reserving some far-future tick at that same cell) would never have seen
    a conflict, since at planning time nothing was there yet. So the moment
    ANY plan is committed, its destination cell is also marked in
    `settled_holds`, immediately and unconditionally (not time-scoped) — a
    plan-time analogue of `static_holds`, cleared by `release_robot` the
    moment that robot needs to move again.
    """

    def __init__(self) -> None:
        self.static_holds: dict[Coord, EntityRef] = {}
        self.settled_holds: dict[Coord, int] = {}
        self.vertex: dict[tuple[Coord, int], int] = {}
        self.edge: dict[tuple[Coord, Coord, int], int] = {}

    def sync_static_holds(self, world: WorldState, planned_robot_ids: set[int]) -> None:
        """Rebuild static holds from scratch from `world.occupancy`, every
        tick (trivial at this problem's scale — not worth incremental
        bookkeeping). Cells belonging to a robot in `planned_robot_ids` (or a
        pallet currently docked to one) are excluded: those are represented by
        that robot's timed reservations instead, not a static hold."""
        self.static_holds = {}
        for coord, ref in world.occupancy.items():
            kind, entity_id = ref
            if kind == "robot" and entity_id in planned_robot_ids:
                continue
            if kind == "pallet":
                pallet = world.pallets[entity_id]
                if pallet.docked_to is not None and pallet.docked_to in planned_robot_ids:
                    continue
            self.static_holds[coord] = ref

    def is_free(self, coord: Coord, t: int, ignore_cells: frozenset[Coord] = frozenset()) -> bool:
        if coord not in ignore_cells:
            if coord in self.static_holds:
                return False
            if coord in self.settled_holds:
                return False
        return (coord, t) not in self.vertex

    def is_free_indefinitely(
        self,
        cells: list[Coord],
        from_t: int,
        ignore_cells: frozenset[Coord] = frozenset(),
        ignore_robot: Optional[int] = None,
    ) -> bool:
        """True if every cell in `cells` is free not just at `from_t` but for
        every tick from `from_t` onward (ignoring `ignore_robot`'s own
        reservations/holds). A robot planning to *settle* somewhere for an
        open-ended stay must use this, not `is_free`, when accepting a goal —
        otherwise it could settle into a cell someone else already reserved
        for some later tick, or already settled at, silently violating the
        table's whole-fleet invariant once that later tick comes due."""
        cell_set = set(cells)
        for cell in cell_set:
            if cell in ignore_cells:
                continue
            if cell in self.static_holds:
                return False
            settled_by = self.settled_holds.get(cell)
            if settled_by is not None and settled_by != ignore_robot:
                return False
        for (coord, t), rid in self.vertex.items():
            if coord in cell_set and t >= from_t and rid != ignore_robot:
                return False
        return True

    def is_edge_free(self, frm: Coord, to: Coord, t: int) -> bool:
        """False if someone already reserved the exact reverse transition
        (arriving at `frm` from `to` at the same time `t`) — a swap."""
        return (to, frm, t) not in self.edge

    def reserve_path(self, robot_id: int, path: list[Coord], start_t: int, footprint_offsets: list[Coord]) -> None:
        """Write a robot's full committed path — including every cell its
        docked pallets occupy along the way — into the shared table for every
        tick it will occupy them, immediately and in full, not just its next
        step. Also marks its destination footprint in `settled_holds`,
        unconditionally, since the robot intends to remain there for an
        open-ended stay once it arrives (see class docstring)."""
        for i, coord in enumerate(path):
            t = start_t + i
            cells = [coord] + [coord + off for off in footprint_offsets]
            for cell in cells:
                self.vertex[(cell, t)] = robot_id
            if i > 0:
                prev = path[i - 1]
                prev_cells = [prev] + [prev + off for off in footprint_offsets]
                for prev_cell, cur_cell in zip(prev_cells, cells):
                    self.edge[(prev_cell, cur_cell, t)] = robot_id

        final_coord = path[-1]
        for cell in [final_coord] + [final_coord + off for off in footprint_offsets]:
            self.settled_holds[cell] = robot_id

    def release_robot(self, robot_id: int) -> None:
        """Drop a robot's committed reservations and settled hold, e.g. right
        before replanning it (a finished path, a newly assigned goal, or a
        deadlock-fallback global replan)."""
        self.vertex = {k: v for k, v in self.vertex.items() if v != robot_id}
        self.edge = {k: v for k, v in self.edge.items() if v != robot_id}
        self.settled_holds = {c: r for c, r in self.settled_holds.items() if r != robot_id}

    def prune_before(self, tick: int) -> None:
        """Drop reservations for ticks that have already elapsed. Settled
        holds are untouched -- they're not time-indexed and only go away via
        `release_robot`."""
        self.vertex = {(c, t): r for (c, t), r in self.vertex.items() if t >= tick}
        self.edge = {(frm, to, t): r for (frm, to, t), r in self.edge.items() if t >= tick}

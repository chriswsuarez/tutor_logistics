from __future__ import annotations

from collections import Counter, deque
from typing import Optional

from warehouse.model.action import Action
from warehouse.model.grid import NORTH, Coord
from warehouse.model.world import WorldState
from warehouse.planning.cooperative_astar import GoalSpec, adjacent_to, any_row, exact_cell, plan
from warehouse.planning.reservation import ReservationTable
from warehouse.tasks.pallet_selector import PalletSelectionPolicy, nearest_pallet_of_sku
from warehouse.tasks.task_types import CollectSkuSubGoal, DeliverSubGoal, RelocateSubGoal, ReplenishSubGoal, SubGoal

_RETRY_MAX_EXPANSIONS = 200_000
# Approaching a pallet from its north cell docks it to the robot's south side,
# which can then never reach the replenishment row (y = height-1) without the
# pallet going out of bounds. Excluded specifically for replenish docking.
_REPLENISH_DOCK_EXCLUDED_OFFSETS = frozenset({NORTH})
# Consecutive tick-failures to find any path before trying a sidestep (see
# _advance_along_path_to's fallback and the class docstring's deadlock note).
_STUCK_TICKS_BEFORE_SIDESTEP = 15


class RobotController:
    """Per-robot FSM. Re-derives its phase from ground-truth world state each
    tick (storage contents, docking state, position) rather than trusting a
    separately-tracked status flag, so it can never desync from what actually
    happened last tick. The only state it keeps across ticks is its subgoal
    queue, the pallet currently targeted for the front CollectSkuSubGoal, and
    the remaining steps of its current committed path.
    """

    def __init__(self, robot_id: int, pallet_selector: PalletSelectionPolicy):
        self.robot_id = robot_id
        self.pallet_selector = pallet_selector
        self.subgoals: deque[SubGoal] = deque()
        self.pending_path: Optional[list[Coord]] = None
        self._target_pallet_id: Optional[int] = None
        self._stuck_ticks = 0

    def is_idle(self) -> bool:
        return not self.subgoals

    def assign(self, subgoals: deque[SubGoal]) -> None:
        self.subgoals = subgoals
        self.pending_path = None
        self._target_pallet_id = None
        self._stuck_ticks = 0

    def propose_action(
        self,
        world: WorldState,
        reservation_table: ReservationTable,
        claimed_picks: Optional[Counter] = None,
        claimed_docks: Optional[set[int]] = None,
    ) -> Optional[Action]:
        """`claimed_picks`/`claimed_docks` track what OTHER controllers have
        already proposed to pick/dock earlier in this same tick's iteration
        (see SimulationDriver._run_one_tick) -- without them, two robots
        adjacent to the same low-stock pallet (or two robots both starting a
        replenishment trip on the same empty pallet) would each independently
        see the same start-of-tick count/docked_to state and both propose a
        conflicting action every tick, get dropped by the engine's batch
        validation, and retry the identical conflict forever, since neither
        robot's own decision-making ever changes. Checking the shrinking
        shared budget before proposing turns that into "wait one tick," which
        resolves itself once the real state updates."""
        self._drop_satisfied_subgoals(world)
        if not self.subgoals:
            return None

        claimed_picks = claimed_picks if claimed_picks is not None else Counter()
        claimed_docks = claimed_docks if claimed_docks is not None else set()

        subgoal = self.subgoals[0]
        if isinstance(subgoal, CollectSkuSubGoal):
            return self._step_collect(subgoal, world, reservation_table, claimed_picks)
        if isinstance(subgoal, DeliverSubGoal):
            return self._step_deliver(world, reservation_table)
        if isinstance(subgoal, RelocateSubGoal):
            return self._step_relocate(subgoal, world, reservation_table, claimed_docks)
        return self._step_replenish(subgoal, world, reservation_table, claimed_docks)

    def _drop_satisfied_subgoals(self, world: WorldState) -> None:
        robot = world.robots[self.robot_id]
        while self.subgoals:
            front = self.subgoals[0]
            if isinstance(front, CollectSkuSubGoal) and robot.storage[front.sku] >= front.quantity:
                self.subgoals.popleft()
                self.pending_path = None
                self._target_pallet_id = None
            elif isinstance(front, DeliverSubGoal) and sum(robot.storage.values()) == 0:
                # Reached only once storage exactly matched an order (a non-empty
                # multiset, per the format's 30-100-item orders), so storage
                # reading empty here can only mean a fulfill just succeeded.
                self.subgoals.popleft()
                self.pending_path = None
            else:
                break

    def _step_collect(
        self,
        subgoal: CollectSkuSubGoal,
        world: WorldState,
        reservation_table: ReservationTable,
        claimed_picks: Counter,
    ):
        robot = world.robots[self.robot_id]

        if self._target_pallet_id is not None and world.pallets[self._target_pallet_id].count == 0:
            self._target_pallet_id = None
            self.pending_path = None

        if self._target_pallet_id is None:
            chosen = self.pallet_selector.select(subgoal.sku, robot.position, world)
            if chosen is None:
                # Every pallet of this SKU is currently empty: divert to
                # replenish the nearest one before resuming collection.
                nearest_id = self._nearest_pallet_of_sku(subgoal.sku, robot.position, world)
                if nearest_id is None:
                    return None  # every instance is already being replenished by another robot; wait
                origin = world.pallets[nearest_id].position
                self.subgoals.appendleft(ReplenishSubGoal(pallet_id=nearest_id, resume=subgoal, origin=origin))
                self.pending_path = None
                return self._step_replenish(self.subgoals[0], world, reservation_table, set())
            self._target_pallet_id = chosen

        pallet = world.pallets[self._target_pallet_id]
        if robot.position not in set(world.grid.neighbors4(pallet.position)):
            return self._advance_along_path_to(adjacent_to(pallet.position, world.grid), world, reservation_table)

        if claimed_picks[pallet.id] >= pallet.count:
            return None  # this tick's stock is already spoken for; retry next tick
        claimed_picks[pallet.id] += 1
        return Action("pick", pallet.position.x, pallet.position.y)

    def _step_deliver(self, world: WorldState, reservation_table: ReservationTable):
        robot = world.robots[self.robot_id]
        if not world.grid.is_fulfillment(robot.position):
            return self._advance_along_path_to(any_row(0, world.grid), world, reservation_table)
        return Action("fulfill", 0, 0)

    def _step_replenish(
        self,
        subgoal: ReplenishSubGoal,
        world: WorldState,
        reservation_table: ReservationTable,
        claimed_docks: set[int],
    ):
        robot = world.robots[self.robot_id]
        pallet = world.pallets[subgoal.pallet_id]

        if pallet.docked_to != self.robot_id:
            valid_approach_cells = {
                c for c in world.grid.neighbors4(pallet.position) if c != pallet.position + NORTH
            }
            if robot.position not in valid_approach_cells:
                return self._advance_along_path_to(
                    adjacent_to(pallet.position, world.grid, exclude_offsets=_REPLENISH_DOCK_EXCLUDED_OFFSETS),
                    world,
                    reservation_table,
                )
            if pallet.id in claimed_docks:
                return None  # someone else is already docking this pallet this tick; retry next tick
            claimed_docks.add(pallet.id)
            return Action("dock", pallet.position.x, pallet.position.y)

        if pallet.count < pallet.max_count:
            # Not yet refilled: this branch (not "is robot home yet") must be
            # the gate here, not robot.position -- once refilled, the return
            # trip moves the robot *off* the replenishment row again, and a
            # position-based check would send it right back, forever.
            replenishment_row = world.grid.height - 1
            if not world.grid.is_replenishment(robot.position):
                return self._advance_along_path_to(any_row(replenishment_row, world.grid), world, reservation_table)
            return None  # arrived this tick; the engine's auto-refill applies at tick end

        # Refilled: drag it back to where it came from before releasing it.
        # Leaving it parked on the replenishment row would, over many trips,
        # accumulate into a wall of pallets that can wedge others (or itself,
        # next time) permanently out of reach -- see _nearest_pallet_of_sku.
        offset = next(off for off, pid in robot.docked_pallets.items() if pid == subgoal.pallet_id)
        target_robot_pos = Coord(subgoal.origin.x - offset.x, subgoal.origin.y - offset.y)
        if robot.position != target_robot_pos:
            return self._advance_along_path_to(exact_cell(target_robot_pos), world, reservation_table)

        self.subgoals.popleft()
        self.subgoals.appendleft(subgoal.resume)
        self.pending_path = None
        self._target_pallet_id = None
        return Action("undock", pallet.position.x, pallet.position.y)

    def _step_relocate(
        self,
        subgoal: RelocateSubGoal,
        world: WorldState,
        reservation_table: ReservationTable,
        claimed_docks: set[int],
    ):
        robot = world.robots[self.robot_id]
        pallet = world.pallets[subgoal.pallet_id]

        if pallet.position == subgoal.target and pallet.docked_to is None:
            self.subgoals.popleft()
            self.pending_path = None
            return None

        if pallet.docked_to != self.robot_id:
            if robot.position not in set(world.grid.neighbors4(pallet.position)):
                return self._advance_along_path_to(adjacent_to(pallet.position, world.grid), world, reservation_table)
            if pallet.id in claimed_docks:
                return None  # someone else is already docking this pallet this tick; retry next tick
            claimed_docks.add(pallet.id)
            return Action("dock", pallet.position.x, pallet.position.y)

        # Relocation targets sit deep inside the grid (y in [1, R], nowhere near
        # either y=0 or y=height-1), so unlike replenish-docking there is no
        # direction whose resulting undock position could ever land out of
        # bounds -- no exclude_offsets needed for the approach above.
        offset = next(off for off, pid in robot.docked_pallets.items() if pid == subgoal.pallet_id)
        target_robot_pos = Coord(subgoal.target.x - offset.x, subgoal.target.y - offset.y)
        if robot.position != target_robot_pos:
            return self._advance_along_path_to(exact_cell(target_robot_pos), world, reservation_table)

        self.subgoals.popleft()
        self.pending_path = None
        self._target_pallet_id = None
        return Action("undock", pallet.position.x, pallet.position.y)

    def _advance_along_path_to(
        self, goal_spec: GoalSpec, world: WorldState, reservation_table: ReservationTable
    ) -> Optional[Action]:
        robot = world.robots[self.robot_id]

        if self.pending_path is None:
            dock_offsets = list(robot.docked_pallets.keys())
            reservation_table.release_robot(self.robot_id)
            path = plan(self.robot_id, robot.position, world.tick, goal_spec, dock_offsets, reservation_table, world.grid)
            if path is None:
                path = plan(
                    self.robot_id,
                    robot.position,
                    world.tick,
                    goal_spec,
                    dock_offsets,
                    reservation_table,
                    world.grid,
                    max_expansions=_RETRY_MAX_EXPANSIONS,
                )
            if path is None:
                # Genuinely no path right now -- possibly a real mutual
                # deadlock: e.g. two robots each parked in the other's only
                # reachable pallet-adjacency cell, in a cluster too dense for
                # either to route around. Neither robot's own goal-directed
                # planning would ever resolve that on its own (the blocker
                # never has a reason to move). After enough consecutive
                # failures, step aside to any free neighboring cell -- this
                # doesn't need to know *why* it's stuck, just increases the
                # chance of unblocking whatever it's inadvertently blocking.
                self._stuck_ticks += 1
                if self._stuck_ticks >= _STUCK_TICKS_BEFORE_SIDESTEP:
                    sidestep = self._find_sidestep(dock_offsets, reservation_table, world)
                    if sidestep is not None:
                        reservation_table.reserve_path(
                            self.robot_id, [robot.position, sidestep], world.tick, dock_offsets
                        )
                        self._stuck_ticks = 0
                        return Action("move", sidestep.x, sidestep.y)
                return None  # genuinely stuck this tick; the driver's watchdog handles prolonged stalls
            self._stuck_ticks = 0
            reservation_table.reserve_path(self.robot_id, path, world.tick, dock_offsets)
            self.pending_path = path

        if len(self.pending_path) <= 1:
            self.pending_path = None
            return None

        next_coord = self.pending_path[1]
        self.pending_path = self.pending_path[1:]
        if len(self.pending_path) <= 1:
            # Just emitted the move that reaches the goal cell -- clear now so
            # the *next* leg's first call sees pending_path as None (needing a
            # fresh plan) rather than a stale single-cell leftover from this one.
            self.pending_path = None
        return Action("move", next_coord.x, next_coord.y)

    def _find_sidestep(
        self, dock_offsets: list[Coord], reservation_table: ReservationTable, world: WorldState
    ) -> Optional[Coord]:
        """Any free neighboring cell the robot (with its docked footprint)
        could step into for one tick, or None if truly boxed in on every
        side. Used only as a last-resort deadlock-breaker (see caller)."""
        robot = world.robots[self.robot_id]
        next_t = world.tick + 1
        own_footprint = frozenset([robot.position] + [robot.position + off for off in dock_offsets])
        for candidate in world.grid.neighbors4(robot.position):
            footprint = [candidate] + [candidate + off for off in dock_offsets]
            if not all(world.grid.in_bounds(c) for c in footprint):
                continue
            if not all(reservation_table.is_free(c, next_t, own_footprint) for c in footprint):
                continue
            if not reservation_table.is_edge_free(robot.position, candidate, next_t):
                continue
            return candidate
        return None

    def _nearest_pallet_of_sku(self, sku: int, from_coord: Coord, world: WorldState) -> Optional[int]:
        """Nearest pallet of `sku`, regardless of stock (used only for the
        replenish-fallback, where every instance is currently empty),
        preferring one with a structurally reachable side excluding north
        (see _REPLENISH_DOCK_EXCLUDED_OFFSETS and has_reachable_neighbor).
        Without this, a pallet that prior replenishment trips happened to
        strand somewhere permanently unreachable (e.g. wedged between two
        other pallets at the grid edge) would deadlock every future order
        needing that SKU forever, even when a farther but reachable pallet
        of the same SKU exists. Also excludes any pallet already docked to
        another robot (already being replenished) with no fallback -- see
        `nearest_pallet_of_sku`'s `exclude_docked` docstring for why;
        returns None (caller waits) if every instance is currently spoken
        for."""
        return nearest_pallet_of_sku(
            sku,
            from_coord,
            world,
            require_stock=False,
            require_reachable=True,
            excluded_offsets=_REPLENISH_DOCK_EXCLUDED_OFFSETS,
            exclude_docked=True,
        )

    def reset_path(self) -> None:
        """Drop the current committed path so the next tick replans from
        scratch. Used by the driver's global deadlock-fallback."""
        self.pending_path = None

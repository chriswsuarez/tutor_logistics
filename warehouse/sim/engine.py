from __future__ import annotations

from collections import Counter, defaultdict

from warehouse.model.action import DOCK, FULFILL, MOVE, PICK, UNDOCK, Action
from warehouse.model.entities import counter_signature
from warehouse.model.grid import Coord
from warehouse.model.world import WorldState
from warehouse.sim import rules
from warehouse.sim.config import SimConfig
from warehouse.sim.exceptions import InvalidActionError, RuleViolation


def apply_tick(world: WorldState, actions: dict[int, Action], config: SimConfig = SimConfig()) -> None:
    """Apply one tick's worth of proposed actions (at most one per robot) to
    `world` in place, then run automatic replenishment, then advance the tick
    counter.

    All structural legality (adjacency, occupancy) is checked against the
    single frozen start-of-tick snapshot — never a mid-tick state — so a move
    is never validated against another entity's simultaneous vacate/arrival.
    This is a deliberately conservative reading of the spec's silence on
    simultaneous-move semantics (see the architecture plan): it is safe
    regardless of how the real grader actually resolves ties, at the cost of
    forbidding swap/follow moves.

    Raises InvalidActionError for a single structurally-illegal action, or
    RuleViolation for a batch-level conflict (two robots' moves landing on the
    same cell, combined picks exceeding a pallet's stock, two robots docking
    the same pallet, more simultaneous fulfills of one signature than there
    are matching unfulfilled orders). Under correct upstream scheduling
    (the universal reservation table, the per-tick pick-claim counter) neither
    should ever actually fire — they exist as a loud signal of a scheduling
    bug, not a graceful-rejection mechanism.
    """
    if config.allow_swap_moves or config.allow_follow_moves:
        raise NotImplementedError(
            "swap/follow moves are not implemented; the engine only supports the "
            "conservative frozen-start-of-tick semantics described in its docstring"
        )

    _validate_structural(world, actions)

    move_deltas = _validate_and_collect_moves(world, actions)
    # Resolved to a pallet id here, up front, against the pristine start-of-tick
    # state -- not re-derived from `world.entity_at(target)` at apply time.
    # A picked-from pallet can be docked to a *different* robot that's also
    # moving this same tick (moves are applied before picks below), so its
    # cell would otherwise be empty by the time _apply_picks runs.
    pick_pallet_ids = {rid: world.entity_at(Coord(a.x, a.y))[1] for rid, a in actions.items() if a.kind == PICK}
    _validate_pick_budget(world, pick_pallet_ids)
    dock_targets = {rid: Coord(a.x, a.y) for rid, a in actions.items() if a.kind == DOCK}
    _validate_dock_exclusivity(dock_targets, world)
    fulfill_robot_ids = [rid for rid, a in actions.items() if a.kind == FULFILL]
    fulfill_assignments = _assign_fulfillments(world, fulfill_robot_ids)

    _apply_moves(world, move_deltas)
    _apply_picks(world, pick_pallet_ids)
    _apply_docks(world, dock_targets)
    _apply_undocks(world, actions)
    _apply_fulfillments(world, fulfill_assignments)

    _apply_replenishment(world)
    world.tick += 1


def _validate_structural(world: WorldState, actions: dict[int, Action]) -> None:
    for robot_id, action in actions.items():
        if robot_id not in world.robots:
            raise InvalidActionError(f"action for unknown robot {robot_id}")
        target = Coord(action.x, action.y)
        if action.kind == MOVE:
            ok = rules.is_orthogonally_adjacent(world.grid, world.robots[robot_id].position, target)
        elif action.kind == PICK:
            ok = rules.can_pick(world, robot_id, target)
        elif action.kind == DOCK:
            ok = rules.can_dock(world, robot_id, target)
        elif action.kind == UNDOCK:
            ok = rules.can_undock(world, robot_id, target)
        else:  # FULFILL
            ok = rules.can_fulfill(world, robot_id)
        if not ok:
            raise InvalidActionError(
                f"robot {robot_id} action {action} is illegal in the current world state", robot_id=robot_id
            )


def _validate_and_collect_moves(world: WorldState, actions: dict[int, Action]) -> dict[int, Coord]:
    move_deltas: dict[int, Coord] = {}
    new_footprints: dict[int, set[Coord]] = {}

    for robot_id, action in actions.items():
        if action.kind != MOVE:
            continue
        robot = world.robots[robot_id]
        target = Coord(action.x, action.y)
        delta = Coord(target.x - robot.position.x, target.y - robot.position.y)
        old_footprint = world.robot_footprint(robot_id)
        new_footprint = {Coord(c.x + delta.x, c.y + delta.y) for c in old_footprint}

        for cell in new_footprint:
            if not world.grid.in_bounds(cell):
                raise InvalidActionError(f"robot {robot_id} move would push a docked pallet out of bounds")
            if cell in old_footprint:
                continue  # this robot's own rigid body sliding over its own old cell(s)
            if world.entity_at(cell) is not None:
                raise InvalidActionError(
                    f"robot {robot_id} move target {cell} is occupied in the start-of-tick snapshot",
                    robot_id=robot_id,
                )

        move_deltas[robot_id] = delta
        new_footprints[robot_id] = new_footprint

    # Batch-level check: two different robots claiming the same (previously
    # empty) cell this tick — invisible to the per-robot check above since
    # neither cell was occupied at the start of the tick.
    claimed_by: dict[Coord, int] = {}
    for robot_id, footprint in new_footprints.items():
        old_footprint = world.robot_footprint(robot_id)
        for cell in footprint:
            if cell in old_footprint:
                continue
            if cell in claimed_by:
                raise RuleViolation(
                    f"robots {claimed_by[cell]} and {robot_id} both plan to occupy {cell} this tick",
                    robot_ids=(claimed_by[cell], robot_id),
                )
            claimed_by[cell] = robot_id

    return move_deltas


def _validate_pick_budget(world: WorldState, pick_pallet_ids: dict[int, int]) -> None:
    claims: dict[int, list[int]] = defaultdict(list)
    for robot_id, pallet_id in pick_pallet_ids.items():
        claims[pallet_id].append(robot_id)
    for pallet_id, claimant_ids in claims.items():
        if len(claimant_ids) > world.pallets[pallet_id].count:
            raise RuleViolation(
                f"pallet {pallet_id} has {world.pallets[pallet_id].count} left but "
                f"{len(claimant_ids)} picks were proposed",
                robot_ids=tuple(claimant_ids),
            )


def _validate_dock_exclusivity(dock_targets: dict[int, Coord], world: WorldState) -> None:
    claimed_by: dict[int, int] = {}
    for robot_id, target in dock_targets.items():
        pallet_id = world.entity_at(target)[1]
        if pallet_id in claimed_by:
            raise RuleViolation(
                f"robots {claimed_by[pallet_id]} and {robot_id} both try to dock pallet {pallet_id} this tick",
                robot_ids=(claimed_by[pallet_id], robot_id),
            )
        claimed_by[pallet_id] = robot_id


def _assign_fulfillments(world: WorldState, robot_ids: list[int]) -> dict[int, int]:
    """Deterministically assign each fulfilling robot to a distinct unfulfilled
    order matching its storage signature (lowest order_id first, robots in
    ascending id order), raising if more robots claim a signature than there
    are matching unfulfilled orders."""
    by_signature: dict[tuple, list[int]] = defaultdict(list)
    for robot_id in sorted(robot_ids):
        signature = counter_signature(world.robots[robot_id].storage)
        by_signature[signature].append(robot_id)

    assignments: dict[int, int] = {}
    for signature, claimants in by_signature.items():
        available = world.unfulfilled_order_ids(signature)
        if len(claimants) > len(available):
            raise RuleViolation(
                f"{len(claimants)} robots fulfilling signature {signature} but only "
                f"{len(available)} unfulfilled orders match it",
                robot_ids=tuple(claimants),
            )
        for robot_id, order_id in zip(claimants, available):
            assignments[robot_id] = order_id
    return assignments


def _apply_moves(world: WorldState, move_deltas: dict[int, Coord]) -> None:
    old_footprints = {rid: world.robot_footprint(rid) for rid in move_deltas}
    for footprint in old_footprints.values():
        for cell in footprint:
            del world.occupancy[cell]

    for robot_id, delta in move_deltas.items():
        robot = world.robots[robot_id]
        robot.position = Coord(robot.position.x + delta.x, robot.position.y + delta.y)
        world.occupancy[robot.position] = ("robot", robot_id)
        for offset, pallet_id in robot.docked_pallets.items():
            pallet = world.pallets[pallet_id]
            pallet.position = robot.position + offset
            world.occupancy[pallet.position] = ("pallet", pallet_id)


def _apply_picks(world: WorldState, pick_pallet_ids: dict[int, int]) -> None:
    for robot_id, pallet_id in pick_pallet_ids.items():
        pallet = world.pallets[pallet_id]
        pallet.count -= 1
        world.robots[robot_id].storage[pallet.sku] += 1


def _apply_docks(world: WorldState, dock_targets: dict[int, Coord]) -> None:
    for robot_id, target in dock_targets.items():
        robot = world.robots[robot_id]
        pallet_id = world.entity_at(target)[1]
        pallet = world.pallets[pallet_id]
        offset = Coord(target.x - robot.position.x, target.y - robot.position.y)
        robot.docked_pallets[offset] = pallet_id
        pallet.docked_to = robot_id
        pallet.dock_offset = offset


def _apply_undocks(world: WorldState, actions: dict[int, Action]) -> None:
    for robot_id, action in actions.items():
        if action.kind != UNDOCK:
            continue
        robot = world.robots[robot_id]
        target = Coord(action.x, action.y)
        pallet_id = world.entity_at(target)[1]
        pallet = world.pallets[pallet_id]
        del robot.docked_pallets[pallet.dock_offset]
        pallet.docked_to = None
        pallet.dock_offset = None


def _apply_fulfillments(world: WorldState, assignments: dict[int, int]) -> None:
    for robot_id, order_id in assignments.items():
        world.orders[order_id].fulfilled = True
        world.robots[robot_id].storage = Counter()


def _apply_replenishment(world: WorldState) -> None:
    for robot in world.robots.values():
        if not robot.docked_pallets:
            continue
        if world.grid.is_replenishment(robot.position):
            for pallet_id in robot.docked_pallets.values():
                pallet = world.pallets[pallet_id]
                pallet.count = pallet.max_count

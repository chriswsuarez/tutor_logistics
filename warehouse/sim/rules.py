from __future__ import annotations

from warehouse.model.entities import counter_signature
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState


def is_orthogonally_adjacent(grid: Grid, a: Coord, b: Coord) -> bool:
    """True if b is one of a's four in-bounds orthogonal neighbors."""
    return b in grid.neighbors4(a)


def can_move(world: WorldState, robot_id: int, target: Coord) -> bool:
    """Single-cell move legality for a robot with no docked pallets. Engine's
    full move validation additionally accounts for docked-pallet footprints
    (see warehouse/sim/engine.py) since that needs multi-cell reasoning this
    predicate deliberately does not."""
    robot = world.robots.get(robot_id)
    if robot is None:
        return False
    if not is_orthogonally_adjacent(world.grid, robot.position, target):
        return False
    return world.entity_at(target) is None


def can_pick(world: WorldState, robot_id: int, target: Coord) -> bool:
    robot = world.robots.get(robot_id)
    if robot is None:
        return False
    if not is_orthogonally_adjacent(world.grid, robot.position, target):
        return False
    occupant = world.entity_at(target)
    if occupant is None or occupant[0] != "pallet":
        return False
    pallet = world.pallets[occupant[1]]
    return pallet.count > 0


def can_dock(world: WorldState, robot_id: int, target: Coord) -> bool:
    robot = world.robots.get(robot_id)
    if robot is None:
        return False
    if not is_orthogonally_adjacent(world.grid, robot.position, target):
        return False
    offset = Coord(target.x - robot.position.x, target.y - robot.position.y)
    if offset in robot.docked_pallets:
        return False  # that side is already occupied
    occupant = world.entity_at(target)
    if occupant is None or occupant[0] != "pallet":
        return False
    pallet = world.pallets[occupant[1]]
    return pallet.docked_to is None


def can_undock(world: WorldState, robot_id: int, target: Coord) -> bool:
    if robot_id not in world.robots:
        return False
    occupant = world.entity_at(target)
    if occupant is None or occupant[0] != "pallet":
        return False
    pallet = world.pallets[occupant[1]]
    return pallet.docked_to == robot_id


def can_fulfill(world: WorldState, robot_id: int) -> bool:
    robot = world.robots.get(robot_id)
    if robot is None:
        return False
    if not world.grid.is_fulfillment(robot.position):
        return False
    signature = counter_signature(robot.storage)
    return len(world.unfulfilled_order_ids(signature)) > 0

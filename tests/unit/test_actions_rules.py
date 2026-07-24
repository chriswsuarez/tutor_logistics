from collections import Counter

import pytest

from warehouse.model.action import Action
from warehouse.model.entities import Order, Pallet, Robot
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState
from warehouse.sim.engine import apply_tick
from warehouse.sim.exceptions import InvalidActionError, RuleViolation


def make_world(robots, pallets, orders=None):
    grid = Grid(width=60, height=40)
    return WorldState(
        grid=grid,
        robots={r.id: r for r in robots},
        pallets={p.id: p for p in pallets},
        orders=orders or [],
    )


# ---- move ----

def test_move_onto_occupied_cell_rejected():
    world = make_world(
        [Robot(id=0, position=Coord(5, 5)), Robot(id=1, position=Coord(6, 5))],
        [],
    )
    with pytest.raises(InvalidActionError):
        apply_tick(world, {0: Action("move", 6, 5)})


def test_move_non_adjacent_rejected():
    world = make_world([Robot(id=0, position=Coord(5, 5))], [])
    with pytest.raises(InvalidActionError):
        apply_tick(world, {0: Action("move", 7, 5)})


def test_move_updates_position_and_occupancy():
    world = make_world([Robot(id=0, position=Coord(5, 5))], [])
    apply_tick(world, {0: Action("move", 6, 5)})
    assert world.robots[0].position == Coord(6, 5)
    assert world.occupancy[Coord(6, 5)] == ("robot", 0)
    assert Coord(5, 5) not in world.occupancy


def test_two_robots_moving_into_same_empty_cell_raises_rule_violation():
    world = make_world(
        [Robot(id=0, position=Coord(5, 5)), Robot(id=1, position=Coord(5, 7))],
        [],
    )
    with pytest.raises(RuleViolation):
        apply_tick(world, {0: Action("move", 5, 6), 1: Action("move", 5, 6)})


def test_move_drags_docked_pallet_along():
    world = make_world(
        [Robot(id=0, position=Coord(5, 5))],
        [Pallet(id=0, sku=0, position=Coord(5, 4), count=3, max_count=3, docked_to=0, dock_offset=Coord(0, -1))],
    )
    world.robots[0].docked_pallets[Coord(0, -1)] = 0
    apply_tick(world, {0: Action("move", 6, 5)})
    assert world.robots[0].position == Coord(6, 5)
    assert world.pallets[0].position == Coord(6, 4)
    assert world.occupancy[Coord(6, 4)] == ("pallet", 0)
    assert Coord(5, 4) not in world.occupancy


def test_move_pushing_docked_pallet_out_of_bounds_rejected():
    # Robot at the west edge (x=0) with a pallet already docked to its west
    # side (off-grid at x=-1 is impossible, so the pallet sits east instead,
    # docked as if trailing the robot); moving further west would push that
    # trailing pallet's mirrored offset off-grid.
    world = make_world(
        [Robot(id=0, position=Coord(1, 5))],
        [Pallet(id=0, sku=0, position=Coord(0, 5), count=3, max_count=3, docked_to=0, dock_offset=Coord(-1, 0))],
    )
    world.robots[0].docked_pallets[Coord(-1, 0)] = 0
    with pytest.raises(InvalidActionError):
        apply_tick(world, {0: Action("move", 0, 5)})


# ---- pick ----

def test_pick_decrements_pallet_and_fills_storage():
    world = make_world(
        [Robot(id=0, position=Coord(5, 5))],
        [Pallet(id=0, sku=3, position=Coord(6, 5), count=2, max_count=2)],
    )
    apply_tick(world, {0: Action("pick", 6, 5)})
    assert world.pallets[0].count == 1
    assert world.robots[0].storage[3] == 1


def test_pick_from_empty_pallet_rejected():
    world = make_world(
        [Robot(id=0, position=Coord(5, 5))],
        [Pallet(id=0, sku=3, position=Coord(6, 5), count=0, max_count=2)],
    )
    with pytest.raises(InvalidActionError):
        apply_tick(world, {0: Action("pick", 6, 5)})


def test_two_robots_picking_one_each_from_count_two_both_succeed():
    world = make_world(
        [Robot(id=0, position=Coord(5, 5)), Robot(id=1, position=Coord(7, 5))],
        [Pallet(id=0, sku=3, position=Coord(6, 5), count=2, max_count=2)],
    )
    apply_tick(world, {0: Action("pick", 6, 5), 1: Action("pick", 6, 5)})
    assert world.pallets[0].count == 0
    assert world.robots[0].storage[3] == 1
    assert world.robots[1].storage[3] == 1


def test_pick_succeeds_even_when_the_docked_pallet_is_dragged_away_the_same_tick():
    # Regression test for a real bug found running the solver on the actual
    # Big Order: robot 1 picks from a pallet docked to robot 0, and robot 0
    # *also* moves this same tick, dragging that pallet elsewhere. Moves are
    # applied before picks, so re-deriving "which pallet is at this cell" via
    # world.entity_at() at apply time (rather than resolving it once, up
    # front, against the pristine start-of-tick state) would find nothing
    # there and crash. The pick's adjacency was valid at the start of the
    # tick, so per the engine's frozen-start-of-tick philosophy it must
    # still succeed, and the pallet ends up at its new (dragged-to) position.
    world = make_world(
        [Robot(id=0, position=Coord(6, 4)), Robot(id=1, position=Coord(5, 5))],
        [Pallet(id=0, sku=3, position=Coord(6, 5), count=2, max_count=2, docked_to=0, dock_offset=Coord(0, 1))],
    )
    world.robots[0].docked_pallets[Coord(0, 1)] = 0

    apply_tick(world, {0: Action("move", 6, 3), 1: Action("pick", 6, 5)})

    assert world.robots[1].storage[3] == 1
    assert world.pallets[0].count == 1
    assert world.pallets[0].position == Coord(6, 4)  # dragged along with robot 0
    assert Coord(6, 5) not in world.occupancy


def test_pick_oversubscription_raises_rule_violation():
    world = make_world(
        [Robot(id=0, position=Coord(5, 5)), Robot(id=1, position=Coord(7, 5))],
        [Pallet(id=0, sku=3, position=Coord(6, 5), count=1, max_count=2)],
    )
    with pytest.raises(RuleViolation):
        apply_tick(world, {0: Action("pick", 6, 5), 1: Action("pick", 6, 5)})


# ---- dock / undock ----

def test_dock_attaches_pallet_and_sets_offset():
    world = make_world(
        [Robot(id=0, position=Coord(5, 5))],
        [Pallet(id=0, sku=1, position=Coord(6, 5), count=1, max_count=5)],
    )
    apply_tick(world, {0: Action("dock", 6, 5)})
    assert world.robots[0].docked_pallets[Coord(1, 0)] == 0
    assert world.pallets[0].docked_to == 0
    assert world.pallets[0].dock_offset == Coord(1, 0)


def test_dock_up_to_four_pallets_fills_all_four_sides():
    robot = Robot(id=0, position=Coord(5, 5))
    pallets = [
        Pallet(id=0, sku=0, position=Coord(6, 5), count=1, max_count=5),
        Pallet(id=1, sku=0, position=Coord(4, 5), count=1, max_count=5),
        Pallet(id=2, sku=0, position=Coord(5, 4), count=1, max_count=5),
        Pallet(id=3, sku=0, position=Coord(5, 6), count=1, max_count=5),
    ]
    world = make_world([robot], pallets)
    apply_tick(world, {0: Action("dock", 6, 5)})
    apply_tick(world, {0: Action("dock", 4, 5)})
    apply_tick(world, {0: Action("dock", 5, 4)})
    apply_tick(world, {0: Action("dock", 5, 6)})
    assert len(world.robots[0].docked_pallets) == 4


def test_redocking_an_already_used_side_rejected():
    # Once the east side already holds a docked pallet, that side can't take
    # a second one even though a robot only has 4 sides total to fill.
    from warehouse.sim.rules import can_dock

    world = make_world(
        [Robot(id=0, position=Coord(5, 5))],
        [Pallet(id=0, sku=0, position=Coord(6, 5), count=1, max_count=5, docked_to=0, dock_offset=Coord(1, 0))],
    )
    world.robots[0].docked_pallets[Coord(1, 0)] = 0
    assert not can_dock(world, 0, Coord(6, 5))


def test_dock_someone_elses_docked_pallet_rejected():
    world = make_world(
        [Robot(id=0, position=Coord(5, 5)), Robot(id=1, position=Coord(6, 6))],
        [Pallet(id=0, sku=0, position=Coord(6, 5), count=1, max_count=5, docked_to=1, dock_offset=Coord(0, -1))],
    )
    world.robots[1].docked_pallets[Coord(0, -1)] = 0
    with pytest.raises(InvalidActionError):
        apply_tick(world, {0: Action("dock", 6, 5)})


def test_two_robots_docking_same_free_pallet_raises_rule_violation():
    world = make_world(
        [Robot(id=0, position=Coord(5, 5)), Robot(id=1, position=Coord(5, 7))],
        [Pallet(id=0, sku=0, position=Coord(5, 6), count=1, max_count=5)],
    )
    with pytest.raises(RuleViolation):
        apply_tick(world, {0: Action("dock", 5, 6), 1: Action("dock", 5, 6)})


def test_undock_leaves_pallet_in_place_and_clears_offset():
    world = make_world(
        [Robot(id=0, position=Coord(5, 5))],
        [Pallet(id=0, sku=0, position=Coord(6, 5), count=1, max_count=5, docked_to=0, dock_offset=Coord(1, 0))],
    )
    world.robots[0].docked_pallets[Coord(1, 0)] = 0
    apply_tick(world, {0: Action("undock", 6, 5)})
    assert world.pallets[0].docked_to is None
    assert world.pallets[0].dock_offset is None
    assert world.pallets[0].position == Coord(6, 5)
    assert Coord(1, 0) not in world.robots[0].docked_pallets


# ---- fulfill ----

def make_order_world(signature_orders):
    robot = Robot(id=0, position=Coord(3, 0))
    return make_world([robot], [], orders=signature_orders)


def test_fulfill_exact_match_succeeds_and_clears_storage():
    order = Order(id=0, requirements=Counter({1: 2, 2: 1}))
    world = make_order_world([order])
    world.robots[0].storage = Counter({1: 2, 2: 1})
    apply_tick(world, {0: Action("fulfill", 0, 0)})
    assert world.orders[0].fulfilled
    assert sum(world.robots[0].storage.values()) == 0


def test_fulfill_with_extra_item_rejected():
    order = Order(id=0, requirements=Counter({1: 2}))
    world = make_order_world([order])
    world.robots[0].storage = Counter({1: 2, 2: 1})
    with pytest.raises(InvalidActionError):
        apply_tick(world, {0: Action("fulfill", 0, 0)})


def test_fulfill_with_missing_item_rejected():
    order = Order(id=0, requirements=Counter({1: 2}))
    world = make_order_world([order])
    world.robots[0].storage = Counter({1: 1})
    with pytest.raises(InvalidActionError):
        apply_tick(world, {0: Action("fulfill", 0, 0)})


def test_fulfill_not_on_fulfillment_row_rejected():
    order = Order(id=0, requirements=Counter({1: 2}))
    robot = Robot(id=0, position=Coord(3, 1))
    world = make_world([robot], [], orders=[order])
    world.robots[0].storage = Counter({1: 2})
    with pytest.raises(InvalidActionError):
        apply_tick(world, {0: Action("fulfill", 0, 0)})


def test_duplicate_signature_orders_both_fulfillable_lowest_id_first():
    orders = [
        Order(id=0, requirements=Counter({1: 1})),
        Order(id=1, requirements=Counter({1: 1})),
    ]
    robots = [Robot(id=0, position=Coord(3, 0)), Robot(id=1, position=Coord(4, 0))]
    world = make_world(robots, [], orders=orders)
    world.robots[0].storage = Counter({1: 1})
    world.robots[1].storage = Counter({1: 1})
    apply_tick(world, {0: Action("fulfill", 0, 0), 1: Action("fulfill", 0, 0)})
    assert world.orders[0].fulfilled
    assert world.orders[1].fulfilled


def test_more_simultaneous_fulfills_than_matching_orders_raises_rule_violation():
    orders = [Order(id=0, requirements=Counter({1: 1}))]
    robots = [Robot(id=0, position=Coord(3, 0)), Robot(id=1, position=Coord(4, 0))]
    world = make_world(robots, [], orders=orders)
    world.robots[0].storage = Counter({1: 1})
    world.robots[1].storage = Counter({1: 1})
    with pytest.raises(RuleViolation):
        apply_tick(world, {0: Action("fulfill", 0, 0), 1: Action("fulfill", 0, 0)})

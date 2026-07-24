from collections import Counter

from warehouse.model.entities import Order, Pallet, Robot
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState


def make_world():
    grid = Grid(width=60, height=40)
    robots = {
        0: Robot(id=0, position=Coord(1, 1)),
        1: Robot(id=1, position=Coord(2, 2)),
    }
    pallets = {
        0: Pallet(id=0, sku=0, position=Coord(5, 5), count=10, max_count=10),
    }
    orders = [
        Order(id=0, requirements=Counter({0: 2, 1: 1})),
        Order(id=1, requirements=Counter({0: 2, 1: 1})),  # duplicate signature of order 0
        Order(id=2, requirements=Counter({2: 3})),
    ]
    return WorldState(grid=grid, robots=robots, pallets=pallets, orders=orders)


def test_occupancy_built_at_init():
    world = make_world()
    assert world.occupancy[Coord(1, 1)] == ("robot", 0)
    assert world.occupancy[Coord(2, 2)] == ("robot", 1)
    assert world.occupancy[Coord(5, 5)] == ("pallet", 0)


def test_rebuild_occupancy_matches_incremental_state():
    world = make_world()
    assert world.rebuild_occupancy() == world.occupancy


def test_robot_footprint_includes_docked_pallets():
    world = make_world()
    world.robots[0].docked_pallets[Coord(0, -1)] = 0
    footprint = world.robot_footprint(0)
    assert footprint == {Coord(1, 1), Coord(1, 0)}


def test_duplicate_signature_orders_both_indexed_lowest_id_first():
    world = make_world()
    sig = world.orders[0].signature()
    assert world.unfulfilled_order_ids(sig) == [0, 1]


def test_fulfilling_one_order_leaves_the_duplicate_unfulfilled():
    world = make_world()
    sig = world.orders[0].signature()
    world.orders[0].fulfilled = True
    assert world.unfulfilled_order_ids(sig) == [1]


def test_all_orders_fulfilled():
    world = make_world()
    assert not world.all_orders_fulfilled()
    for order in world.orders:
        order.fulfilled = True
    assert world.all_orders_fulfilled()


def test_signature_lookup_for_unknown_multiset_is_empty():
    world = make_world()
    assert world.unfulfilled_order_ids(((99, 1),)) == []

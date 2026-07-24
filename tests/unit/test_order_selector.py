from collections import Counter

from warehouse.model.entities import Order, Pallet, Robot
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState
from warehouse.tasks.order_selector import FifoOrderSelector, NearestOrderSelector
from warehouse.tasks.pallet_selector import NearestAvailablePallet


def make_world(orders, pallets=None):
    grid = Grid(width=60, height=40)
    return WorldState(
        grid=grid, robots={0: Robot(id=0, position=Coord(0, 0))}, pallets=pallets or {}, orders=orders
    )


def test_selects_lowest_unfulfilled_order_id():
    orders = [Order(id=0, requirements=Counter({0: 1})), Order(id=1, requirements=Counter({1: 1}))]
    world = make_world(orders)
    assert FifoOrderSelector().select(world, claimed_order_ids=set(), from_coord=Coord(0, 0)) == 0


def test_skips_fulfilled_orders():
    orders = [Order(id=0, requirements=Counter({0: 1}), fulfilled=True), Order(id=1, requirements=Counter({1: 1}))]
    world = make_world(orders)
    assert FifoOrderSelector().select(world, claimed_order_ids=set(), from_coord=Coord(0, 0)) == 1


def test_skips_claimed_orders():
    orders = [Order(id=0, requirements=Counter({0: 1})), Order(id=1, requirements=Counter({1: 1}))]
    world = make_world(orders)
    assert FifoOrderSelector().select(world, claimed_order_ids={0}, from_coord=Coord(0, 0)) == 1


def test_returns_none_when_no_orders_remain():
    orders = [Order(id=0, requirements=Counter({0: 1}), fulfilled=True)]
    world = make_world(orders)
    assert FifoOrderSelector().select(world, claimed_order_ids=set(), from_coord=Coord(0, 0)) is None


def test_nearest_order_selector_picks_the_closer_orders_pallet():
    pallets = {
        0: Pallet(id=0, sku=0, position=Coord(1, 0), count=5, max_count=5),
        1: Pallet(id=1, sku=1, position=Coord(30, 30), count=5, max_count=5),
    }
    orders = [
        Order(id=0, requirements=Counter({1: 1})),  # far pallet
        Order(id=1, requirements=Counter({0: 1})),  # near pallet
    ]
    world = make_world(orders, pallets)
    selector = NearestOrderSelector(NearestAvailablePallet())
    assert selector.select(world, claimed_order_ids=set(), from_coord=Coord(0, 0)) == 1


def test_nearest_order_selector_uses_the_orders_closest_required_sku():
    pallets = {
        0: Pallet(id=0, sku=0, position=Coord(20, 20), count=5, max_count=5),
        1: Pallet(id=1, sku=1, position=Coord(1, 0), count=5, max_count=5),
    }
    # order 0 needs both skus; its *closest* one (sku 1) is right next to the robot
    orders = [Order(id=0, requirements=Counter({0: 1, 1: 1}))]
    world = make_world(orders, pallets)
    selector = NearestOrderSelector(NearestAvailablePallet())
    assert selector.select(world, claimed_order_ids=set(), from_coord=Coord(0, 0)) == 0


def test_nearest_order_selector_falls_back_to_any_pallet_when_sku_out_of_stock():
    pallets = {0: Pallet(id=0, sku=0, position=Coord(3, 0), count=0, max_count=5)}
    orders = [Order(id=0, requirements=Counter({0: 1}))]
    world = make_world(orders, pallets)
    selector = NearestOrderSelector(NearestAvailablePallet())
    assert selector.select(world, claimed_order_ids=set(), from_coord=Coord(0, 0)) == 0


def test_nearest_order_selector_skips_fulfilled_and_claimed_orders():
    pallets = {0: Pallet(id=0, sku=0, position=Coord(1, 0), count=5, max_count=5)}
    orders = [
        Order(id=0, requirements=Counter({0: 1}), fulfilled=True),
        Order(id=1, requirements=Counter({0: 1})),
    ]
    world = make_world(orders, pallets)
    selector = NearestOrderSelector(NearestAvailablePallet())
    assert selector.select(world, claimed_order_ids=set(), from_coord=Coord(0, 0)) == 1
    assert selector.select(world, claimed_order_ids={1}, from_coord=Coord(0, 0)) is None

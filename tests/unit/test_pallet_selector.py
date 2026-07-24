from warehouse.model.entities import Pallet, Robot
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState
from warehouse.tasks.pallet_selector import NearestAvailablePallet


def make_world(pallets):
    grid = Grid(width=60, height=40)
    return WorldState(grid=grid, robots={0: Robot(id=0, position=Coord(0, 0))}, pallets=pallets, orders=[])


def test_selects_nearest_pallet_with_stock():
    pallets = {
        0: Pallet(id=0, sku=1, position=Coord(10, 10), count=5, max_count=5),
        1: Pallet(id=1, sku=1, position=Coord(2, 2), count=5, max_count=5),
    }
    world = make_world(pallets)
    selector = NearestAvailablePallet()
    assert selector.select(1, Coord(0, 0), world) == 1


def test_ignores_empty_pallets():
    pallets = {
        0: Pallet(id=0, sku=1, position=Coord(2, 2), count=0, max_count=5),
        1: Pallet(id=1, sku=1, position=Coord(10, 10), count=5, max_count=5),
    }
    world = make_world(pallets)
    selector = NearestAvailablePallet()
    assert selector.select(1, Coord(0, 0), world) == 1


def test_returns_none_when_all_pallets_of_sku_are_empty():
    pallets = {0: Pallet(id=0, sku=1, position=Coord(2, 2), count=0, max_count=5)}
    world = make_world(pallets)
    selector = NearestAvailablePallet()
    assert selector.select(1, Coord(0, 0), world) is None


def test_ignores_pallets_of_a_different_sku():
    pallets = {0: Pallet(id=0, sku=2, position=Coord(1, 1), count=5, max_count=5)}
    world = make_world(pallets)
    selector = NearestAvailablePallet()
    assert selector.select(1, Coord(0, 0), world) is None


def test_ties_broken_by_lowest_pallet_id():
    pallets = {
        5: Pallet(id=5, sku=1, position=Coord(3, 0), count=5, max_count=5),
        2: Pallet(id=2, sku=1, position=Coord(3, 0), count=5, max_count=5),
    }
    world = make_world(pallets)
    selector = NearestAvailablePallet()
    assert selector.select(1, Coord(0, 0), world) == 2


def test_skips_a_pallet_fully_boxed_in_by_other_pallets():
    # Regression test for a real bug found running the solver on the actual
    # Big Order: the nearest pallet of a SKU can be completely surrounded on
    # all four sides by *other* pallets (which never move on their own), so
    # it can never be approached by anyone -- not just for replenishment
    # (which additionally excludes the north side), but for ordinary picking
    # too. Selecting it anyway because it's nearest deadlocks every future
    # need for that SKU forever, even when a farther, reachable pallet of the
    # same SKU exists.
    boxed_in = Pallet(id=0, sku=1, position=Coord(5, 5), count=5, max_count=5)
    blockers = {
        1: Pallet(id=1, sku=9, position=Coord(4, 5), count=1, max_count=1),
        2: Pallet(id=2, sku=9, position=Coord(6, 5), count=1, max_count=1),
        3: Pallet(id=3, sku=9, position=Coord(5, 4), count=1, max_count=1),
        4: Pallet(id=4, sku=9, position=Coord(5, 6), count=1, max_count=1),
    }
    reachable = Pallet(id=5, sku=1, position=Coord(20, 20), count=5, max_count=5)
    pallets = {0: boxed_in, **blockers, 5: reachable}
    world = make_world(pallets)
    selector = NearestAvailablePallet()
    assert selector.select(1, Coord(0, 0), world) == 5


def test_returns_none_rather_than_a_boxed_in_pallet_when_its_the_only_stock():
    # Regression test for a real bug found running the solver on the actual
    # Big Order: a pallet can have real stock but be permanently unreachable
    # (boxed in on all sides), while its siblings of the same SKU are empty
    # but perfectly reachable. Falling back to "select it anyway, nothing
    # else has stock" deadlocks every future need for that SKU forever --
    # the correct signal is "no *usable* stock right now," so the caller
    # (RobotController._step_collect) falls through to the replenish path
    # and refills a reachable sibling instead.
    boxed_in_with_stock = Pallet(id=0, sku=1, position=Coord(5, 5), count=74, max_count=212)
    blockers = {
        1: Pallet(id=1, sku=9, position=Coord(4, 5), count=1, max_count=1),
        2: Pallet(id=2, sku=9, position=Coord(6, 5), count=1, max_count=1),
        3: Pallet(id=3, sku=9, position=Coord(5, 4), count=1, max_count=1),
        4: Pallet(id=4, sku=9, position=Coord(5, 6), count=1, max_count=1),
    }
    reachable_but_empty = Pallet(id=5, sku=1, position=Coord(20, 20), count=0, max_count=212)
    pallets = {0: boxed_in_with_stock, **blockers, 5: reachable_but_empty}
    world = make_world(pallets)
    selector = NearestAvailablePallet()
    assert selector.select(1, Coord(0, 0), world) is None

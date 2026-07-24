from collections import Counter

from warehouse.model.entities import Order, Pallet, Robot
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState
from warehouse.tasks.relocation import plan_relocations, sku_demand_rank


def make_world(pallets, orders, width=12, height=6):
    grid = Grid(width=width, height=height)
    robots = {0: Robot(id=0, position=Coord(width - 1, height - 1))}
    return WorldState(grid=grid, robots=robots, pallets={p.id: p for p in pallets}, orders=orders)


def test_sku_demand_rank_orders_by_distinct_order_count_desc_ties_by_sku_id():
    pallets = [
        Pallet(id=0, sku=5, position=Coord(1, 3), count=1, max_count=1),
        Pallet(id=1, sku=7, position=Coord(2, 3), count=1, max_count=1),
        Pallet(id=2, sku=9, position=Coord(3, 3), count=1, max_count=1),
    ]
    orders = [
        Order(id=0, requirements=Counter({5: 1, 7: 1})),
        Order(id=1, requirements=Counter({5: 1})),
        Order(id=2, requirements=Counter({9: 1, 7: 1})),
    ]  # sku5: 2 orders, sku7: 2 orders, sku9: 1 order -- tie between 5 and 7 broken by id
    world = make_world(pallets, orders)

    assert sku_demand_rank(world) == [5, 7, 9]


def test_sku_demand_rank_counts_units_within_one_order_only_once():
    pallets = [Pallet(id=0, sku=1, position=Coord(1, 3), count=1, max_count=1)]
    orders = [Order(id=0, requirements=Counter({1: 50}))]  # one order, many units
    world = make_world(pallets, orders)

    assert sku_demand_rank(world) == [1]


def test_plan_relocations_breaks_an_equidistant_tie_in_favor_of_higher_demand():
    # Assignment is greedy nearest-slot-first, processed in demand-rank order
    # (see plan_relocations' docstring for why: minimizing each pallet's own
    # drag distance matters more than strictly ranking final row position,
    # since a long drag with a fixed dock offset can leave a robot's rigid
    # footprint with no valid route at all -- found empirically). Demand rank
    # still matters as *priority*: when two pallets are equidistant from the
    # same slot, whichever is processed first (higher demand) wins it.
    # width=3 -> one slot column (x=1), two rows -> slots (1,1) and (1,3),
    # both pallets sitting at (1,2) are equidistant (distance 1) from either.
    pallets = [
        Pallet(id=0, sku=1, position=Coord(1, 2), count=1, max_count=1),  # low demand
        Pallet(id=1, sku=2, position=Coord(1, 2), count=1, max_count=1),  # high demand
    ]
    orders = [Order(id=i, requirements=Counter({2: 1})) for i in range(5)] + [
        Order(id=5, requirements=Counter({1: 1}))
    ]
    world = make_world(pallets, orders, width=3)

    targets = plan_relocations(world)

    assert targets[1] == Coord(1, 1)  # sku 2 (5 orders) wins the tie, gets the slot closer to y=0
    assert targets[0] == Coord(1, 3)


def test_plan_relocations_assigns_each_pallet_its_nearest_available_slot():
    # width=12 -> slot columns {1,3,5,7,9} (row 1 only, 5 pallets). Pallet 2
    # sits right next to slot (5,1); a naive rank-order fill (rather than
    # greedy nearest-slot) would instead hand it whatever the fill sequence
    # reaches next, regardless of proximity.
    pallets = [
        Pallet(id=0, sku=0, position=Coord(1, 3), count=1, max_count=1),
        Pallet(id=1, sku=1, position=Coord(3, 3), count=1, max_count=1),
        Pallet(id=2, sku=2, position=Coord(5, 2), count=1, max_count=1),  # right next to slot (5, 1)
        Pallet(id=3, sku=3, position=Coord(7, 3), count=1, max_count=1),
        Pallet(id=4, sku=4, position=Coord(9, 3), count=1, max_count=1),
    ]
    orders = [Order(id=0, requirements=Counter({i: 1 for i in range(5)}))]
    world = make_world(pallets, orders, width=12)

    targets = plan_relocations(world)

    assert targets[2] == Coord(5, 1)


def test_plan_relocations_never_targets_an_even_cell_or_edge_column():
    # Pallets only ever rest at (odd x, odd y) -- a full 2D checkerboard, not
    # just corridor columns -- so every neighbor of a target lands on a
    # corridor row or column regardless of which side a drag approaches from.
    # See plan_relocations' docstring for the deadlock this prevents.
    width = 12
    pallets = [Pallet(id=i, sku=i, position=Coord(1 + i % 5, 3), count=1, max_count=1) for i in range(9)]
    orders = [Order(id=0, requirements=Counter({i: 1 for i in range(9)}))]
    world = make_world(pallets, orders, width=width)

    targets = plan_relocations(world)

    assert all(c.x % 2 == 1 for c in targets.values())
    assert all(c.y % 2 == 1 for c in targets.values())
    assert all(c.x != width - 1 for c in targets.values())


def test_plan_relocations_target_count_matches_pallet_count_and_all_distinct():
    pallets = [Pallet(id=i, sku=i, position=Coord(1 + i % 5, 3), count=1, max_count=1) for i in range(9)]
    orders = [Order(id=0, requirements=Counter({i: 1 for i in range(9)}))]
    world = make_world(pallets, orders)

    targets = plan_relocations(world)

    assert len(targets) == 9
    assert len(set(targets.values())) == 9


def test_plan_relocations_ceil_divides_rows_when_pallet_count_isnt_a_clean_multiple():
    # width=12 -> slot columns are odd x excluding the edge column 11: {1,3,5,7,9}
    # -- 5 slot columns per pallet row.
    width = 12
    pallets = [Pallet(id=i, sku=i, position=Coord(1 + i % 5, 3), count=1, max_count=1) for i in range(9)]
    orders = [Order(id=0, requirements=Counter({i: 1 for i in range(9)}))]
    world = make_world(pallets, orders, width=width)

    targets = plan_relocations(world)

    # ceil(9 / 5) == 2 pallet rows needed; rows are odd-y (1, then 3), not consecutive.
    assert max(c.y for c in targets.values()) == 3

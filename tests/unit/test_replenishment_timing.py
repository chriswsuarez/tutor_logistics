from warehouse.model.action import Action
from warehouse.model.entities import Pallet, Robot
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState
from warehouse.sim.engine import apply_tick


def make_world(robots, pallets):
    grid = Grid(width=60, height=40)
    return WorldState(
        grid=grid,
        robots={r.id: r for r in robots},
        pallets={p.id: p for p in pallets},
        orders=[],
    )


def test_docked_pallet_at_replenishment_row_refills_even_after_same_tick_pick():
    robot = Robot(id=0, position=Coord(5, 39))
    pallet = Pallet(id=0, sku=1, position=Coord(6, 39), count=3, max_count=10, docked_to=0, dock_offset=Coord(1, 0))
    robot.docked_pallets[Coord(1, 0)] = 0
    world = make_world([robot], [pallet])

    apply_tick(world, {0: Action("pick", 6, 39)})

    assert world.pallets[0].count == 10  # picked to 2, then refilled to max in the same tick
    assert world.robots[0].storage[1] == 1


def test_undocked_pallet_resting_at_replenishment_row_never_refills():
    robot = Robot(id=0, position=Coord(5, 39))
    pallet = Pallet(id=0, sku=1, position=Coord(6, 39), count=3, max_count=10)  # not docked to anyone
    world = make_world([robot], [pallet])

    apply_tick(world, {0: Action("pick", 6, 39)})

    assert world.pallets[0].count == 2  # decremented, no refill: not docked


def test_arriving_at_replenishment_row_this_tick_refills_this_same_tick():
    robot = Robot(id=0, position=Coord(5, 38))
    pallet = Pallet(id=0, sku=1, position=Coord(6, 38), count=3, max_count=10, docked_to=0, dock_offset=Coord(1, 0))
    robot.docked_pallets[Coord(1, 0)] = 0
    world = make_world([robot], [pallet])

    apply_tick(world, {0: Action("move", 5, 39)})

    assert world.robots[0].position == Coord(5, 39)
    assert world.pallets[0].position == Coord(6, 39)
    assert world.pallets[0].count == 10


def test_undocking_while_parked_skips_that_ticks_refill():
    robot = Robot(id=0, position=Coord(5, 39))
    pallet = Pallet(id=0, sku=1, position=Coord(6, 39), count=3, max_count=10, docked_to=0, dock_offset=Coord(1, 0))
    robot.docked_pallets[Coord(1, 0)] = 0
    world = make_world([robot], [pallet])

    apply_tick(world, {0: Action("undock", 6, 39)})

    assert world.pallets[0].count == 3  # unchanged: undocked before the refill check ran
    assert world.pallets[0].docked_to is None


def test_robot_with_no_docked_pallets_at_replenishment_row_does_nothing():
    robot = Robot(id=0, position=Coord(5, 38))
    world = make_world([robot], [])
    apply_tick(world, {0: Action("move", 5, 39)})
    assert world.robots[0].position == Coord(5, 39)  # just confirms the tick applied cleanly

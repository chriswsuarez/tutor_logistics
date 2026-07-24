from warehouse.model.entities import Pallet, Robot
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState
from warehouse.planning.reservation import ReservationTable


def make_world():
    grid = Grid(width=10, height=10)
    robots = {
        0: Robot(id=0, position=Coord(2, 2)),
        1: Robot(id=1, position=Coord(5, 5)),
    }
    pallets = {
        0: Pallet(id=0, sku=0, position=Coord(3, 2), count=5, max_count=5, docked_to=0, dock_offset=Coord(1, 0)),
        1: Pallet(id=1, sku=1, position=Coord(7, 7), count=5, max_count=5),
    }
    robots[0].docked_pallets[Coord(1, 0)] = 0
    return WorldState(grid=grid, robots=robots, pallets=pallets, orders=[])


def test_sync_static_holds_includes_idle_robots_and_pallets():
    world = make_world()
    table = ReservationTable()
    table.sync_static_holds(world, planned_robot_ids=set())
    assert Coord(2, 2) in table.static_holds
    assert Coord(5, 5) in table.static_holds
    assert Coord(3, 2) in table.static_holds  # docked pallet
    assert Coord(7, 7) in table.static_holds  # free-standing pallet


def test_sync_static_holds_excludes_planned_robot_and_its_docked_pallets():
    world = make_world()
    table = ReservationTable()
    table.sync_static_holds(world, planned_robot_ids={0})
    assert Coord(2, 2) not in table.static_holds  # robot 0 itself
    assert Coord(3, 2) not in table.static_holds  # pallet docked to robot 0
    assert Coord(5, 5) in table.static_holds  # robot 1 still idle/static
    assert Coord(7, 7) in table.static_holds  # unrelated free pallet


def test_is_free_respects_static_holds():
    world = make_world()
    table = ReservationTable()
    table.sync_static_holds(world, planned_robot_ids=set())
    assert not table.is_free(Coord(5, 5), t=10)
    assert table.is_free(Coord(9, 9), t=10)


def test_is_free_ignore_cells_bypasses_static_hold():
    world = make_world()
    table = ReservationTable()
    table.sync_static_holds(world, planned_robot_ids=set())
    assert not table.is_free(Coord(5, 5), t=10)
    assert table.is_free(Coord(5, 5), t=10, ignore_cells=frozenset({Coord(5, 5)}))


def test_reserve_path_blocks_cell_only_at_reserved_time():
    table = ReservationTable()
    path = [Coord(0, 0), Coord(1, 0), Coord(2, 0)]
    table.reserve_path(robot_id=7, path=path, start_t=5, footprint_offsets=[])
    assert not table.is_free(Coord(1, 0), t=6)
    assert table.is_free(Coord(1, 0), t=7)
    assert table.is_free(Coord(1, 0), t=5)


def test_reserve_path_also_reserves_docked_pallet_footprint():
    table = ReservationTable()
    path = [Coord(0, 0), Coord(1, 0)]
    table.reserve_path(robot_id=1, path=path, start_t=0, footprint_offsets=[Coord(0, -1)])
    # at t=1 the robot is at (1,0), its docked pallet trails at (1,-1)
    assert not table.is_free(Coord(1, 0), t=1)
    assert not table.is_free(Coord(1, -1), t=1)


def test_edge_conflict_detects_swap():
    table = ReservationTable()
    # robot A moves (0,0) -> (1,0) arriving at t=1
    table.reserve_path(robot_id=1, path=[Coord(0, 0), Coord(1, 0)], start_t=0, footprint_offsets=[])
    # robot B attempting the exact reverse transition at the same t must be rejected
    assert not table.is_edge_free(Coord(1, 0), Coord(0, 0), t=1)
    # a non-conflicting transition remains fine
    assert table.is_edge_free(Coord(5, 5), Coord(5, 6), t=1)


def test_release_robot_removes_only_its_own_reservations():
    table = ReservationTable()
    table.reserve_path(robot_id=1, path=[Coord(0, 0), Coord(1, 0)], start_t=0, footprint_offsets=[])
    table.reserve_path(robot_id=2, path=[Coord(5, 5), Coord(5, 6)], start_t=0, footprint_offsets=[])
    table.release_robot(1)
    assert table.is_free(Coord(1, 0), t=1)
    assert not table.is_free(Coord(5, 6), t=1)


def test_prune_before_drops_elapsed_reservations():
    table = ReservationTable()
    table.reserve_path(robot_id=1, path=[Coord(0, 0), Coord(1, 0), Coord(2, 0)], start_t=0, footprint_offsets=[])
    table.prune_before(2)
    assert table.is_free(Coord(1, 0), t=1)  # elapsed, pruned
    assert not table.is_free(Coord(2, 0), t=2)  # still current/future

from warehouse.model.grid import Coord, Grid
from warehouse.planning.cooperative_astar import GoalSpec, adjacent_to, any_row, plan
from warehouse.planning.reservation import ReservationTable


def exact_coord_goal(target: Coord) -> GoalSpec:
    return GoalSpec(
        is_goal=lambda c: c == target,
        heuristic=lambda c: abs(c.x - target.x) + abs(c.y - target.y),
    )


def assert_valid_step_sequence(path):
    for a, b in zip(path, path[1:]):
        assert a == b or b in Grid(60, 40).neighbors4(a), f"illegal step {a} -> {b}"


def test_simple_path_on_empty_grid():
    grid = Grid(width=10, height=10)
    table = ReservationTable()
    path = plan(0, Coord(0, 0), 0, any_row(5, grid), [], table, grid)
    assert path is not None
    assert path[0] == Coord(0, 0)
    assert path[-1].y == 5
    assert len(path) == 6  # 5 moves, no reason to detour or wait
    assert_valid_step_sequence(path)


def test_adjacent_to_exclude_offsets_removes_that_candidate():
    grid = Grid(width=10, height=10)
    pallet_pos = Coord(5, 5)
    north_of_pallet = Coord(5, 4)

    plain = adjacent_to(pallet_pos, grid)
    assert north_of_pallet in plain.candidates

    excluded = adjacent_to(pallet_pos, grid, exclude_offsets=frozenset({Coord(0, -1)}))
    assert north_of_pallet not in excluded.candidates
    assert not excluded.is_goal(north_of_pallet)
    # the other three sides remain valid candidates
    assert len(excluded.candidates) == len(plain.candidates) - 1


def test_adjacent_to_goal_stops_next_to_target_not_on_it():
    grid = Grid(width=10, height=10)
    table = ReservationTable()
    pallet_pos = Coord(5, 5)
    path = plan(0, Coord(0, 5), 0, adjacent_to(pallet_pos, grid), [], table, grid)
    assert path is not None
    assert path[-1] in set(grid.neighbors4(pallet_pos))
    assert path[-1] != pallet_pos


def test_detours_around_a_wall_through_the_only_gap():
    grid = Grid(width=5, height=5)
    table = ReservationTable()
    # Wall along y=2 blocking every column except x=2 (the only gap).
    for x in (0, 1, 3, 4):
        table.static_holds[Coord(x, 2)] = ("pallet", -1)

    path = plan(0, Coord(0, 0), 0, exact_coord_goal(Coord(0, 4)), [], table, grid)
    assert path is not None
    assert path[0] == Coord(0, 0)
    assert path[-1] == Coord(0, 4)
    assert_valid_step_sequence(path)
    for coord in path:
        if coord.y == 2:
            assert coord.x == 2  # only the gap may be used to cross the wall


def test_waits_or_detours_around_another_robots_reserved_path():
    grid = Grid(width=5, height=5)
    table = ReservationTable()
    # Robot 99 already committed to marching straight down column x=2.
    reserved_path = [Coord(2, 0), Coord(2, 1), Coord(2, 2), Coord(2, 3), Coord(2, 4)]
    table.reserve_path(robot_id=99, path=reserved_path, start_t=0, footprint_offsets=[])

    path = plan(0, Coord(0, 2), 0, exact_coord_goal(Coord(4, 2)), [], table, grid)
    assert path is not None
    assert_valid_step_sequence(path)
    for i, coord in enumerate(path):
        if coord == Coord(2, 2):
            assert i != 2, "must not cross (2,2) at t=2, which robot 99 has reserved"


def test_multi_cell_footprint_navigates_a_two_row_corridor():
    grid = Grid(width=5, height=3)
    table = ReservationTable()
    # Wall only below the 2-row corridor (y=2), y=0 and y=1 both open across.
    for x in (1, 2, 3):
        table.static_holds[Coord(x, 2)] = ("pallet", -1)

    # Robot travels along y=0 with a pallet docked to its south (offset (0,1)),
    # so its footprint spans y=0 and y=1 the whole way — both rows are clear.
    path = plan(0, Coord(0, 0), 0, exact_coord_goal(Coord(4, 0)), [Coord(0, 1)], table, grid)
    assert path is not None
    assert path[-1] == Coord(4, 0)


def test_multi_cell_footprint_blocked_by_a_one_row_corridor():
    grid = Grid(width=5, height=3)
    table = ReservationTable()
    # Now both y=1 and y=2 are walled off across the corridor — only y=0 is
    # open, too narrow for a robot trailing a pallet to its south.
    for x in (1, 2, 3):
        table.static_holds[Coord(x, 1)] = ("pallet", -1)
        table.static_holds[Coord(x, 2)] = ("pallet", -1)

    path = plan(0, Coord(0, 0), 0, exact_coord_goal(Coord(4, 0)), [Coord(0, 1)], table, grid, max_expansions=2000)
    assert path is None


def test_plan_returns_none_cleanly_for_unreachable_goal():
    grid = Grid(width=5, height=5)
    table = ReservationTable()
    target = Coord(4, 4)
    # Both in-bounds neighbors of the corner target are sealed off.
    table.static_holds[Coord(3, 4)] = ("pallet", -1)
    table.static_holds[Coord(4, 3)] = ("pallet", -1)

    path = plan(0, Coord(0, 0), 0, exact_coord_goal(target), [], table, grid, max_expansions=500)
    assert path is None


def test_goal_rejects_a_cell_with_a_conflicting_far_future_reservation():
    # Regression test for a real bug found running the solver against the
    # actual Big Order: robot 99 reserved a cell for a far-future tick (as if
    # planned "well ahead of time", long before any conflict was knowable). A
    # different robot later wants to *settle* indefinitely (e.g. to pick many
    # times) at one of several candidate cells that happens to include that
    # same cell -- it must recognize the future conflict and pick a different
    # candidate, not just check whether the cell is free *right now*.
    grid = Grid(width=3, height=2)
    table = ReservationTable()
    table.reserve_path(robot_id=99, path=[Coord(1, 0)], start_t=100, footprint_offsets=[])

    settle_targets = {Coord(1, 0), Coord(1, 1)}
    goal = GoalSpec(
        is_goal=lambda c: c in settle_targets,
        heuristic=lambda c: min(abs(c.x - t.x) + abs(c.y - t.y) for t in settle_targets),
    )

    path = plan(0, Coord(0, 0), 0, goal, [], table, grid)
    assert path is not None
    assert path[-1] == Coord(1, 1), "must avoid (1,0), which is reserved far in the future"


def test_goal_rejected_with_no_alternative_and_tight_budget_returns_none():
    grid = Grid(width=2, height=1)
    table = ReservationTable()
    table.reserve_path(robot_id=99, path=[Coord(1, 0)], start_t=100, footprint_offsets=[])

    path = plan(0, Coord(0, 0), 0, exact_coord_goal(Coord(1, 0)), [], table, grid, max_expansions=10)
    assert path is None


def test_edge_conflict_forces_a_detour_instead_of_swapping():
    # A strict 2-cell (or 1D) corridor makes a full swap mathematically
    # impossible no matter how long either robot waits (no node has degree >=
    # 3 to step aside into) -- that's correct MAPF behavior, not a bug. This
    # test instead uses a 2x2 grid (a 4-cycle), where the swap is illegal but
    # a same-cost-class detour around the loop is available.
    grid = Grid(width=2, height=2)
    table = ReservationTable()
    # Robot 99 moves (1,0) -> (0,0), arriving at t=1.
    table.reserve_path(robot_id=99, path=[Coord(1, 0), Coord(0, 0)], start_t=0, footprint_offsets=[])

    # A naive robot 0 going (0,0) -> (1,0) arriving at t=1 would be an exact
    # swap, and simply waiting at (0,0) is also illegal (robot 99 arrives
    # there at t=1) -- the only legal option is to flee around the loop.
    path = plan(0, Coord(0, 0), 0, exact_coord_goal(Coord(1, 0)), [], table, grid)
    assert path is not None
    assert path != [Coord(0, 0), Coord(1, 0)], "must not perform the swap"
    assert path[-1] == Coord(1, 0)
    assert_valid_step_sequence(path)
    assert len(path) == 4  # goes the long way around the 2x2 loop

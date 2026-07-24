from collections import deque

import pytest

from warehouse.model.entities import Pallet, Robot
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState
from warehouse.tasks.pallet_selector import NearestAvailablePallet
from warehouse.tasks.relocation import _STALL_TICKS_BEFORE_ERROR, RelocationCoordinator
from warehouse.tasks.robot_controller import RobotController
from warehouse.tasks.task_types import RelocateSubGoal


def make_controllers(robot_ids):
    return {rid: RobotController(rid, NearestAvailablePallet()) for rid in robot_ids}


def test_assigns_nearest_backlog_pallet_regardless_of_target_row():
    # Row order doesn't constrain execution order at all (see class
    # docstring) -- assignment is pure nearest-pallet-first, same pattern as
    # NearestOrderSelector. Pallet 1 is closer despite its target being in a
    # deeper row than pallet 0's.
    grid = Grid(width=20, height=10)
    robot = Robot(id=0, position=Coord(5, 5))
    far_but_top_row = Pallet(id=0, sku=0, position=Coord(15, 5), count=1, max_count=1)
    near_but_deep_row = Pallet(id=1, sku=1, position=Coord(6, 5), count=1, max_count=1)
    world = WorldState(grid=grid, robots={0: robot}, pallets={0: far_but_top_row, 1: near_but_deep_row}, orders=[])

    controllers = make_controllers([0])
    targets = {0: Coord(1, 1), 1: Coord(1, 5)}
    coordinator = RelocationCoordinator(controllers, targets, world)

    coordinator.assign_idle_robots(world)

    assert controllers[0].subgoals[0] == RelocateSubGoal(pallet_id=1, target=Coord(1, 5))


def test_a_pallet_whose_own_target_is_free_stays_assignable_even_if_it_blocks_another():
    # No row barrier: unlike a strict row-by-row scheme, a pallet remains
    # immediately eligible for assignment purely because its OWN target is
    # free, regardless of whether some other pallet is (transiently) blocked
    # by its still-unmoved original position. This is what prevents the real
    # stall found empirically: a barrier that gates on "is this pallet's row
    # open yet" can leave a blocker stuck behind its own row's turn for far
    # longer than any real cyclic dependency would take.
    grid = Grid(width=20, height=10)
    robot = Robot(id=0, position=Coord(5, 5))
    blocker = Pallet(id=0, sku=0, position=Coord(2, 1), count=1, max_count=1)  # sits on pallet 1's target
    blocked = Pallet(id=1, sku=1, position=Coord(10, 5), count=1, max_count=1)
    world = WorldState(grid=grid, robots={0: robot}, pallets={0: blocker, 1: blocked}, orders=[])

    controllers = make_controllers([0])
    targets = {0: Coord(1, 1), 1: Coord(2, 1)}
    coordinator = RelocationCoordinator(controllers, targets, world)

    coordinator.assign_idle_robots(world)

    assert controllers[0].subgoals[0].pallet_id == 0  # blocker's own target (1,1) is free -- assignable now


def test_prioritizes_a_pallet_currently_blocking_a_docked_robots_landing_cell():
    # Regression test for the real starvation found empirically: pallet 2
    # normally wins on row-priority alone (target row 1 vs pallet 1's row 9),
    # but pallet 1 is sitting on robot 0's committed landing cell -- if it's
    # never bumped to the front, robot 0 can never finish while other idle
    # robots keep preferring closer/higher-priority backlog items instead.
    grid = Grid(width=20, height=10)
    robot0 = Robot(id=0, position=Coord(10, 10))
    pallet0 = Pallet(id=0, sku=0, position=Coord(11, 10), count=1, max_count=1, docked_to=0, dock_offset=Coord(1, 0))
    robot0.docked_pallets[Coord(1, 0)] = 0
    blocker = Pallet(id=1, sku=1, position=Coord(4, 1), count=1, max_count=1)  # sits on robot 0's landing cell
    attractive = Pallet(id=2, sku=2, position=Coord(4, 2), count=1, max_count=1)  # closer, lower target row
    robot1 = Robot(id=1, position=Coord(4, 2))
    world = WorldState(
        grid=grid, robots={0: robot0, 1: robot1}, pallets={0: pallet0, 1: blocker, 2: attractive}, orders=[]
    )

    controllers = make_controllers([0, 1])
    controllers[0].assign(deque([RelocateSubGoal(pallet_id=0, target=Coord(5, 1))]))  # landing cell = (4, 1)
    targets = {0: Coord(5, 1), 1: Coord(9, 9), 2: Coord(3, 1)}
    coordinator = RelocationCoordinator(controllers, targets, world)
    coordinator._backlog = {1, 2}  # pallet 0 already "assigned" via the manual controllers[0].assign above

    coordinator.assign_idle_robots(world)

    assert controllers[1].subgoals[0].pallet_id == 1


def test_pallet_already_at_its_own_target_is_excluded_from_backlog():
    # Regression test for the real stall found empirically: a pallet whose
    # original position already equals its computed target needs no
    # relocation at all -- but if left in the backlog, the "is target free"
    # check sees the pallet's own presence there (entity_at(target) returns
    # itself, not None) and reads it as permanently blocked by a not-yet-moved
    # sibling, since nothing will ever relocate a pallet with nowhere to go.
    # Four pallets on the real Big Order stalled the whole phase this way.
    grid = Grid(width=20, height=10)
    already_placed = Pallet(id=0, sku=0, position=Coord(1, 1), count=1, max_count=1)
    needs_moving = Pallet(id=1, sku=1, position=Coord(6, 5), count=1, max_count=1)
    world = WorldState(grid=grid, robots={}, pallets={0: already_placed, 1: needs_moving}, orders=[])

    coordinator = RelocationCoordinator(make_controllers([]), {0: Coord(1, 1), 1: Coord(2, 1)}, world)

    assert coordinator._backlog == {1}


def test_is_done_requires_backlog_empty_and_controllers_idle():
    grid = Grid(width=20, height=10)
    robot = Robot(id=0, position=Coord(5, 5))
    pallet = Pallet(id=0, sku=0, position=Coord(6, 5), count=1, max_count=1)
    world = WorldState(grid=grid, robots={0: robot}, pallets={0: pallet}, orders=[])

    controllers = make_controllers([0])
    coordinator = RelocationCoordinator(controllers, {0: Coord(1, 1)}, world)

    assert not coordinator.is_done()
    coordinator.assign_idle_robots(world)
    assert not coordinator.is_done()  # assigned but controller still busy

    controllers[0].assign(deque())  # simulate completion
    assert coordinator.is_done()


def test_skips_pallets_whose_target_is_currently_occupied():
    grid = Grid(width=20, height=10)
    robot = Robot(id=0, position=Coord(5, 5))
    blocked = Pallet(id=0, sku=0, position=Coord(6, 5), count=1, max_count=1)
    free = Pallet(id=1, sku=1, position=Coord(8, 5), count=1, max_count=1)
    # An unrelated robot sits on pallet 0's assigned target, occupying it.
    occupier = Robot(id=1, position=Coord(1, 1))
    world = WorldState(grid=grid, robots={0: robot, 1: occupier}, pallets={0: blocked, 1: free}, orders=[])

    controllers = make_controllers([0])
    targets = {0: Coord(1, 1), 1: Coord(2, 1)}
    coordinator = RelocationCoordinator(controllers, targets, world)

    coordinator.assign_idle_robots(world)

    assert controllers[0].subgoals[0] == RelocateSubGoal(pallet_id=1, target=Coord(2, 1))


def test_sustained_blocked_stall_raises_runtime_error():
    grid = Grid(width=20, height=10)
    robot = Robot(id=0, position=Coord(5, 5))
    blocked = Pallet(id=0, sku=0, position=Coord(6, 5), count=1, max_count=1)
    occupier = Robot(id=1, position=Coord(1, 1))  # permanently sits on the only backlog pallet's target
    world = WorldState(grid=grid, robots={0: robot, 1: occupier}, pallets={0: blocked}, orders=[])

    controllers = make_controllers([0])
    coordinator = RelocationCoordinator(controllers, {0: Coord(1, 1)}, world)

    with pytest.raises(RuntimeError, match="relocation deadlock"):
        for _ in range(_STALL_TICKS_BEFORE_ERROR + 1):
            coordinator.assign_idle_robots(world)


def test_stuck_subgoal_past_max_ticks_raises_runtime_error():
    # Regression test for the real bug found empirically: a robot can be
    # DOCKED and keep emitting actions (e.g. periodic congestion sidesteps)
    # forever without its RelocateSubGoal ever completing, which resets the
    # generic per-tick stall counters without the underlying goal ever
    # becoming reachable. This watchdog checks direct subgoal progress
    # instead, keyed off how long ago the pallet was assigned.
    grid = Grid(width=20, height=10)
    robot = Robot(id=0, position=Coord(5, 5))
    pallet = Pallet(id=0, sku=0, position=Coord(6, 5), count=1, max_count=1, docked_to=0, dock_offset=Coord(1, 0))
    robot.docked_pallets[Coord(1, 0)] = 0
    world = WorldState(grid=grid, robots={0: robot}, pallets={0: pallet}, orders=[])

    controllers = make_controllers([0])
    controllers[0].assign(deque([RelocateSubGoal(pallet_id=0, target=Coord(1, 1))]))
    coordinator = RelocationCoordinator(controllers, {0: Coord(1, 1)}, world)
    coordinator._assigned_at[0] = 0  # simulate: assigned at tick 0, never completed

    class FakeWorld:
        tick = 10_000
        robots = world.robots
        pallets = world.pallets

        @staticmethod
        def entity_at(coord):
            return None

    with pytest.raises(RuntimeError, match="relocation stuck"):
        coordinator.assign_idle_robots(FakeWorld())

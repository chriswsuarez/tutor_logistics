from collections import Counter, deque

import pytest

from warehouse.model.action import Action
from warehouse.model.entities import Order, Pallet, Robot
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState
from warehouse.planning.reservation import ReservationTable
from warehouse.sim.engine import apply_tick
from warehouse.tasks.pallet_selector import NearestAvailablePallet
from warehouse.tasks.robot_controller import RobotController
from warehouse.tasks.task_manager import TaskManager
from warehouse.tasks.task_types import CollectSkuSubGoal, RelocateSubGoal, ReplenishSubGoal


def run_single_robot_to_completion(world: WorldState, controller: RobotController, max_ticks: int = 500) -> list[str]:
    table = ReservationTable()
    emitted_kinds: list[str] = []
    for _ in range(max_ticks):
        planned_ids = {controller.robot_id} if controller.pending_path is not None else set()
        table.sync_static_holds(world, planned_ids)
        action = controller.propose_action(world, table)
        actions = {}
        if action is not None:
            actions[controller.robot_id] = action
            emitted_kinds.append(action.kind)
        apply_tick(world, actions)
        table.prune_before(world.tick)
        if controller.is_idle():
            return emitted_kinds
    pytest.fail(f"robot did not finish within {max_ticks} ticks")


def test_collect_and_deliver_a_simple_order():
    grid = Grid(width=10, height=5)
    robot = Robot(id=0, position=Coord(5, 2))
    pallet = Pallet(id=0, sku=0, position=Coord(7, 2), count=5, max_count=5)
    order = Order(id=0, requirements=Counter({0: 2}))
    world = WorldState(grid=grid, robots={0: robot}, pallets={0: pallet}, orders=[order])

    controller = RobotController(0, NearestAvailablePallet())
    controller.assign(TaskManager.decompose(order, robot.position, world, NearestAvailablePallet()))

    emitted = run_single_robot_to_completion(world, controller)

    assert world.orders[0].fulfilled
    assert emitted.count("pick") == 2
    assert emitted.count("fulfill") == 1
    assert world.pallets[0].count == 3


def test_replenishment_cycle_when_pallet_runs_out():
    grid = Grid(width=10, height=8)  # replenishment row is y=7
    robot = Robot(id=0, position=Coord(5, 3))
    # Only one pallet for sku 0, capacity 1 -- the order needs 2, forcing a
    # replenishment trip partway through collection.
    pallet = Pallet(id=0, sku=0, position=Coord(6, 3), count=1, max_count=1)
    order = Order(id=0, requirements=Counter({0: 2}))
    world = WorldState(grid=grid, robots={0: robot}, pallets={0: pallet}, orders=[order])

    controller = RobotController(0, NearestAvailablePallet())
    controller.assign(TaskManager.decompose(order, robot.position, world, NearestAvailablePallet()))

    emitted = run_single_robot_to_completion(world, controller)

    assert world.orders[0].fulfilled
    assert "dock" in emitted
    assert "undock" in emitted
    assert emitted.count("pick") == 2
    assert world.pallets[0].docked_to is None  # ended undocked
    assert world.pallets[0].count == 0  # picked once more after the refill, then delivered


def test_replenishment_never_docks_from_the_north_of_the_pallet():
    # Regression test for a real bug found running the solver on the actual
    # Big Order: approaching a pallet from directly above (the cell north of
    # it) docks the pallet to the robot's *south* side. A south-docked pallet
    # can never reach the replenishment row (y = height-1) without going out
    # of bounds -- the robot would search forever and never find a path.
    # Here the robot starts directly north of the pallet, so "nearest
    # adjacent cell" would naturally be the forbidden one; the controller
    # must pick a different side instead and still complete successfully.
    grid = Grid(width=10, height=8)  # replenishment row is y=7
    robot = Robot(id=0, position=Coord(5, 0))
    pallet = Pallet(id=0, sku=0, position=Coord(5, 3), count=1, max_count=1)
    order = Order(id=0, requirements=Counter({0: 2}))
    world = WorldState(grid=grid, robots={0: robot}, pallets={0: pallet}, orders=[order])

    controller = RobotController(0, NearestAvailablePallet())
    controller.assign(TaskManager.decompose(order, robot.position, world, NearestAvailablePallet()))

    emitted = run_single_robot_to_completion(world, controller, max_ticks=200)

    assert world.orders[0].fulfilled
    assert "dock" in emitted
    assert "undock" in emitted
    assert world.pallets[0].docked_to is None


def test_multiple_skus_collected_before_delivery():
    grid = Grid(width=10, height=5)
    robot = Robot(id=0, position=Coord(5, 2))
    pallets = {
        0: Pallet(id=0, sku=0, position=Coord(6, 2), count=5, max_count=5),
        1: Pallet(id=1, sku=1, position=Coord(4, 2), count=5, max_count=5),
    }
    order = Order(id=0, requirements=Counter({0: 1, 1: 1}))
    world = WorldState(grid=grid, robots={0: robot}, pallets=pallets, orders=[order])

    controller = RobotController(0, NearestAvailablePallet())
    controller.assign(TaskManager.decompose(order, robot.position, world, NearestAvailablePallet()))

    run_single_robot_to_completion(world, controller)

    assert world.orders[0].fulfilled
    assert world.pallets[0].count == 4
    assert world.pallets[1].count == 4


def test_controller_becomes_idle_with_no_assignment():
    grid = Grid(width=10, height=5)
    robot = Robot(id=0, position=Coord(5, 2))
    world = WorldState(grid=grid, robots={0: robot}, pallets={}, orders=[])
    controller = RobotController(0, NearestAvailablePallet())
    table = ReservationTable()
    table.sync_static_holds(world, set())
    assert controller.is_idle()
    assert controller.propose_action(world, table) is None


def test_shared_pick_budget_prevents_conflicting_picks_on_a_low_stock_pallet():
    # Regression test for a real bug found running the solver on the actual
    # Big Order: two robots adjacent to the same pallet, whose start-of-tick
    # count is 1, each independently see count > 0 and both propose a pick.
    # Without a shared per-tick claim, the engine rejects the batch (combined
    # picks exceed count), both get dropped and forced to replan, and -- since
    # neither robot's own decision-making changed -- they propose the exact
    # same conflicting picks again next tick, forever, and the pallet's count
    # never actually decrements.
    grid = Grid(width=10, height=5)
    pallet = Pallet(id=0, sku=0, position=Coord(5, 2), count=1, max_count=5)
    robot_a = Robot(id=0, position=Coord(4, 2))
    robot_b = Robot(id=1, position=Coord(6, 2))
    world = WorldState(grid=grid, robots={0: robot_a, 1: robot_b}, pallets={0: pallet}, orders=[])

    controller_a = RobotController(0, NearestAvailablePallet())
    controller_b = RobotController(1, NearestAvailablePallet())
    controller_a.assign(deque([CollectSkuSubGoal(sku=0, quantity=1)]))
    controller_b.assign(deque([CollectSkuSubGoal(sku=0, quantity=1)]))

    table = ReservationTable()
    table.sync_static_holds(world, set())
    claimed_picks: Counter = Counter()
    claimed_docks: set[int] = set()

    action_a = controller_a.propose_action(world, table, claimed_picks, claimed_docks)
    action_b = controller_b.propose_action(world, table, claimed_picks, claimed_docks)

    assert action_a == Action("pick", 5, 2)
    assert action_b is None  # budget already spoken for this tick, must wait rather than double-claim

    apply_tick(world, {0: action_a})  # action_b is None -> not included, robot 1 simply waits
    assert world.pallets[0].count == 0
    assert world.robots[0].storage[0] == 1
    assert world.robots[1].storage[0] == 0


def test_shared_dock_budget_prevents_two_robots_docking_the_same_pallet():
    grid = Grid(width=10, height=5)
    pallet = Pallet(id=0, sku=0, position=Coord(5, 2), count=5, max_count=5)
    robot_a = Robot(id=0, position=Coord(4, 2))
    robot_b = Robot(id=1, position=Coord(6, 2))
    world = WorldState(grid=grid, robots={0: robot_a, 1: robot_b}, pallets={0: pallet}, orders=[])

    controller_a = RobotController(0, NearestAvailablePallet())
    controller_b = RobotController(1, NearestAvailablePallet())
    resume = CollectSkuSubGoal(sku=0, quantity=1)
    controller_a.assign(deque([ReplenishSubGoal(pallet_id=0, resume=resume, origin=pallet.position)]))
    controller_b.assign(deque([ReplenishSubGoal(pallet_id=0, resume=resume, origin=pallet.position)]))

    table = ReservationTable()
    table.sync_static_holds(world, set())
    claimed_picks: Counter = Counter()
    claimed_docks: set[int] = set()

    action_a = controller_a.propose_action(world, table, claimed_picks, claimed_docks)
    action_b = controller_b.propose_action(world, table, claimed_picks, claimed_docks)

    assert action_a == Action("dock", 5, 2)
    assert action_b is None  # pallet already claimed for docking this tick


def test_relocate_drags_a_pallet_to_its_target_and_ends_undocked_there():
    grid = Grid(width=10, height=5)
    robot = Robot(id=0, position=Coord(5, 2))
    pallet = Pallet(id=0, sku=0, position=Coord(6, 2), count=5, max_count=5)
    world = WorldState(grid=grid, robots={0: robot}, pallets={0: pallet}, orders=[])

    controller = RobotController(0, NearestAvailablePallet())
    controller.assign(deque([RelocateSubGoal(pallet_id=0, target=Coord(1, 1))]))

    emitted = run_single_robot_to_completion(world, controller)

    assert "dock" in emitted
    assert "undock" in emitted
    assert world.pallets[0].position == Coord(1, 1)
    assert world.pallets[0].docked_to is None


def test_relocate_is_a_noop_when_pallet_already_at_target():
    grid = Grid(width=10, height=5)
    robot = Robot(id=0, position=Coord(5, 2))
    pallet = Pallet(id=0, sku=0, position=Coord(6, 2), count=5, max_count=5)
    world = WorldState(grid=grid, robots={0: robot}, pallets={0: pallet}, orders=[])

    controller = RobotController(0, NearestAvailablePallet())
    controller.assign(deque([RelocateSubGoal(pallet_id=0, target=Coord(6, 2))]))

    table = ReservationTable()
    table.sync_static_holds(world, set())
    action = controller.propose_action(world, table)

    assert action is None
    assert controller.is_idle()  # subgoal dropped immediately, no dock/undock needed


def test_relocate_permits_the_north_approach():
    # Explicit regression guard that _step_relocate does NOT share
    # ReplenishSubGoal's north-approach exclusion: relocation targets sit
    # deep inside the grid, never near y=height-1, so docking a pallet to the
    # robot's south side (by approaching from the north) is never a problem
    # here. The robot starts directly north of the pallet, so "nearest
    # adjacent cell" naturally is the side replenishment would have forbidden.
    grid = Grid(width=10, height=8)
    robot = Robot(id=0, position=Coord(5, 1))
    pallet = Pallet(id=0, sku=0, position=Coord(5, 3), count=5, max_count=5)
    world = WorldState(grid=grid, robots={0: robot}, pallets={0: pallet}, orders=[])

    controller = RobotController(0, NearestAvailablePallet())
    controller.assign(deque([RelocateSubGoal(pallet_id=0, target=Coord(2, 1))]))

    emitted = run_single_robot_to_completion(world, controller, max_ticks=200)

    assert world.pallets[0].position == Coord(2, 1)
    assert world.pallets[0].docked_to is None


def test_relocate_shares_dock_budget_with_claimed_docks():
    grid = Grid(width=10, height=5)
    pallet = Pallet(id=0, sku=0, position=Coord(5, 2), count=5, max_count=5)
    robot_a = Robot(id=0, position=Coord(4, 2))
    robot_b = Robot(id=1, position=Coord(6, 2))
    world = WorldState(grid=grid, robots={0: robot_a, 1: robot_b}, pallets={0: pallet}, orders=[])

    controller_a = RobotController(0, NearestAvailablePallet())
    controller_b = RobotController(1, NearestAvailablePallet())
    controller_a.assign(deque([RelocateSubGoal(pallet_id=0, target=Coord(1, 1))]))
    controller_b.assign(deque([RelocateSubGoal(pallet_id=0, target=Coord(1, 1))]))

    table = ReservationTable()
    table.sync_static_holds(world, set())
    claimed_picks: Counter = Counter()
    claimed_docks: set[int] = set()

    action_a = controller_a.propose_action(world, table, claimed_picks, claimed_docks)
    action_b = controller_b.propose_action(world, table, claimed_picks, claimed_docks)

    assert action_a == Action("dock", 5, 2)
    assert action_b is None  # pallet already claimed for docking this tick


def test_collect_waits_rather_than_double_replenish_a_pallet_already_being_dragged():
    # Regression test for a real bug found running the solver on the actual
    # Big Order: pallet has zero stock and is already docked to (being
    # replenished by) a DIFFERENT robot -- mid-transit, not resting anywhere
    # meaningful. A second robot independently needing the same SKU must not
    # fabricate its own ReplenishSubGoal capturing that transient position as
    # "origin": it would try to permanently settle a drag on whatever cell
    # the first robot happens to be passing through right now, which can be
    # arbitrarily far from -- and much harder to reach than -- the pallet's
    # true rest position (this produced a near-unreachable target deep in
    # busy traffic that burned full-budget A* searches every tick for
    # hundreds of ticks). It should simply wait instead.
    grid = Grid(width=10, height=8)
    pallet = Pallet(id=0, sku=0, position=Coord(3, 3), count=0, max_count=5, docked_to=1, dock_offset=Coord(1, 0))
    other_robot = Robot(id=1, position=Coord(2, 3))
    other_robot.docked_pallets[Coord(1, 0)] = 0
    robot = Robot(id=0, position=Coord(5, 3))
    world = WorldState(grid=grid, robots={0: robot, 1: other_robot}, pallets={0: pallet}, orders=[])

    controller = RobotController(0, NearestAvailablePallet())
    controller.assign(deque([CollectSkuSubGoal(sku=0, quantity=1)]))

    table = ReservationTable()
    table.sync_static_holds(world, set())
    action = controller.propose_action(world, table)

    assert action is None
    assert len(controller.subgoals) == 1  # no bogus ReplenishSubGoal appended
    assert isinstance(controller.subgoals[0], CollectSkuSubGoal)


def test_replenish_fallback_skips_a_pallet_boxed_in_by_other_pallets():
    # Regression test for a real bug found running the solver on the actual
    # Big Order: a pallet stranded (by an earlier, unrelated replenishment
    # trip) between two *other* pallets, with its only remaining side being
    # the excluded north approach, is permanently unreachable. Selecting it
    # anyway (because it happens to be nearest) deadlocks every future need
    # for that SKU forever, even though a farther, reachable pallet of the
    # same SKU exists.
    grid = Grid(width=10, height=6)  # replenishment row is y=5
    boxed_in = Pallet(id=0, sku=7, position=Coord(5, 5), count=0, max_count=100)
    blocker_west = Pallet(id=1, sku=1, position=Coord(4, 5), count=10, max_count=10)
    blocker_east = Pallet(id=2, sku=1, position=Coord(6, 5), count=10, max_count=10)
    reachable = Pallet(id=3, sku=7, position=Coord(2, 2), count=0, max_count=100)
    robot = Robot(id=0, position=Coord(5, 4))  # directly north of the boxed-in pallet, i.e. nearest to it
    world = WorldState(
        grid=grid,
        robots={0: robot},
        pallets={0: boxed_in, 1: blocker_west, 2: blocker_east, 3: reachable},
        orders=[],
    )

    controller = RobotController(0, NearestAvailablePallet())
    chosen = controller._nearest_pallet_of_sku(7, robot.position, world)

    assert chosen == 3  # skips the nearer-but-structurally-unreachable pallet 0

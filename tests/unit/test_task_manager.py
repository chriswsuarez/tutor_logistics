from collections import Counter

from warehouse.model.entities import Order, Pallet, Robot
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState
from warehouse.tasks.order_selector import FifoOrderSelector
from warehouse.tasks.pallet_selector import NearestAvailablePallet
from warehouse.tasks.robot_controller import RobotController
from warehouse.tasks.task_manager import TaskManager
from warehouse.tasks.task_types import CollectSkuSubGoal, DeliverSubGoal


def make_world(orders, pallets=None):
    grid = Grid(width=60, height=40)
    robots = {0: Robot(id=0, position=Coord(0, 0)), 1: Robot(id=1, position=Coord(1, 0))}
    return WorldState(grid=grid, robots=robots, pallets=pallets or {}, orders=orders)


def make_task_manager():
    pallet_selector = NearestAvailablePallet()
    controllers = {
        0: RobotController(0, pallet_selector),
        1: RobotController(1, pallet_selector),
    }
    return TaskManager(controllers, FifoOrderSelector(), pallet_selector), controllers


def test_decompose_builds_one_collect_subgoal_per_sku_then_deliver_nearest_first():
    order = Order(id=0, requirements=Counter({3: 2, 7: 1}))
    pallets = {
        0: Pallet(id=0, sku=3, position=Coord(20, 0), count=5, max_count=5),  # far
        1: Pallet(id=1, sku=7, position=Coord(1, 0), count=5, max_count=5),  # near
    }
    world = WorldState(
        grid=Grid(width=60, height=40), robots={0: Robot(id=0, position=Coord(0, 0))}, pallets=pallets, orders=[order]
    )

    subgoals = list(TaskManager.decompose(order, Coord(0, 0), world, NearestAvailablePallet()))

    assert subgoals[:-1] == [CollectSkuSubGoal(sku=7, quantity=1), CollectSkuSubGoal(sku=3, quantity=2)]
    assert subgoals[-1] == DeliverSubGoal(order_id=0)


def test_decompose_falls_back_to_any_pallet_when_sku_has_no_stock():
    order = Order(id=0, requirements=Counter({3: 1}))
    pallets = {0: Pallet(id=0, sku=3, position=Coord(5, 0), count=0, max_count=5)}
    world = WorldState(
        grid=Grid(width=60, height=40), robots={0: Robot(id=0, position=Coord(0, 0))}, pallets=pallets, orders=[order]
    )

    subgoals = list(TaskManager.decompose(order, Coord(0, 0), world, NearestAvailablePallet()))

    assert subgoals == [CollectSkuSubGoal(sku=3, quantity=1), DeliverSubGoal(order_id=0)]


def test_assign_idle_robots_gives_each_idle_robot_a_distinct_order():
    pallets = {
        0: Pallet(id=0, sku=0, position=Coord(2, 0), count=5, max_count=5),
        1: Pallet(id=1, sku=1, position=Coord(2, 0), count=5, max_count=5),
    }
    orders = [Order(id=0, requirements=Counter({0: 1})), Order(id=1, requirements=Counter({1: 1}))]
    world = make_world(orders, pallets)
    task_manager, controllers = make_task_manager()

    task_manager.assign_idle_robots(world)

    assert not controllers[0].is_idle()
    assert not controllers[1].is_idle()
    assert task_manager.claimed_order_ids == {0, 1}


def test_assign_idle_robots_skips_already_busy_robots():
    pallets = {
        0: Pallet(id=0, sku=0, position=Coord(2, 0), count=5, max_count=5),
        1: Pallet(id=1, sku=1, position=Coord(2, 0), count=5, max_count=5),
    }
    orders = [Order(id=0, requirements=Counter({0: 1})), Order(id=1, requirements=Counter({1: 1}))]
    world = make_world(orders, pallets)
    task_manager, controllers = make_task_manager()

    controllers[0].assign(TaskManager.decompose(orders[0], Coord(0, 0), world, task_manager.pallet_selector))
    task_manager.claimed_order_ids.add(0)
    task_manager.assign_idle_robots(world)

    assert task_manager.claimed_order_ids == {0, 1}
    assert controllers[1].subgoals[-1].order_id == 1


def test_assign_idle_robots_does_nothing_when_no_orders_left():
    orders = [Order(id=0, requirements=Counter({0: 1}), fulfilled=True)]
    world = make_world(orders)
    task_manager, controllers = make_task_manager()

    task_manager.assign_idle_robots(world)

    assert controllers[0].is_idle()
    assert controllers[1].is_idle()
    assert task_manager.claimed_order_ids == set()

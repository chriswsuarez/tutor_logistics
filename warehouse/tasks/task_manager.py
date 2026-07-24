from __future__ import annotations

from collections import deque

from warehouse.model.entities import Order
from warehouse.model.grid import Coord
from warehouse.model.world import WorldState
from warehouse.tasks.order_selector import OrderSelectionPolicy
from warehouse.tasks.pallet_selector import PalletSelectionPolicy, nearest_pallet_of_sku
from warehouse.tasks.robot_controller import RobotController
from warehouse.tasks.task_types import CollectSkuSubGoal, DeliverSubGoal, SubGoal


def _manhattan(a: Coord, b: Coord) -> int:
    return abs(a.x - b.x) + abs(a.y - b.y)


class TaskManager:
    """Owns the order queue and keeps every idle robot busy independently —
    nothing here makes robots lockstep, each RobotController progresses its
    own subgoal queue at its own pace."""

    def __init__(
        self,
        controllers: dict[int, RobotController],
        order_selector: OrderSelectionPolicy,
        pallet_selector: PalletSelectionPolicy,
    ):
        self.controllers = controllers
        self.order_selector = order_selector
        self.pallet_selector = pallet_selector
        self.claimed_order_ids: set[int] = set()

    def assign_idle_robots(self, world: WorldState) -> None:
        for robot_id in sorted(self.controllers):
            controller = self.controllers[robot_id]
            if not controller.is_idle():
                continue
            position = world.robots[robot_id].position
            order_id = self.order_selector.select(world, self.claimed_order_ids, position)
            if order_id is None:
                continue  # no unclaimed unfulfilled orders left right now
            self.claimed_order_ids.add(order_id)
            subgoals = self.decompose(world.orders[order_id], position, world, self.pallet_selector)
            controller.assign(subgoals)

    @staticmethod
    def decompose(order: Order, from_position: Coord, world: WorldState, pallet_selector: PalletSelectionPolicy) -> "deque[SubGoal]":
        """Greedy nearest-neighbor tour over the order's distinct SKUs,
        starting from the assigning robot's current position: at each step,
        pick whichever remaining SKU's likely pallet (current selector's
        choice, or the nearest instance regardless of stock if none has any
        right now) is closest to wherever the tour has reached so far, then
        continue from there. This is only a routing *estimate* -- the actual
        pick still re-selects a pallet live each time (stock may have moved
        on) -- but it replaces an arbitrary SKU-id ordering that could zigzag
        across the whole grid with a materially shorter approximate route."""
        remaining: dict[int, int] = dict(order.requirements)
        subgoals: deque[SubGoal] = deque()
        position = from_position
        while remaining:
            best_sku = None
            best_pallet_pos = None
            best_distance = None
            for sku in remaining:
                pallet_id = pallet_selector.select(sku, position, world)
                if pallet_id is None:
                    pallet_id = nearest_pallet_of_sku(sku, position, world, require_stock=False)
                pallet_pos = world.pallets[pallet_id].position
                distance = _manhattan(position, pallet_pos)
                if best_distance is None or distance < best_distance:
                    best_distance, best_sku, best_pallet_pos = distance, sku, pallet_pos
            quantity = remaining.pop(best_sku)
            subgoals.append(CollectSkuSubGoal(sku=best_sku, quantity=quantity))
            position = best_pallet_pos
        subgoals.append(DeliverSubGoal(order_id=order.id))
        return subgoals

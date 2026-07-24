from __future__ import annotations

from typing import Optional, Protocol

from warehouse.model.grid import Coord
from warehouse.model.world import WorldState
from warehouse.tasks.pallet_selector import PalletSelectionPolicy, nearest_pallet_of_sku


def _manhattan(a: Coord, b: Coord) -> int:
    return abs(a.x - b.x) + abs(a.y - b.y)


class OrderSelectionPolicy(Protocol):
    def select(self, world: WorldState, claimed_order_ids: set[int], from_coord: Coord) -> Optional[int]:
        """Return the id of an unfulfilled, unclaimed order to assign next
        (an idle robot currently at `from_coord`), or None if none remain."""
        ...


class FifoOrderSelector:
    """Lowest unfulfilled, unclaimed order id, ignoring locality entirely."""

    def select(self, world: WorldState, claimed_order_ids: set[int], from_coord: Coord) -> Optional[int]:
        for order in world.orders:
            if not order.fulfilled and order.id not in claimed_order_ids:
                return order.id
        return None


class NearestOrderSelector:
    """Assigns the unclaimed, unfulfilled order whose closest required SKU
    (by nearest-pallet distance, ignoring current stock so an order isn't
    penalized just because its cheapest pallet happens to be empty right
    now) is nearest to the idle robot -- a cheap proxy for "how much travel
    would starting this order cost from here", isolated behind
    OrderSelectionPolicy so a fancier multi-item routing cost or SKU-sharing
    batch strategy is a drop-in replacement."""

    def __init__(self, pallet_selector: PalletSelectionPolicy):
        self.pallet_selector = pallet_selector

    def select(self, world: WorldState, claimed_order_ids: set[int], from_coord: Coord) -> Optional[int]:
        best_order_id: Optional[int] = None
        best_key: Optional[tuple[int, int]] = None
        for order in world.orders:
            if order.fulfilled or order.id in claimed_order_ids:
                continue
            key = (self._closest_sku_distance(order.requirements, from_coord, world), order.id)
            if best_key is None or key < best_key:
                best_key = key
                best_order_id = order.id
        return best_order_id

    def _closest_sku_distance(self, requirements, from_coord: Coord, world: WorldState) -> int:
        best = None
        for sku in requirements:
            pallet_id = self.pallet_selector.select(sku, from_coord, world)
            if pallet_id is None:
                pallet_id = nearest_pallet_of_sku(sku, from_coord, world, require_stock=False)
            distance = _manhattan(from_coord, world.pallets[pallet_id].position)
            if best is None or distance < best:
                best = distance
        return best

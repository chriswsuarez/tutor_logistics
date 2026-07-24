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
    """Assigns the unclaimed, unfulfilled order whose FARTHEST required SKU
    (by nearest-pallet distance, ignoring current stock so an order isn't
    penalized just because its cheapest pallet happens to be empty right
    now) is closest to the idle robot -- a cheap proxy for "how much travel
    would starting this order cost from here", isolated behind
    OrderSelectionPolicy so a fancier multi-item routing cost or SKU-sharing
    batch strategy is a drop-in replacement.

    Farthest, not closest: once pallets are relocated into a compact
    near-y=0 band (see warehouse/tasks/relocation.py), nearly every order
    shares at least one of the handful of near-universal SKUs (e.g. one
    present in 995 of 1000 orders on the real Big Order) sitting right next
    to the fulfillment row -- so the *closest* required SKU is almost always
    ~0 distance for every order alike, making it useless for telling orders
    apart. The tour TaskManager.decompose builds is star-shaped from a
    near-fixed hub: its length is dominated by the trip out to (and back
    from) whichever required SKU sits farthest from here, not by whichever
    happens to be nearest -- so farthest-required-SKU distance is the
    proxy that actually discriminates between a cheap order and an
    expensive one now."""

    def __init__(self, pallet_selector: PalletSelectionPolicy):
        self.pallet_selector = pallet_selector

    def select(self, world: WorldState, claimed_order_ids: set[int], from_coord: Coord) -> Optional[int]:
        best_order_id: Optional[int] = None
        best_key: Optional[tuple[int, int]] = None
        for order in world.orders:
            if order.fulfilled or order.id in claimed_order_ids:
                continue
            key = (self._farthest_sku_distance(order.requirements, from_coord, world), order.id)
            if best_key is None or key < best_key:
                best_key = key
                best_order_id = order.id
        return best_order_id

    def _farthest_sku_distance(self, requirements, from_coord: Coord, world: WorldState) -> int:
        worst = None
        for sku in requirements:
            pallet_id = self.pallet_selector.select(sku, from_coord, world)
            if pallet_id is None:
                pallet_id = nearest_pallet_of_sku(sku, from_coord, world, require_stock=False)
            distance = _manhattan(from_coord, world.pallets[pallet_id].position)
            if worst is None or distance > worst:
                worst = distance
        return worst

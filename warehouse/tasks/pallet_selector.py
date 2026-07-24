from __future__ import annotations

from typing import Optional, Protocol

from warehouse.model.grid import Coord
from warehouse.model.world import WorldState


class PalletSelectionPolicy(Protocol):
    def select(self, sku: int, from_coord: Coord, world: WorldState) -> Optional[int]:
        """Return the id of a pallet of `sku` with stock available, or None if
        every pallet of that SKU is currently empty."""
        ...


def _manhattan(a: Coord, b: Coord) -> int:
    return abs(a.x - b.x) + abs(a.y - b.y)


def has_reachable_neighbor(position: Coord, world: WorldState, excluded_offsets: frozenset[Coord] = frozenset()) -> bool:
    """True if at least one of `position`'s orthogonal neighbors (other than
    any in `excluded_offsets`) is not permanently occupied by another pallet.
    Pallets never move on their own, so a pallet with every non-excluded side
    taken by other pallets can *never* be approached by anyone -- a robot
    currently standing on a side doesn't disqualify it, robots eventually
    move on. Any pallet-selection policy must check this, not just distance:
    the warehouse can (and, empirically, does) place a pallet fully boxed in
    by others, and simple nearest-with-stock selection would otherwise
    deadlock forever on it while farther, actually-reachable siblings of the
    same SKU sit unused."""
    for cell in world.grid.neighbors4(position):
        offset = Coord(cell.x - position.x, cell.y - position.y)
        if offset in excluded_offsets:
            continue
        occupant = world.entity_at(cell)
        if occupant is None or occupant[0] != "pallet":
            return True
    return False


def nearest_pallet_of_sku(
    sku: int,
    from_coord: Coord,
    world: WorldState,
    require_stock: bool,
    require_reachable: bool = False,
    excluded_offsets: frozenset[Coord] = frozenset(),
    fallback_to_nearest: bool = True,
) -> Optional[int]:
    """Nearest pallet of `sku` to `from_coord` (ties broken by pallet id).
    With `require_stock=True`, only considers pallets with count > 0 (returns
    None if every instance is currently empty); with False, considers every
    pallet of that SKU regardless of stock -- used for route-planning
    distance estimates and the replenish fallback, where "nearest, period" is
    the relevant question rather than "nearest with stock right now".

    With `require_reachable=True`, skips candidates with no structurally
    reachable side (see `has_reachable_neighbor`) in favor of the next
    nearest. If none qualify, `fallback_to_nearest=True` returns the plain
    nearest anyway (nothing better to try -- used by the replenish fallback,
    which has no further fallback of its own); `fallback_to_nearest=False`
    returns None instead (used by ordinary stocked-pallet selection: a pallet
    that has stock but is permanently unreachable, e.g. boxed in by other
    pallets on every side, must never be "selected" just because it's the
    only one with stock -- that would deadlock every future need for the SKU
    forever. Returning None here tells the caller "no usable stock right
    now," which correctly falls through to the replenish path instead,
    where a reachable-but-empty sibling can be refilled and used)."""
    candidates = [p for p in world.pallets.values() if p.sku == sku and (not require_stock or p.count > 0)]
    if not candidates:
        return None
    candidates.sort(key=lambda p: (_manhattan(from_coord, p.position), p.id))
    if require_reachable:
        for pallet in candidates:
            if has_reachable_neighbor(pallet.position, world, excluded_offsets):
                return pallet.id
        return candidates[0].id if fallback_to_nearest else None
    return candidates[0].id


class NearestAvailablePallet:
    """V1 policy: nearest pallet of the SKU that currently has stock, skipping
    any candidate with no structurally reachable side in favor of the next
    nearest, and reporting "none available" rather than a stocked-but-
    unreachable pallet if that's all there is. Kept isolated behind
    PalletSelectionPolicy so a later batching/high-runner placement strategy
    is a drop-in replacement."""

    def select(self, sku: int, from_coord: Coord, world: WorldState) -> Optional[int]:
        return nearest_pallet_of_sku(
            sku, from_coord, world, require_stock=True, require_reachable=True, fallback_to_nearest=False
        )

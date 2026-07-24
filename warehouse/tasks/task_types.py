from __future__ import annotations

from dataclasses import dataclass

from warehouse.model.grid import Coord


class SubGoal:
    """Marker base for a robot's queued sub-goals. Deliberately plain data —
    any planning-in-progress state (which pallet was picked, a committed
    path) lives on RobotController instead, keeping these declarative."""


@dataclass
class CollectSkuSubGoal(SubGoal):
    sku: int
    quantity: int  # total units of this SKU the order needs (fixed; progress is read from
    # robot.storage[sku] directly rather than a separately-tracked mutable counter)


@dataclass
class DeliverSubGoal(SubGoal):
    order_id: int


@dataclass
class ReplenishSubGoal(SubGoal):
    pallet_id: int
    resume: SubGoal  # the CollectSkuSubGoal to re-enter once the pallet is refilled
    origin: Coord  # the pallet's position before this trip, so it can be dragged back there
    # afterward instead of being left parked on the replenishment row -- otherwise pallets
    # accumulate there over many trips until some become permanently wedged between others,
    # unreachable for all future replenishment

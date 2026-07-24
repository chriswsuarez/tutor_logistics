from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from typing import Optional

from warehouse.model.entities import Order, Pallet, Robot
from warehouse.model.grid import Coord, Grid

EntityRef = tuple[str, int]  # ("robot", id) | ("pallet", id)


@dataclass
class WorldState:
    """Single source of truth for both the simulator and the pathfinder.

    `occupancy` is the one position->entity map both consult for collision and
    adjacency checks; it must be kept in sync by whatever mutates positions
    (only warehouse.sim.engine should ever do that during a real run).
    """

    grid: Grid
    robots: dict[int, Robot]
    pallets: dict[int, Pallet]
    orders: list[Order]
    tick: int = 0
    occupancy: dict[Coord, EntityRef] = field(default_factory=dict)
    orders_by_signature: dict[tuple, list[int]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.occupancy:
            self.occupancy = self.rebuild_occupancy()
        if not self.orders_by_signature:
            self.orders_by_signature = self._build_signature_index()

    def _build_signature_index(self) -> dict[tuple, list[int]]:
        index: dict[tuple, list[int]] = defaultdict(list)
        for order in self.orders:
            index[order.signature()].append(order.id)
        for ids in index.values():
            ids.sort()
        return dict(index)

    def rebuild_occupancy(self) -> dict[Coord, EntityRef]:
        """Recompute occupancy from scratch. Used at init and as a debug/test
        assertion (`world.occupancy == world.rebuild_occupancy()`) to catch
        incremental-bookkeeping drift bugs."""
        occ: dict[Coord, EntityRef] = {}
        for robot in self.robots.values():
            occ[robot.position] = ("robot", robot.id)
        for pallet in self.pallets.values():
            occ[pallet.position] = ("pallet", pallet.id)
        return occ

    def entity_at(self, coord: Coord) -> Optional[EntityRef]:
        return self.occupancy.get(coord)

    def robot_footprint(self, robot_id: int) -> set[Coord]:
        """A robot's own cell plus every pallet currently docked to it."""
        robot = self.robots[robot_id]
        cells = {robot.position}
        for offset in robot.docked_pallets:
            cells.add(robot.position + offset)
        return cells

    def unfulfilled_order_ids(self, signature: tuple) -> list[int]:
        return [oid for oid in self.orders_by_signature.get(signature, ()) if not self.orders[oid].fulfilled]

    def all_orders_fulfilled(self) -> bool:
        return all(order.fulfilled for order in self.orders)

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from typing import Optional

from warehouse.model.grid import Coord


def counter_signature(counter: "Counter[int]") -> tuple[tuple[int, int], ...]:
    """Canonical hashable signature of a multiset, skipping zero-count entries
    (Counter arithmetic can leave zero/negative keys behind) so it compares
    equal to an Order's signature regardless of how the Counter was built."""
    return tuple(sorted((sku, qty) for sku, qty in counter.items() if qty > 0))


@dataclass
class Pallet:
    id: int
    sku: int
    position: Coord
    count: int
    max_count: int
    docked_to: Optional[int] = None  # robot id, or None if free-standing
    dock_offset: Optional[Coord] = None  # unit vector from the owning robot, fixed at dock time


@dataclass
class Robot:
    id: int
    position: Coord
    # sku -> quantity currently carried. Unbounded: the spec never gives robots a
    # storage capacity (unlike pallets' maxCount), so no "is robot full" check exists.
    storage: Counter[int] = field(default_factory=Counter)
    # offset unit vector -> pallet id. A plain dict caps docking at 4 automatically
    # since there are only 4 unit vectors; a docked pallet's absolute position is
    # always `robot.position + offset`.
    docked_pallets: dict[Coord, int] = field(default_factory=dict)


@dataclass
class Order:
    id: int
    requirements: Counter[int]
    fulfilled: bool = False

    def signature(self) -> tuple[tuple[int, int], ...]:
        return counter_signature(self.requirements)

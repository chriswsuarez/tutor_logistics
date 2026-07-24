from __future__ import annotations

from typing import Iterator, NamedTuple


class Coord(NamedTuple):
    x: int
    y: int

    def __add__(self, other: "Coord") -> "Coord":  # type: ignore[override]
        return Coord(self.x + other.x, self.y + other.y)


# Unit vectors used both for movement and for docked-pallet offsets.
NORTH = Coord(0, -1)
SOUTH = Coord(0, 1)
WEST = Coord(-1, 0)
EAST = Coord(1, 0)
UNIT_VECTORS = (NORTH, SOUTH, WEST, EAST)


class Grid:
    """The 60x40 warehouse grid. Dimensions are a puzzle constant, never read
    from the worklist file — the constructor args exist only so unit tests
    can exercise pathfinding on a small grid without waiting on a big one."""

    def __init__(self, width: int = 60, height: int = 40) -> None:
        self.width = width
        self.height = height

    def in_bounds(self, c: Coord) -> bool:
        return 0 <= c.x < self.width and 0 <= c.y < self.height

    def neighbors4(self, c: Coord) -> Iterator[Coord]:
        for d in UNIT_VECTORS:
            n = c + d
            if self.in_bounds(n):
                yield n

    def is_fulfillment(self, c: Coord) -> bool:
        return c.y == 0

    def is_replenishment(self, c: Coord) -> bool:
        return c.y == self.height - 1

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass

from warehouse.model.entities import Order, Pallet, Robot
from warehouse.model.grid import Coord, Grid
from warehouse.model.world import WorldState


class ParseError(ValueError):
    pass


@dataclass
class ProblemInstance:
    robot_starts: list[Coord]
    sku_capacities: list[int]
    pallets: list[tuple[Coord, int]]  # (position, sku)
    orders: list[list[int]]  # list of sku lists


def parse_worklist(path: str) -> ProblemInstance:
    with open(path) as f:
        lines = [line.strip() for line in f if line.strip() != ""]

    pos = 0

    def next_line() -> str:
        nonlocal pos
        if pos >= len(lines):
            raise ParseError(f"unexpected end of file at line {pos}")
        line = lines[pos]
        pos += 1
        return line

    def next_int() -> int:
        line = next_line()
        try:
            return int(line)
        except ValueError as exc:
            raise ParseError(f"expected an integer, got {line!r} at line {pos}") from exc

    def next_ints() -> list[int]:
        line = next_line()
        try:
            return [int(tok) for tok in line.split()]
        except ValueError as exc:
            raise ParseError(f"expected integers, got {line!r} at line {pos}") from exc

    num_robots = next_int()
    robot_starts = []
    for _ in range(num_robots):
        x, y = next_ints()
        robot_starts.append(Coord(x, y))

    num_skus = next_int()
    sku_capacities = [next_int() for _ in range(num_skus)]

    num_pallets = next_int()
    pallets = []
    for _ in range(num_pallets):
        x, y, sku = next_ints()
        if not (0 <= sku < num_skus):
            raise ParseError(f"pallet references out-of-range sku {sku}")
        pallets.append((Coord(x, y), sku))

    num_orders = next_int()
    orders = []
    for _ in range(num_orders):
        orders.append(next_ints())

    if pos != len(lines):
        raise ParseError(f"trailing unparsed content starting at line {pos}")

    return ProblemInstance(
        robot_starts=robot_starts,
        sku_capacities=sku_capacities,
        pallets=pallets,
        orders=orders,
    )


def build_world(instance: ProblemInstance, grid: Grid | None = None) -> WorldState:
    grid = grid or Grid()

    robots = {
        i: Robot(id=i, position=pos) for i, pos in enumerate(instance.robot_starts)
    }

    pallets = {}
    for i, (pos, sku) in enumerate(instance.pallets):
        capacity = instance.sku_capacities[sku]
        pallets[i] = Pallet(id=i, sku=sku, position=pos, count=capacity, max_count=capacity)

    orders = [
        Order(id=i, requirements=Counter(skus)) for i, skus in enumerate(instance.orders)
    ]

    return WorldState(grid=grid, robots=robots, pallets=pallets, orders=orders)

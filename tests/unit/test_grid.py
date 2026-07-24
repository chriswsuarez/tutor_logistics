from warehouse.model.grid import Coord, Grid


def test_in_bounds():
    grid = Grid(width=60, height=40)
    assert grid.in_bounds(Coord(0, 0))
    assert grid.in_bounds(Coord(59, 39))
    assert not grid.in_bounds(Coord(60, 0))
    assert not grid.in_bounds(Coord(0, 40))
    assert not grid.in_bounds(Coord(-1, 0))


def test_neighbors4_interior():
    grid = Grid(width=60, height=40)
    neighbors = set(grid.neighbors4(Coord(5, 5)))
    assert neighbors == {Coord(5, 4), Coord(5, 6), Coord(4, 5), Coord(6, 5)}


def test_neighbors4_corner_clips_out_of_bounds():
    grid = Grid(width=60, height=40)
    neighbors = set(grid.neighbors4(Coord(0, 0)))
    assert neighbors == {Coord(1, 0), Coord(0, 1)}


def test_fulfillment_and_replenishment_rows():
    grid = Grid(width=60, height=40)
    assert grid.is_fulfillment(Coord(30, 0))
    assert not grid.is_fulfillment(Coord(30, 1))
    assert grid.is_replenishment(Coord(30, 39))
    assert not grid.is_replenishment(Coord(30, 38))


def test_coord_add_is_unit_vector_offset():
    assert Coord(5, 5) + Coord(0, -1) == Coord(5, 4)

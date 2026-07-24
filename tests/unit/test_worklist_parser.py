import pytest

from warehouse.io.worklist_parser import ParseError, build_world, parse_worklist
from warehouse.model.grid import Coord

BIG_ORDER_PATH = "task/BIG_ORDER.txt"


def test_big_order_counts_match_the_real_file():
    instance = parse_worklist(BIG_ORDER_PATH)
    assert len(instance.robot_starts) == 5
    assert len(instance.sku_capacities) == 100
    assert len(instance.pallets) == 240
    assert len(instance.orders) == 1000


def test_big_order_capacities_in_expected_range():
    instance = parse_worklist(BIG_ORDER_PATH)
    assert all(80 <= cap <= 300 for cap in instance.sku_capacities)


def test_big_order_pallets_never_in_fulfillment_or_replenishment_rows():
    instance = parse_worklist(BIG_ORDER_PATH)
    for pos, _sku in instance.pallets:
        assert pos.y != 0
        assert pos.y != 39


def test_big_order_pallet_per_sku_distribution():
    instance = parse_worklist(BIG_ORDER_PATH)
    counts: dict[int, int] = {}
    for _pos, sku in instance.pallets:
        counts[sku] = counts.get(sku, 0) + 1
    tally: dict[int, int] = {}
    for c in counts.values():
        tally[c] = tally.get(c, 0) + 1
    assert tally == {1: 5, 2: 50, 3: 45}


def test_build_world_from_big_order():
    instance = parse_worklist(BIG_ORDER_PATH)
    world = instance and build_world(instance)
    assert len(world.robots) == 5
    assert len(world.pallets) == 240
    assert len(world.orders) == 1000
    assert world.robots[0].position == instance.robot_starts[0]
    assert world.pallets[0].count == world.pallets[0].max_count


def test_malformed_missing_lines_raises_parse_error(tmp_path):
    bad_file = tmp_path / "bad.txt"
    bad_file.write_text("2\n0 0\n")  # claims 2 robots, only gives 1 position
    with pytest.raises(ParseError):
        parse_worklist(str(bad_file))


def test_malformed_non_integer_raises_parse_error(tmp_path):
    bad_file = tmp_path / "bad.txt"
    bad_file.write_text("1\n0 0\nnot_a_number\n")
    with pytest.raises(ParseError):
        parse_worklist(str(bad_file))


def test_pallet_out_of_range_sku_raises_parse_error(tmp_path):
    bad_file = tmp_path / "bad.txt"
    bad_file.write_text("1\n0 0\n2\n10\n10\n1\n0 1 5\n0\n")
    with pytest.raises(ParseError):
        parse_worklist(str(bad_file))


def test_trailing_content_raises_parse_error(tmp_path):
    bad_file = tmp_path / "bad.txt"
    bad_file.write_text("1\n0 0\n1\n10\n0\n0\nextra garbage line\n")
    with pytest.raises(ParseError):
        parse_worklist(str(bad_file))


def test_minimal_valid_file_round_trips(tmp_path):
    small = tmp_path / "small.txt"
    small.write_text("1\n2 3\n1\n5\n1\n4 4 0\n1\n0\n")
    instance = parse_worklist(str(small))
    assert instance.robot_starts == [Coord(2, 3)]
    assert instance.sku_capacities == [5]
    assert instance.pallets == [(Coord(4, 4), 0)]
    assert instance.orders == [[0]]

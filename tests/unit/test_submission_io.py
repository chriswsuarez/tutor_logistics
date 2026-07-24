import pytest

from warehouse.io.submission_parser import SubmissionParseError, parse_submission
from warehouse.io.submission_writer import SubmissionLog
from warehouse.model.action import Action


def test_writer_emits_expected_line_format(tmp_path):
    log = SubmissionLog()
    log.record_tick(0, {0: Action("move", 1, 0), 1: Action("pick", 10, 5)})
    log.record_tick(1, {0: Action("move", 2, 0)})
    out = tmp_path / "sol.txt"
    log.write(str(out))
    lines = out.read_text().splitlines()
    assert lines == [
        "0 0 move 1 0",
        "0 1 pick 10 5",
        "1 0 move 2 0",
    ]


def test_writer_sorts_robots_within_a_tick(tmp_path):
    log = SubmissionLog()
    log.record_tick(0, {5: Action("move", 1, 0), 1: Action("move", 2, 0)})
    out = tmp_path / "sol.txt"
    log.write(str(out))
    lines = out.read_text().splitlines()
    assert lines == ["0 1 move 2 0", "0 5 move 1 0"]


def test_writer_rejects_non_increasing_tick():
    log = SubmissionLog()
    log.record_tick(1, {0: Action("move", 1, 0)})
    with pytest.raises(ValueError):
        log.record_tick(1, {1: Action("move", 1, 0)})
    with pytest.raises(ValueError):
        log.record_tick(0, {1: Action("move", 1, 0)})


def test_writer_final_tick_tracks_last_recorded():
    log = SubmissionLog()
    assert log.final_tick == 0
    log.record_tick(7, {0: Action("move", 1, 0)})
    assert log.final_tick == 7


def test_parser_round_trips_writer_output(tmp_path):
    log = SubmissionLog()
    log.record_tick(0, {0: Action("move", 1, 0), 1: Action("pick", 10, 5)})
    log.record_tick(1, {0: Action("fulfill", 0, 0)})
    out = tmp_path / "sol.txt"
    log.write(str(out))

    entries = parse_submission(str(out))
    assert [(e.tick, e.robot_id, e.action.kind) for e in entries] == [
        (0, 0, "move"),
        (0, 1, "pick"),
        (1, 0, "fulfill"),
    ]


def test_parser_rejects_out_of_order_ticks(tmp_path):
    bad = tmp_path / "bad.txt"
    bad.write_text("1 0 move 1 0\n0 1 move 2 0\n")
    with pytest.raises(SubmissionParseError):
        parse_submission(str(bad))


def test_parser_rejects_duplicate_tick_robot_pair(tmp_path):
    bad = tmp_path / "bad.txt"
    bad.write_text("0 0 move 1 0\n0 0 move 2 0\n")
    with pytest.raises(SubmissionParseError):
        parse_submission(str(bad))


def test_parser_rejects_unknown_action_kind(tmp_path):
    bad = tmp_path / "bad.txt"
    bad.write_text("0 0 teleport 1 0\n")
    with pytest.raises(SubmissionParseError):
        parse_submission(str(bad))


def test_parser_rejects_wrong_field_count(tmp_path):
    bad = tmp_path / "bad.txt"
    bad.write_text("0 0 move 1\n")
    with pytest.raises(SubmissionParseError):
        parse_submission(str(bad))


def test_parser_allows_same_tick_different_robots(tmp_path):
    ok = tmp_path / "ok.txt"
    ok.write_text("0 0 move 1 0\n0 1 move 2 0\n")
    entries = parse_submission(str(ok))
    assert len(entries) == 2

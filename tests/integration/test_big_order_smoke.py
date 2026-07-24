import pytest

from warehouse.cli import validate as validate_cli
from warehouse.io.worklist_parser import build_world, parse_worklist
from warehouse.sim_driver import SimulationDriver

BIG_ORDER_PATH = "task/BIG_ORDER.txt"


@pytest.mark.slow
def test_big_order_solves_and_independently_validates_clean(tmp_path, capsys):
    """Runs the real solver against the actual Big Order input, then
    independently re-validates the emitted submission via a *fresh* parse
    through the validator CLI's own engine replay -- this is why the
    validator is a first-class module rather than a throwaway script: it's
    also the test oracle for the full-scale run."""
    instance = parse_worklist(BIG_ORDER_PATH)
    world = build_world(instance)

    driver = SimulationDriver(world)
    log = driver.run()

    assert world.all_orders_fulfilled()

    submission_path = tmp_path / "big_order_solution.txt"
    log.write(str(submission_path))

    exit_code = validate_cli.main([BIG_ORDER_PATH, str(submission_path)])
    output = capsys.readouterr().out
    assert exit_code == 0, output
    assert "valid: all 1000 orders fulfilled" in output

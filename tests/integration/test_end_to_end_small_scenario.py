from warehouse.cli import validate as validate_cli
from warehouse.io.worklist_parser import build_world, parse_worklist
from warehouse.sim_driver import SimulationDriver

FIXTURE_PATH = "tests/fixtures/small_scenario.txt"


def test_small_scenario_solves_and_validates_clean(tmp_path, capsys):
    instance = parse_worklist(FIXTURE_PATH)
    world = build_world(instance)

    driver = SimulationDriver(world)
    log = driver.run()

    assert world.all_orders_fulfilled()
    assert all(order.fulfilled for order in world.orders)  # includes both duplicate-signature orders

    submission_path = tmp_path / "solution.txt"
    log.write(str(submission_path))

    exit_code = validate_cli.main([FIXTURE_PATH, str(submission_path)])
    output = capsys.readouterr().out
    assert exit_code == 0, output
    assert "valid: all 4 orders fulfilled" in output


def test_small_scenario_forces_a_replenishment_round_trip():
    # sku 0 has a single capacity-2 pallet but two orders each need 2 units of
    # it (4 total demand) -- this can only complete via at least one dock ->
    # travel to y=39 -> auto-refill -> undock cycle.
    instance = parse_worklist(FIXTURE_PATH)
    world = build_world(instance)

    driver = SimulationDriver(world)
    driver.run()

    assert world.all_orders_fulfilled()
    sku0_pallet = next(p for p in world.pallets.values() if p.sku == 0)
    assert sku0_pallet.docked_to is None  # every replenish cycle ends by undocking

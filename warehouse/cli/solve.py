from __future__ import annotations

import argparse
import sys

from warehouse.io.worklist_parser import build_world, parse_worklist
from warehouse.sim_driver import IncompleteSolutionError, SimulationDriver


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Solve a warehouse worklist and write a submission file.")
    parser.add_argument("worklist_path")
    parser.add_argument("output_path")
    args = parser.parse_args(argv)

    instance = parse_worklist(args.worklist_path)
    world = build_world(instance)
    driver = SimulationDriver(world)

    try:
        log = driver.run()
    except IncompleteSolutionError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    log.write(args.output_path)
    print(f"solved: submission spans ticks 0..{log.final_tick} ({log.final_tick + 1} timesteps) -> {args.output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

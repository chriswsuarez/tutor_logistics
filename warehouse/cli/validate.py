from __future__ import annotations

import argparse
import sys
from collections import defaultdict

from warehouse.io.submission_parser import SubmissionParseError, parse_submission
from warehouse.io.worklist_parser import build_world, parse_worklist
from warehouse.sim.engine import apply_tick
from warehouse.sim.exceptions import InvalidActionError, RuleViolation


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Replay a submission file against a worklist and report violations.")
    parser.add_argument("worklist_path")
    parser.add_argument("submission_path")
    args = parser.parse_args(argv)

    instance = parse_worklist(args.worklist_path)
    world = build_world(instance)

    try:
        entries = parse_submission(args.submission_path)
    except SubmissionParseError as exc:
        print(f"error: malformed submission file: {exc}", file=sys.stderr)
        return 1

    by_tick: dict[int, dict] = defaultdict(dict)
    for entry in entries:
        by_tick[entry.tick][entry.robot_id] = entry.action

    last_tick = max(by_tick) if by_tick else -1
    for tick in range(last_tick + 1):
        actions = by_tick.get(tick, {})
        try:
            apply_tick(world, actions)
        except (InvalidActionError, RuleViolation) as exc:
            print(f"error at tick {tick}: {exc}", file=sys.stderr)
            return 1

    unfulfilled = sum(1 for order in world.orders if not order.fulfilled)
    if unfulfilled:
        print(f"invalid: {unfulfilled} of {len(world.orders)} orders remain unfulfilled after tick {last_tick}")
        return 1

    print(f"valid: all {len(world.orders)} orders fulfilled; submission spans ticks 0..{last_tick} ({last_tick + 1} timesteps)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

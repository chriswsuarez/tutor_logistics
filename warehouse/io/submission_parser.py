from __future__ import annotations

from dataclasses import dataclass

from warehouse.model.action import Action


class SubmissionParseError(ValueError):
    pass


@dataclass(frozen=True)
class SubmissionEntry:
    tick: int
    robot_id: int
    action: Action


def parse_submission(path: str) -> list[SubmissionEntry]:
    """Parse an arbitrary submission file (ours or hand-written) for the
    validator CLI, enforcing the format's own ordering rules rather than
    silently tolerating or reordering violations."""
    entries: list[SubmissionEntry] = []
    seen: set[tuple[int, int]] = set()
    last_tick = None

    with open(path) as f:
        for lineno, raw in enumerate(f, start=1):
            line = raw.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 5:
                raise SubmissionParseError(f"line {lineno}: expected 5 fields, got {line!r}")
            tick_s, robot_id_s, kind, x_s, y_s = parts
            try:
                tick, robot_id, x, y = int(tick_s), int(robot_id_s), int(x_s), int(y_s)
            except ValueError as exc:
                raise SubmissionParseError(f"line {lineno}: non-integer field in {line!r}") from exc

            if last_tick is not None and tick < last_tick:
                raise SubmissionParseError(f"line {lineno}: tick {tick} out of order (last was {last_tick})")
            last_tick = tick

            if (tick, robot_id) in seen:
                raise SubmissionParseError(f"line {lineno}: duplicate (timestep, robot_id) pair ({tick}, {robot_id})")
            seen.add((tick, robot_id))

            try:
                action = Action(kind=kind, x=x, y=y)
            except ValueError as exc:
                raise SubmissionParseError(f"line {lineno}: {exc}") from exc

            entries.append(SubmissionEntry(tick=tick, robot_id=robot_id, action=action))

    return entries

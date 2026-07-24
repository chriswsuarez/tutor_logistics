from __future__ import annotations

from typing import Optional

from warehouse.model.action import Action


class SubmissionLog:
    """Accumulates one tick's worth of actions at a time and writes them out in
    the `<timestep> <robot_id> <action> <x> <y>` submission format. Enforces
    strictly increasing tick order and, by construction (one dict entry per
    robot per call), no duplicate (timestep, robot_id) pairs."""

    def __init__(self) -> None:
        self._lines: list[tuple[int, int, str, int, int]] = []
        self._last_tick: Optional[int] = None

    def record_tick(self, tick: int, actions: dict[int, Action]) -> None:
        if self._last_tick is not None and tick <= self._last_tick:
            raise ValueError(f"tick {tick} is not strictly greater than last recorded tick {self._last_tick}")
        self._last_tick = tick
        for robot_id in sorted(actions):
            action = actions[robot_id]
            self._lines.append((tick, robot_id, action.kind, action.x, action.y))

    def write(self, path: str) -> None:
        with open(path, "w") as f:
            for tick, robot_id, kind, x, y in self._lines:
                f.write(f"{tick} {robot_id} {kind} {x} {y}\n")

    @property
    def final_tick(self) -> int:
        return self._last_tick if self._last_tick is not None else 0

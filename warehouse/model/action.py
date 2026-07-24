from __future__ import annotations

from dataclasses import dataclass

MOVE = "move"
PICK = "pick"
DOCK = "dock"
UNDOCK = "undock"
FULFILL = "fulfill"

ACTION_KINDS = (MOVE, PICK, DOCK, UNDOCK, FULFILL)


@dataclass(frozen=True)
class Action:
    """One robot's single action for a single timestep, in submission-line shape."""

    kind: str
    x: int
    y: int

    def __post_init__(self) -> None:
        if self.kind not in ACTION_KINDS:
            raise ValueError(f"unknown action kind {self.kind!r}")

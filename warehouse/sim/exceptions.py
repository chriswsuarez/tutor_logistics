class RuleViolation(Exception):
    """Raised when a batch of proposed actions would violate the simulation
    rules. Under normal operation the driver/task layer should never construct
    such a batch (see warehouse/sim/config.py and sim_driver.py for how
    over-commit is avoided by construction) — this firing signals a bug in our
    own scheduling, not a legitimately-rejected player move.

    `robot_ids` names every robot implicated in the conflict, letting a caller
    (e.g. SimulationDriver) drop just those actions and force a replan for
    just those robots rather than treating the whole tick as unrecoverable.
    """

    def __init__(self, message: str, robot_ids: tuple[int, ...] = ()) -> None:
        super().__init__(message)
        self.robot_ids = robot_ids


class InvalidActionError(Exception):
    """Raised when a single proposed action is structurally invalid against
    the current world state (e.g. moving to a non-adjacent cell, picking from
    a non-adjacent pallet).

    `robot_id` names the robot whose action was invalid, letting a caller drop
    just that action and force a replan for just that robot rather than
    treating the whole tick as unrecoverable.
    """

    def __init__(self, message: str, robot_id: int | None = None) -> None:
        super().__init__(message)
        self.robot_id = robot_id

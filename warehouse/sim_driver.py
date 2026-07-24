from __future__ import annotations

from collections import Counter
from typing import Optional

from warehouse.io.submission_writer import SubmissionLog
from warehouse.model.action import Action
from warehouse.model.world import WorldState
from warehouse.planning.reservation import ReservationTable
from warehouse.sim.config import SimConfig
from warehouse.sim.engine import apply_tick
from warehouse.sim.exceptions import InvalidActionError, RuleViolation
from warehouse.tasks.order_selector import NearestOrderSelector
from warehouse.tasks.pallet_selector import NearestAvailablePallet
from warehouse.tasks.robot_controller import RobotController
from warehouse.tasks.task_manager import TaskManager

_STALL_TICKS_BEFORE_GLOBAL_REPLAN = 100


class IncompleteSolutionError(Exception):
    """Raised if `max_ticks` elapses with orders still unfulfilled -- a real
    bug (a structural deadlock our fallback couldn't resolve, or a genuinely
    infeasible instance), not something callers should expect to hit."""


class SimulationDriver:
    """Ties the reservation table, task manager, robot controllers, and
    simulation engine together tick by tick, producing a SubmissionLog.

    Each tick: (a) idle robots get a new order; (b) the shared reservation
    table's static holds are resynced from the current world state; (c) every
    controller proposes at most one action, in a fixed ascending-robot-id
    order; (d) the engine applies the whole batch atomically; (e) the tick's
    actions are logged; (f) elapsed reservations are pruned. A simple
    execution-time watchdog triggers a global replan if any robot makes no
    progress for too many consecutive ticks (a secondary safety net -- the
    primary deadlock defense is each robot's own retry-with-larger-budget in
    RobotController._advance_along_path_to).
    """

    def __init__(self, world: WorldState, config: Optional[SimConfig] = None):
        self.world = world
        self.config = config or SimConfig()
        self.reservation_table = ReservationTable()
        pallet_selector = NearestAvailablePallet()
        controllers = {rid: RobotController(rid, pallet_selector) for rid in world.robots}
        self.task_manager = TaskManager(controllers, NearestOrderSelector(pallet_selector), pallet_selector)
        self._stall_counts = {rid: 0 for rid in world.robots}

    def run(self) -> SubmissionLog:
        log = SubmissionLog()
        while not self.world.all_orders_fulfilled():
            if self.world.tick >= self.config.max_ticks:
                raise IncompleteSolutionError(
                    f"reached max_ticks={self.config.max_ticks} with orders still unfulfilled"
                )
            self._run_one_tick(log)
        return log

    def _run_one_tick(self, log: SubmissionLog) -> None:
        self.task_manager.assign_idle_robots(self.world)

        planned_ids = {
            rid for rid, controller in self.task_manager.controllers.items() if controller.pending_path is not None
        }
        self.reservation_table.sync_static_holds(self.world, planned_ids)

        claimed_picks: Counter = Counter()
        claimed_docks: set[int] = set()
        actions = {}
        for robot_id in sorted(self.task_manager.controllers):
            controller = self.task_manager.controllers[robot_id]
            action = controller.propose_action(self.world, self.reservation_table, claimed_picks, claimed_docks)
            if action is not None:
                actions[robot_id] = action
                self._stall_counts[robot_id] = 0
            elif controller.is_idle():
                self._stall_counts[robot_id] = 0
            else:
                self._stall_counts[robot_id] += 1

        tick = self.world.tick
        self._apply_tick_with_recovery(actions)
        if actions:
            log.record_tick(tick, actions)
        self.reservation_table.prune_before(self.world.tick)

        if self._stall_counts and max(self._stall_counts.values()) >= _STALL_TICKS_BEFORE_GLOBAL_REPLAN:
            self._global_replan_fallback()

    def _apply_tick_with_recovery(self, actions: dict[int, Action]) -> None:
        """Apply this tick's actions, defensively: correct upstream scheduling
        (the universal reservation table, the per-tick pick-claim counter)
        should mean the engine never rejects anything, but a robot must never
        be permanently stuck because of a single stale/invalid action. If the
        engine does reject one, drop just the offending robot(s)' action(s),
        force those robots to replan from scratch next tick, and retry with
        the rest. `apply_tick` fully validates before mutating anything, so a
        raised exception means no side effects occurred yet -- retrying with a
        smaller action set from the same world state is always safe, and each
        retry strictly shrinks `actions`, so this always terminates."""
        while True:
            try:
                apply_tick(self.world, actions, self.config)
                return
            except InvalidActionError as exc:
                if exc.robot_id is None or exc.robot_id not in actions:
                    raise
                del actions[exc.robot_id]
                self._force_replan(exc.robot_id)
            except RuleViolation as exc:
                offenders = [rid for rid in exc.robot_ids if rid in actions]
                if not offenders:
                    raise
                for robot_id in offenders:
                    del actions[robot_id]
                    self._force_replan(robot_id)

    def _force_replan(self, robot_id: int) -> None:
        self.reservation_table.release_robot(robot_id)
        self.task_manager.controllers[robot_id].reset_path()
        self._stall_counts[robot_id] = 0

    def _global_replan_fallback(self) -> None:
        for robot_id in self.task_manager.controllers:
            self._force_replan(robot_id)

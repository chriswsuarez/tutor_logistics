from __future__ import annotations

from collections import Counter, defaultdict, deque

from warehouse.model.grid import Coord
from warehouse.model.world import WorldState
from warehouse.tasks.robot_controller import RobotController
from warehouse.tasks.task_types import RelocateSubGoal

# Generous margin: with only 5 robots, a blocked pallet may need to wait for a
# multi-hop chain of blockers to clear sequentially (empirically up to 3 deep on
# the real Big Order), each taking up to ~100 ticks, so this only fires on a
# genuine cyclic target dependency, not ordinary chain-waiting.
_STALL_TICKS_BEFORE_ERROR = 400

# Same generous margin, but measuring a different failure mode: a robot that IS
# assigned and DOES keep emitting actions (e.g. repeated congestion sidesteps)
# without its subgoal ever actually completing -- a real bug found empirically
# (an unreachable landing cell from a flawed layout) that the driver's generic
# per-tick stall counters never caught, since a periodic sidestep resets them
# without the underlying goal ever becoming reachable. This checks direct
# subgoal progress instead.
_MAX_TICKS_PER_RELOCATION = 500

# Cap on simultaneously in-flight relocations. The shared reservation table
# requires a robot's destination footprint to be free not just on arrival but
# for every tick anyone else has already committed a path through (see
# ReservationTable.is_free_indefinitely) -- fine in the sparse, low-contention
# traffic ordinary order fulfillment sees, but found empirically to make A*
# searches (and their expensive max_expansions retries) pathologically
# expensive when many robots are simultaneously threading paths through the
# same small set of shared checkerboard corridor cells. Throttling concurrency
# keeps route overlap rare enough for that same search to stay cheap, at the
# cost of some upfront parallelism (still cheap overall: relocation is a
# one-time phase against an 80k+-tick budget).
_MAX_CONCURRENT_RELOCATIONS = 2


def _manhattan(a: Coord, b: Coord) -> int:
    return abs(a.x - b.x) + abs(a.y - b.y)


def sku_demand_rank(world: WorldState) -> list[int]:
    """SKUs ranked by distinct-order-visit-count (how many separate orders
    need at least one unit), descending, ties broken by sku id for
    determinism. Visit count, not total units, is what determines how many
    separate *trips* a pallet's resting position affects -- an order needing
    50 units of a SKU in one visit costs the same single trip as one needing
    just 1."""
    visit_counts: Counter[int] = Counter()
    for order in world.orders:
        for sku, qty in order.requirements.items():
            if qty > 0:
                visit_counts[sku] += 1
    all_skus = {p.sku for p in world.pallets.values()}
    return sorted(all_skus, key=lambda sku: (-visit_counts[sku], sku))


def plan_relocations(world: WorldState) -> dict[int, Coord]:
    """pallet_id -> permanent target Coord near y=0 for every pallet, ranked so
    the most order-visited SKUs land closest to the fulfillment row. Existing
    nearest-neighbor tour routing (TaskManager.decompose) is already close to
    the achievable rate for the *current* scattered layout (tour edge length
    ~ sqrt(area/N) for N required SKUs spread over the warehouse's usable
    area); the remaining lever is shrinking that area, not the routing
    algorithm over it.

    Pallets only ever rest at (odd x, odd y) cells -- a full 2D checkerboard,
    not just corridor *columns*. This is load-bearing, not decorative: a
    robot completing a drag must stand on whichever of the target's 4
    neighbors its dock offset (fixed back at the pallet's *original*
    position, from an arbitrary approach direction) happens to require. A
    column-only corridor scheme (every 3rd column clear, rest packed) makes
    that landing cell safe horizontally but not vertically -- two vertically
    adjacent slot rows sharing the same slot column are *each other's*
    landing cells, so a north/south approach can require settling into a
    different pallet's own permanent target: a real deadlock this exact bug
    produced when first tried (empirically: a robot got permanently stuck
    mid-drag, its landing cell occupied by pallet 82's own target, and the
    generic congestion-sidestep fallback masked it indefinitely instead of
    surfacing it, since it kept emitting *an* action without ever finishing).
    Checkerboarding fixes this structurally: every neighbor of an (odd, odd)
    cell has exactly one odd coordinate flip to even, landing on a corridor
    row or column -- never another target -- regardless of which side the
    robot happens to approach from, so no direction needs to be forced.

    Slot columns exclude the grid's rightmost column when it's odd (dragging
    with dock offset WEST requires the robot to stand at target.x + 1, out of
    bounds there; no mirror problem at the left edge since x=0 is always even
    already never a slot column).
    """
    grid = world.grid
    slot_columns = [x for x in range(1, grid.width, 2) if x != grid.width - 1]
    num_pallets = len(world.pallets)
    rows_needed = -(-num_pallets // len(slot_columns))  # ceil div
    pallet_rows = [1 + 2 * i for i in range(rows_needed)]
    assert pallet_rows[-1] <= grid.height - 3, "relocation band would collide with the replenishment row"
    slots = [Coord(x, y) for y in pallet_rows for x in slot_columns]

    pallets_by_sku: dict[int, list[int]] = defaultdict(list)
    for pallet in world.pallets.values():
        pallets_by_sku[pallet.sku].append(pallet.id)
    for ids in pallets_by_sku.values():
        ids.sort()
    ordered_pallet_ids = [pid for sku in sku_demand_rank(world) for pid in pallets_by_sku[sku]]

    # Greedy nearest-slot assignment (in rank order, so higher-demand SKUs get
    # first pick), not a rigid row-major zip: a pure rank-order fill can send a
    # pallet clear across the grid to whatever slot the fill sequence happens
    # to reach next, forcing a long drag with a *fixed* dock offset (side is
    # locked in at dock time, before the target is ever approached). Found
    # empirically on the real Big Order that a long enough drag can leave the
    # robot's rigid 2-cell footprint with no valid route at all through the
    # combination of still-scattered original pallets and partially-filled
    # slots -- not merely a slow search, confirmed genuinely unreachable even
    # at 2,000,000 A* expansions, though the *destination* cell itself was
    # free. Minimizing each pallet's own drag distance keeps trips short
    # enough that this kind of rigid-body dead end becomes far less likely,
    # while still landing every pallet in the same compact near-y=0 band
    # (exactly which slot within the band matters far less than getting
    # there at all).
    remaining_slots = set(slots)
    targets: dict[int, Coord] = {}
    for pallet_id in ordered_pallet_ids:
        position = world.pallets[pallet_id].position
        nearest = min(remaining_slots, key=lambda s: (_manhattan(position, s), s.y, s.x))
        remaining_slots.discard(nearest)
        targets[pallet_id] = nearest
    return targets


class RelocationCoordinator:
    """Drives the one-time upfront pallet-relocation phase, structurally
    parallel to TaskManager (same assign_idle_robots(world) shape) so
    SimulationDriver can share the exact same per-tick machinery
    (sync_static_holds, claimed_picks/claimed_docks, batch apply, stall
    watchdog) for both phases -- only which assigner populates idle robots'
    subgoal queues differs.

    Single flat backlog, assigned purely by nearest-pallet-first (like
    NearestOrderSelector elsewhere): which row a pallet's target sits in
    (fixed once by plan_relocations, based on demand rank) doesn't need to
    constrain *execution order* at all, since every pallet gets relocated
    exactly once regardless of the order robots get to them. Two stricter
    schemes were tried and rejected empirically: a hard per-row barrier
    deadlock-adjacent (an original position can land on a *later*,
    not-yet-opened row's target, leaving its blocker ineligible for
    assignment until the whole current row finishes -- stalling far longer
    than any real cyclic dependency would); and even a soft "prefer the
    closest row" tiebreak (without a barrier) caused all 5 robots to
    converge on the same handful of rows simultaneously, saturating the
    shared corridor cells serving them and starving robots whose landing
    cell needed one of those same corridor cells to go indefinitely free.
    Pure nearest-first naturally spreads robots across the whole grid
    instead.

    Only offers a robot a backlog pallet whose target cell is CURRENTLY free
    (world.entity_at(target) is None) -- this one condition fully and
    automatically resolves any chain-shaped dependency (pallet A's target
    occupied by not-yet-moved pallet B) via opportunistic reordering, with
    zero bookkeeping: A simply waits in the backlog until whichever robot
    relocates B clears the cell. A genuine cycle (A blocks B blocks A) would
    stall forever under this scheme since neither side's target ever frees
    -- verified empirically absent for the real Big Order input (30
    original-position/target collisions, longest dependency chain 3, zero
    cycles), so rather than build a staging-cell cycle-breaker for a case
    confirmed not to occur, a sustained stall raises a loud RuntimeError
    instead of silently hanging until max_ticks.
    """

    def __init__(self, controllers: dict[int, RobotController], targets: dict[int, Coord], world: WorldState):
        self.controllers = controllers
        self.targets = targets
        # A pallet whose original position already happens to equal its own
        # computed target needs no relocation -- and must never enter the
        # backlog, or the "is target free" check below sees its own presence
        # there and reads it as permanently blocked by a not-yet-moved
        # sibling (since nothing will ever relocate a pallet with nowhere to
        # go), stalling forever. Found empirically: 4 pallets on the real Big
        # Order landed exactly on their own target and stalled the whole
        # phase until excluded here.
        self._backlog: set[int] = {
            pid for pid, target in targets.items() if world.pallets[pid].position != target
        }
        self._stall_ticks = 0
        self._assigned_at: dict[int, int] = {}

    def is_done(self) -> bool:
        return not self._backlog and all(c.is_idle() for c in self.controllers.values())

    def _blocking_pallet_ids(self, world: WorldState) -> set[int]:
        """Backlog pallet ids currently sitting on some already-docked robot's
        landing cell -- these must be cleared before that robot can ever
        finish, so they need top assignment priority regardless of their own
        target's row. Without this, a pallet with-mediocre row-priority
        sitting on a busy robot's landing spot can be passed over
        indefinitely by other idle robots choosing closer/higher-priority
        backlog items, permanently starving the blocked robot even though
        nothing is structurally deadlocked -- found empirically on the real
        Big Order (one robot stuck 500+ ticks while its blocker sat
        unclaimed in the backlog, untouched, the whole time)."""
        blockers = set()
        for controller in self.controllers.values():
            if controller.is_idle() or not controller.subgoals:
                continue
            subgoal = controller.subgoals[0]
            if not isinstance(subgoal, RelocateSubGoal):
                continue
            robot = world.robots[controller.robot_id]
            pallet = world.pallets[subgoal.pallet_id]
            if pallet.docked_to != controller.robot_id:
                continue  # still approaching/not docked yet -- no landing cell committed
            offset = next(off for off, pid in robot.docked_pallets.items() if pid == subgoal.pallet_id)
            landing = Coord(subgoal.target.x - offset.x, subgoal.target.y - offset.y)
            occupant = world.entity_at(landing)
            if occupant is not None and occupant[0] == "pallet" and occupant[1] in self._backlog:
                blockers.add(occupant[1])
        return blockers

    def assign_idle_robots(self, world: WorldState) -> None:
        urgent = self._blocking_pallet_ids(world)
        active = sum(1 for c in self.controllers.values() if not c.is_idle())

        any_idle_wanting_work = False
        any_assigned = False
        for robot_id in sorted(self.controllers):
            controller = self.controllers[robot_id]
            if not controller.is_idle() or not self._backlog:
                continue
            if active >= _MAX_CONCURRENT_RELOCATIONS:
                continue
            any_idle_wanting_work = True
            position = world.robots[robot_id].position
            available = [pid for pid in self._backlog if world.entity_at(self.targets[pid]) is None]
            if not available:
                continue  # every remaining pallet is blocked by an unmoved sibling; wait
            pallet_id = min(
                available,
                key=lambda pid: (pid not in urgent, _manhattan(position, world.pallets[pid].position), pid),
            )
            self._backlog.discard(pallet_id)
            self._assigned_at[pallet_id] = world.tick
            controller.assign(deque([RelocateSubGoal(pallet_id=pallet_id, target=self.targets[pallet_id])]))
            any_assigned = True
            active += 1

        for controller in self.controllers.values():
            if controller.is_idle() or not controller.subgoals:
                continue
            subgoal = controller.subgoals[0]
            if not isinstance(subgoal, RelocateSubGoal):
                continue
            started = self._assigned_at.get(subgoal.pallet_id)
            if started is not None and world.tick - started > _MAX_TICKS_PER_RELOCATION:
                raise RuntimeError(
                    f"relocation stuck: pallet {subgoal.pallet_id} has been mid-drag for over "
                    f"{_MAX_TICKS_PER_RELOCATION} ticks without completing -- likely an "
                    "unreachable landing cell; investigate the layout"
                )

        if any_idle_wanting_work and not any_assigned:
            self._stall_ticks += 1
            if self._stall_ticks >= _STALL_TICKS_BEFORE_ERROR:
                raise RuntimeError(
                    "relocation deadlock: idle robots have work waiting but every remaining "
                    "backlog pallet has a target cell blocked by another not-yet-moved pallet "
                    f"(a cyclic dependency) for {_STALL_TICKS_BEFORE_ERROR} consecutive ticks -- "
                    "this wasn't expected to occur for the real Big Order input; investigate the layout"
                )
        else:
            self._stall_ticks = 0

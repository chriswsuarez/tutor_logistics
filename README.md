# The Big Order — Warehouse Robot Fleet Solver

A Python solver for the ["Big Order" warehouse routing challenge](task/task.md) from Tutor
Intelligence: command a fleet of 5 robots around a 60×40 grid warehouse to fulfill 1,000 orders as
fast as possible, picking items from finite-stock pallets and delivering exact-match baskets to a
fulfillment row, replenishing depleted pallets along the way. The score is the total number of
timesteps until every order is fulfilled — lower is better.

This repo contains the puzzle input, the solver, a from-scratch simulation engine that replays and
validates any submission file, and the test suite used to check both.

**Current result on the real puzzle input:** 62,448 timesteps (down from an initial naive baseline
of ~180,000). See [Design](#design) for how, and [Possible Improvements](#possible-improvements)
for what was tried beyond that and didn't pan out.

## Requirements

- Python 3.11+ (developed against 3.14)
- No third-party packages at runtime — the solver itself is pure standard library
- `pytest` and `ruff` as dev-only dependencies, for running the test suite and linting

## Setup

```bash
python -m venv .venv
.venv/bin/pip install -e ".[dev]"
```

## Running a test on `BIG_ORDER.txt`

Solve the real puzzle input and write a submission file:

```bash
.venv/bin/python -m warehouse.cli.solve task/BIG_ORDER.txt out/solution.txt
```

This prints the resulting score (total timesteps) and takes a little over two minutes wall-clock
on a modern laptop. Then independently validate the submission — this replays it from a fresh
parse of the input against the same rules engine used as the test suite's oracle, and reports any
rule violations or confirms success:

```bash
.venv/bin/python -m warehouse.cli.validate task/BIG_ORDER.txt out/solution.txt
```

A clean run prints something like:

```
valid: all 1000 orders fulfilled; submission spans ticks 0..62447 (62448 timesteps)
```

The submission file itself is a plain text log, one line per action:

```
<timestep> <robot_id> <action> <x> <y>
```

which you can also drop directly into the project's browser-based Testbench to visualize the run.

### Running the test suite

```bash
.venv/bin/pytest              # fast unit + integration tests (a few hundred ms)
.venv/bin/pytest -m slow      # also runs the full Big Order end-to-end (~2 minutes)
.venv/bin/ruff check .        # lint
```

The `slow` test is the same solve-then-validate flow as above, wired up as an automated regression
test — it's the closest thing this repo has to a correctness oracle, since there's no local access
to the real grader.

### Solving a different worklist

The solver isn't hardcoded to the Big Order — it reads any file in the same
[Worklist format](task/task.md#worklist-format-big_ordertxt):

```bash
.venv/bin/python -m warehouse.cli.solve <your-worklist.txt> <output.txt>
```

## Design

### The core loop

Every timestep, the simulator (`warehouse/sim_driver.py`) does the same four things: hand any idle
robot a new task, let every robot's controller propose one action each, apply the whole batch
atomically against the rules engine, and log it. The rules engine
(`warehouse/sim/engine.py::apply_tick`) is a faithful, from-scratch reimplementation of the puzzle's
mechanics — it validates a tick's proposed actions against a single frozen start-of-tick snapshot
(a deliberately conservative reading of the spec's silence on simultaneous-move semantics), then
applies moves, picks, docks, undocks, fulfillments, and automatic replenishment in that order. This
same engine is what `validate.py` replays independently, so "the solver's own idea of what
happened" and "an independent check of what happened" are never the same code path pretending
twice.

### Cooperative pathfinding

Robots don't just path toward a goal and hope for the best — a single shared `ReservationTable`
(`warehouse/planning/reservation.py`) tracks, for the whole fleet, everything currently at rest,
every robot's entire committed future path (written in full the moment it's planned, not just its
next step), and each plan's destination held indefinitely from the moment it's committed. A robot
plans via prioritized space-time A* (`warehouse/planning/cooperative_astar.py`) against this shared
table, so two robots can never end up planning to occupy the same cell at the same time, even if one
plan was committed long before the other.

### Where the time actually goes

Profiling the very first working version (~80,600 timesteps) showed that ~97% of all actions were
plain `move` — picking, delivering, and replenishing are comparatively cheap and mostly fixed
regardless of layout. Almost all of that movement was ordinary travel between a robot's current
position and whichever pallet it needed next, and pallets sat wherever the puzzle input happened to
scatter them, with `y` roughly uniform across the whole 40-row warehouse. The existing route
construction inside an order (a greedy nearest-neighbor tour over that order's needed SKUs) was
already close to the best achievable *for that layout* — the lever wasn't a smarter routing
algorithm, it was changing where pallets physically sit.

### Pallet relocation — the biggest single win

Nothing in the puzzle rules says a pallet has to stay where it started. `warehouse/tasks/relocation.py`
adds a one-time phase, run before any order fulfillment begins, that permanently drags every one of
the 240 pallets to a slot in a compact band near the fulfillment row — ranked so the SKUs needed by
the most distinct orders end up closest. Since a high-demand SKU is picked from far more often than
it's ever replenished, this trades a small, fixed, one-time relocation cost for a large, recurring
reduction in the travel every subsequent order has to do.

Pallets rest only at `(odd x, odd y)` cells — a full checkerboard, not just alternating columns or
rows — so that every neighbor of any target is guaranteed to be a permanently clear corridor cell,
regardless of which side a robot happens to approach from. This property turned out to be load
bearing, not decorative: see [Possible Improvements](#possible-improvements) for what happened when
denser, direction-forcing layouts were tried instead.

### Order and pallet selection

Once relocation is in place, nearly every order shares one of a handful of near-universal SKUs
parked right next to the fulfillment row (one SKU alone appears in 995 of the 1,000 real orders), so
picking "whichever unclaimed order has its *closest* required item nearest to me" stops
discriminating between orders — almost every order looks equally close. `NearestOrderSelector`
instead measures the **farthest** required SKU: since a robot's tour out and back is roughly
star-shaped from a near-fixed hub, its length is dominated by the trip to whichever item is
*farthest* away, not the nearest.

Replenishment (`ReplenishSubGoal` in `warehouse/tasks/robot_controller.py`) only ever docks a pallet
from its south side, forcing every replenishment drag to travel vertically. This isn't an arbitrary
restriction — a horizontally-docked pallet trails sideways through a densely-packed pallet column
for the length of the whole round trip to the replenishment row and back, which was found
empirically to produce routes that were genuinely unreachable (not just slow), stalling a robot for
hundreds of ticks.

## Possible Improvements

The score came down in three validated, shipped steps — pallet relocation (~80,600 → ~66,000),
order-selection tuning (~66,000 → ~62,448), and a replenishment-docking fix that was really a
correctness fix as much as a speedup. Past that point, several further ideas were tried and
**reverted** after they turned out to have real correctness problems on the full-scale input, not
just marginal payoffs. They're recorded here because the reasoning is genuinely useful for whoever
picks this up next, not just trivia:

- **A denser relocation band.** The checkerboard layout is only 25% dense (both rows and columns
  alternate). Tried skipping only rows *or* only columns instead of both, with the freed-up
  direction forced to a single safe approach side (mirroring the replenishment fix above). This
  broke in three different, non-obvious ways in sequence — a landing-cell collision, a long drag
  becoming genuinely unreachable through a densely-packed column, and finally a **deadlock baked
  into the input itself** (four real pallets in the puzzle data happen to sit in a vertical stack,
  which a vertical-only approach constraint turns into a mutual lockout no priority heuristic can
  resolve). Reverted to the full checkerboard, whose real property — *no* direction ever needs to be
  forced — turned out to be exactly what made it robust against this whole class of problem.
- **Co-occurrence-aware pallet slotting.** The idea: cluster SKUs that frequently appear in the same
  order near each other spatially, not just by raw demand rank. Checked directly against the real
  order data first — the actual co-occurrence signal (lift, i.e. actual vs. expected-if-independent)
  only ranges 0.70–1.46, meaning orders behave close to independent random draws weighted by
  popularity. There isn't much real "bundle" structure here to exploit, and integrating it would
  mean overriding the nearest-own-position placement rule that keeps relocation drags short — not
  worth the risk for the likely payoff.
- **2-opt route refinement.** A textbook local-search pass on top of the existing greedy
  nearest-neighbor tour, shortening individual order routes by ~9% in isolated testing. On the full
  real input, it caused two robots to **livelock** near the end of the run — a perpetual 4-tick
  position oscillation where each robot's completed step was immediately undone by the other's
  shifting reservations on replan. Every tick proposed a *valid*-looking move, so none of the
  existing stall-detection watchdogs (which only notice "no action was proposed") ever caught it.
  Reverted rather than build and validate a new "net progress over time" watchdog under time
  pressure.

That last point is itself the clearest remaining opportunity: **a genuine liveness watchdog** that
tracks a robot's net displacement over a sliding window of ticks, not just whether it proposed *an*
action, would catch livelocks that the current stuck-tick/global-replan mechanisms structurally
can't see. Building that first, before attempting another routing optimization, would make the next
attempt at 2-opt (or anything else that reorders when/where robots move) much safer to validate.

Beyond that, reasonable next directions include: a smarter order-selection cost function than
"farthest required SKU" (e.g. an actual estimated tour cost for a bounded candidate set of orders,
rather than a single-point proxy); explicit load-balancing across the 5 robots so the run's makespan
isn't determined by whichever robot happens to draw the least convenient remaining order near the
end; and reducing the one-time relocation phase's own cost further, though it's already a small
fraction (~3%) of total movement.

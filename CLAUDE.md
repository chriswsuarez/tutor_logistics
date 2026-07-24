# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository state

This repository holds a working Python solver for a warehouse robot-fleet routing challenge ("The
Big Order" from Tutor Intelligence):

- [task/task.md](task/task.md) — full problem spec (rules, formats, scoring)
- [task/BIG_ORDER.txt](task/BIG_ORDER.txt) — the actual puzzle input (1348 lines): 5 robots, 100
  SKUs, 240 pallets, 1000 orders, in the Worklist Format described below
- [warehouse/](warehouse/) — the solver package (see Architecture below)
- [tests/](tests/) — pytest suite (unit + integration + a slow full-scale smoke test)

The solver reads a Worklist file (any file matching the format below, not just the Big Order) and
emits a Submission file. It currently solves the real Big Order in ~66.0k timesteps (score) — see
`out/solution_relocated.txt` (current best, produced by the pallet-relocation phase described below)
vs. the earlier `out/solution_80603.txt` (pre-relocation) and `out/solution_180387.txt` (before task
assignment was tuned). Order/pallet selection during normal fulfillment is still fairly naive
(nearest-pallet, greedy nearest-neighbor SKU routing per order); further optimizing the score
(e.g. an order-selection cost proxy that accounts for non-hub items, or co-occurrence-aware
slotting) is ongoing work — see `warehouse/tasks/relocation.py`'s module docstring for context.

### Commands

The runtime itself is stdlib-only; `pytest`/`ruff` are dev deps. No virtualenv is checked in — set
one up first with `python -m venv .venv && .venv/bin/pip install -e ".[dev]"`.

- `.venv/bin/pytest` — fast unit + integration tests (excludes the slow Big Order run by default,
  configured via `addopts` in `pyproject.toml`)
- `.venv/bin/pytest -m slow` — also runs the full Big Order end-to-end (~2 minutes)
- `.venv/bin/ruff check .` — lint
- `.venv/bin/python -m warehouse.cli.solve <worklist.txt> <output.txt>` — solve a worklist, write a
  submission file, print the resulting score (total timesteps)
- `.venv/bin/python -m warehouse.cli.validate <worklist.txt> <submission.txt>` — replay an
  arbitrary submission file against the rules from a fresh parse and report violations/success;
  this is the same engine code path used as the test oracle for the full-scale smoke test, since
  there's no local access to the real grader/Testbench

### Architecture

- `warehouse/model/` — `Grid`/`Coord`, `Robot`/`Pallet`/`Order` dataclasses, `WorldState` (the
  single `occupancy` map both the simulator and the pathfinder read)
- `warehouse/io/` — Worklist parser/`WorldState` builder, submission writer/parser
- `warehouse/sim/` — `engine.apply_tick(world, actions, config)`: the reference rules engine.
  Validates a whole tick's proposed actions against a single frozen start-of-tick snapshot before
  mutating anything (a deliberately conservative reading of the spec's silence on simultaneous-move
  semantics — see `SimConfig`/`engine.py` docstrings), then applies moves → picks → docks →
  undocks → fulfills → automatic replenishment, in that order
- `warehouse/planning/` — cooperative multi-robot pathfinding, decoupled from `sim`/`tasks`:
  - `ReservationTable` is the **one shared spatio-temporal costmap** every robot's planner reads
    from and writes to. Beyond simple occupancy, it tracks three states: `static_holds` (anything
    currently at rest, resynced from real world state every tick), timed `vertex`/`edge`
    reservations (a robot's entire committed future path, written in full the instant it's
    planned — this is what guarantees two robots can never end up planning to occupy the same cell
    at the same timestep, even when one plan was committed long before the other), and
    `settled_holds` (a plan's destination cell, held indefinitely and eagerly from the moment the
    plan is committed — not just at its arrival tick — since a robot settling in for an open-ended
    stay, e.g. picking many times, doesn't reserve any ticks past arrival otherwise, leaving a gap
    a far-earlier-committed plan could collide with once its reserved tick comes due)
  - `cooperative_astar.plan(...)` — prioritized space-time A*; a candidate goal cell is only
    accepted once confirmed free *indefinitely* (via `ReservationTable.is_free_indefinitely`), not
    just at the instant of arrival, given the open-ended-stay reasoning above. Goals with a small
    enumerable candidate set (`adjacent_to`, `any_row`, `exact_cell`) carry that set so a
    currently-hopeless goal (every candidate permanently blocked) can be detected in O(candidates)
    instead of exhausting the search budget every tick.
- `warehouse/tasks/` — `RobotController` (per-robot FSM; re-derives its phase from ground-truth
  world state each tick rather than trusting a separately-tracked flag) and `TaskManager` (assigns
  unfulfilled orders to idle robots via a swappable `OrderSelectionPolicy`/`PalletSelectionPolicy`).
  Replenishment is a `ReplenishSubGoal`: dock (never from the pallet's north side — that would put
  the pallet on the robot's south side, unable to reach the replenishment row without going out of
  bounds) → travel to `y=height-1` (auto-refills at tick end) → **drag the pallet back to where it
  came from** before undocking (leaving it parked on the replenishment row would, over many trips,
  accumulate into a wall of pallets that can wedge others — or itself, next time — permanently out
  of reach) → resume collecting. `nearest_pallet_of_sku`'s `exclude_docked` skips any pallet already
  docked to another robot when picking a replenish target: without it, a second robot independently
  discovering the same empty SKU can capture that pallet's *current mid-transit* coordinate as its
  own `ReplenishSubGoal.origin`, then try to permanently drag it back to a meaningless waypoint deep
  in traffic instead of its true resting spot — found empirically to burn full-budget A* searches
  every tick for hundreds of ticks.
- `warehouse/tasks/relocation.py` — a one-time upfront phase (`RelocationCoordinator`), run before
  normal order fulfillment, that permanently drags every pallet to a demand-ranked slot near `y=0`
  (`plan_relocations`; SKUs ranked by distinct-order-visit-count, most-visited closest to the
  fulfillment row). This is the single biggest lever found so far: profiling showed ~97% of all
  actions in a solve are `move`, and per-leg travel distance is dominated by how far scattered
  pallets sit from the delivery row, not by routing quality (the existing greedy nearest-neighbor
  tour in `TaskManager.decompose` is already close to the achievable rate for a given layout).
  Pallets rest only at `(odd x, odd y)` cells — a full checkerboard, not just corridor *columns* —
  so every neighbor of a target is structurally guaranteed a corridor cell regardless of which side
  a drag approaches from; a column-only scheme was tried and found to deadlock (a robot's landing
  cell could land on a *different* pallet's own permanent target). Assignment is greedy
  nearest-slot-first in demand-rank order, not a rigid row-major fill, because a long drag with a
  *fixed* dock offset can leave a robot's rigid 2-cell footprint with no valid route at all through
  the combination of still-scattered originals and partially-filled slots — confirmed empirically
  (genuinely unreachable even at 2,000,000 A* expansions, not just slow). See the module's and
  class's docstrings for the fuller history of rejected designs (a hard per-row barrier, a
  soft-row-preference tiebreak) and why each one either deadlocked or created corridor contention.
- `warehouse/sim_driver.py` — `SimulationDriver` ties it together tick by tick: run the relocation
  phase to completion (unless `relocate_pallets=False`), then loop: assign idle robots → resync
  reservation table → each controller proposes at most one action (threading shared per-tick
  pick/dock claim counters so two robots sharing a low-stock pallet or an empty pallet to replenish
  don't each independently propose a conflicting action every tick forever) → apply the batch, with
  defensive recovery (drop and force-replan just the offending robot(s) rather than crash) if the
  engine ever rejects an action → log → prune. A stall watchdog triggers a global replan if any
  robot makes no progress for too many consecutive ticks.
- `warehouse/cli/` — `solve.py`, `validate.py` (see Commands above)

## Problem model

A solver needs to simulate this exactly, so treat these rules as authoritative:

- **Grid**: 60 wide x 40 tall, `(x, y)` from `(0,0)` top-left. Row `y=0` is the fulfillment zone
  (deliver orders here). Row `y=39` is the replenishment zone (refill pallets here). Pallets never
  start inside either zone.
- **Entities**: 5 robots, each with internal "storage" inventory; pallets, each holding one SKU
  with a finite `count` up to a per-SKU `maxCount` (all pallets of the same SKU share capacity).
- **Actions** (one per robot per timestep, `<timestep> <robot_id> <action> <x> <y>`):
  - `move x y` — to an orthogonally adjacent empty cell
  - `pick x y` — take 1 item from an adjacent pallet into storage (fails if pallet empty; combined
    picks from multiple robots in the same timestep can't exceed current `count`)
  - `dock x y` — attach an adjacent pallet so it moves with the robot (up to 4 docked, one per side)
  - `undock x y` — detach a docked pallet
  - `fulfill x y` — deliver at `y=0`; robot's storage must EXACTLY match an unfulfilled order (`x y`
    ignored)
- **Replenishment**: automatic and free — at the end of any timestep where a robot sits on `y=39`
  with pallets docked, those docked pallets reset to `maxCount`. Picks earlier in the same timestep
  are applied before the refill. Only docked pallets refill; the robot triggers it, not the pallet.
- **Collision**: no two entities (robots or pallets, docked or not) may occupy the same cell.
- **Scoring**: total timesteps until all 1000 orders are fulfilled — lower is better. Orders don't
  need to be fulfilled in a particular sequence.

## File formats

**Worklist input** (`task/BIG_ORDER.txt`):
```
<num_robots>
<x> <y>              # per robot starting position
<num_skus>
<capacity>           # per SKU, 0-indexed — this is maxCount, and initial count
<num_pallets>
<x> <y> <sku>        # per pallet
<num_orders>
<sku1> <sku2> ...    # per order, space-separated SKUs to collect (with repeats)
```

**Submission output** (what a solver must produce):
```
<timestep> <robot_id> <action> <x> <y>
```
Lines must be in non-decreasing timestep order, no duplicate `(timestep, robot_id)` pairs, and
idle robots simply have no line for that timestep.

There is a browser-based Testbench (external, not in this repo) for visualizing/validating a
submission file before it's uploaded to the leaderboard.

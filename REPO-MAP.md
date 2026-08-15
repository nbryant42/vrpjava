# Repository map

Last verified: 2026-08-15.

`vrpjava` is a Java 21 educational implementation of heuristic and exact vehicle-routing algorithms. It uses ojAlgo
for mathematical optimization and intentionally exposes whether a result is heuristic, merely feasible, or proven
optimal. It is a library plus tests; there is no command-line application. See `README.md` for the motivating papers,
usage notes, performance observations, and the non-commercial license.

## Entry points and data conventions

- `com.github.vrpjava.cvrp.CVRPSolver` is the public CVRP base API. `solve(...)` validates input and returns a
  `CVRPSolver.Result` with a `State`, objective, and set of routes.
- `com.github.vrpjava.cvrp.OjAlgoCVRPSolver` is the exact symmetric CVRP facade. One instance owns a scheduler and worker
  threads, so callers should reuse it and close it (prefer try-with-resources).
- `ClarkeWrightCVRPSolver` and `NearestNeighborCVRPSolver` are CVRP heuristics. Clarke-Wright is the exact solver's
  default incumbent generator and is replaceable through `setHeuristic(...)`.
- `com.github.vrpjava.atsp.ATSPSolver` is the ATSP base API and also contains a nearest-neighbor heuristic.
- `com.github.vrpjava.atsp.OjAlgoATSPSolver` is the ojAlgo assignment-plus-subtour-cut implementation.

CVRP conventions:

- Vertex `0` is the depot. `demands[0]` is zero; customer demands are positive.
- Costs are a symmetric lower-triangular `BigDecimal[][]`. Use `Util.lookup(...)` rather than indexing the absent upper
  triangle directly.
- A returned route starts with `0` and omits the repeated closing depot; objective calculation adds the final edge back
  to `0` implicitly.
- The exact model packs undirected edge variables into a flat lower triangle. The `base(...)` and `getVariable...(...)`
  helpers in `OjAlgoCVRPSolver` define that mapping.

ATSP uses a full square cost matrix and returns an ordered list of directed `Edge` records.

## Exact CVRP flow

1. `CVRPSolver.solve(...)` validates the problem and raises `minVehicles` to the simple capacity lower bound.
2. `OjAlgoCVRPSolver.doSolve(...)` creates a `Job`.
3. `Job` obtains a heuristic incumbent, builds the relaxed two-index vehicle-flow model, and tightens the root bound.
4. `Worker.updateBounds(...)` alternates ojAlgo solves with rounded-capacity cuts from `RccSepCVRPCuts` and connected-
   component/subtour cuts from `SubtourCuts`.
5. `Job.run()` queues the root node and registers it with the solver-owned `Scheduler`.
6. `Scheduler` shares worker threads fairly across jobs. Each `Worker` copies the global model, fixes branch variables,
   solves and separates the node, then either fathoms it, queues two children, or reports an incumbent.
7. `Job` owns the global bound model, incumbent, depth-first/best-first queue, deadline, node counts, and final-state
   transition. Cuts discovered at nodes may also be propagated to its `GlobalBounds` model.

Supporting classes:

- `Node`: immutable branch decisions, depth, and inherited lower bound.
- `GlobalBounds`: synchronized root model and lazily refreshed relaxation result.
- `RccSepCVRPCuts`: exact RCC separation subproblem; uses ojAlgo callback strategies to collect equally strong cuts.
- `CutCandidates` / `CallbackStrategy`: candidate collection and the reflective ojAlgo integer-strategy adapter.
- `SubtourCuts`: cheaper cuts derived from disconnected fractional components.
- `Util`: matrix validation, model/deadline setup, variable construction, cost lookup, and optional hardware tuning.

## Tests

- `src/test/java/com/github/vrpjava/atsp`: nearest-neighbor and exact ATSP behavior.
- `src/test/java/com/github/vrpjava/cvrp/AbstractCVRPSolverTest.java`: shared heuristic contract tests.
- `src/test/java/com/github/vrpjava/cvrp/OjAlgoCVRPSolverTest.java`: exact-solver examples, timeout behavior, helpers,
  and disabled larger benchmarks.
- `src/test/resources/com/github/vrpjava/large-problem.json`: larger fixture used by solver tests.

Use tiny deterministic instances for regressions. For exactness, independently verify route structure and objective;
small brute-force enumeration is preferable to comparing only against another solver path.

## Correctness hotspots

Before modifying search or separation, trace both the mathematical result and its proof status:

- `.level(...)` means equality in ojAlgo; many no-good and subtour cuts require only `.upper(...)` or `.lower(...)`.
- A non-optimal ojAlgo state can mean timeout or failure, not just infeasibility.
- An integer-looking solution can still violate capacity constraints absent a required RCC.
- Queue exhaustion is not a proof if a worker abandoned an unresolved node.
- Conversely, a proven global lower bound equal to a valid heuristic incumbent is enough to report optimality.
- `Scheduler`, `Job`, global cuts, and RCC callback collection cross thread boundaries; audit locking and exception paths.
- ojAlgo's RCC callbacks may run concurrently; `CutCandidates` must synchronize its objective and mutable cut set.
- An exception must not strand a node in flight or permanently kill a scheduler worker.
- Deadline handling must terminate loops without presenting a merely feasible result as exact.

## Current handoff (update when resolved)

The main worktree currently contains three uncommitted, deliberately failing correctness-test changes. Preserve them:

- `ATSPSolverTest`: rejected provisional nearest-neighbor candidates must not be marked visited.
- `OjAlgoATSPSolverTest`: eliminating an incumbent subtour must not exclude the true Hamiltonian optimum.
- `OjAlgoCVRPSolverOracleTest`: six tiny brute-force oracle cases plus a lower-bound-equals-incumbent proof case.

The intended green commit sequence is:

1. Pair the nearest-neighbor regression with moving the visited mutation after final destination selection.
2. Pair the ATSP regression with changing the incumbent-subtour equality cut to an upper-bound cut.
3. Pair the CVRP oracle/proof regressions with explicit search-proof-completeness handling. Do not implement this as an
   unconditional `HEURISTIC`-to-`OPTIMAL` promotion when the queue empties; unresolved/timed-out nodes must prevent an
   optimality claim.

Historical work recovered from the old dirty worktree is committed and pushed on `wip/recovered-rcc-comb`; treat it as
an archival experiment rather than merging it wholesale:

- `1b5807c` preserves the comb-inequality draft and the original depth-one RCC experiment.
- `65984cd` extends full RCC separation through depths 1-3, caps each RCC solve at 15 seconds, changes a search
  threshold, and appends caveats/next steps to `CombInequalities.md`.
- Its timeout path can consume a non-optimal RCC result, so it is not proof-safe as written.
- No comb separator was implemented. For fixed `t = 2`, the draft's parity observation is sound; variable tooth count
  requires `sum(alpha) + 3t = 2 beta + 1`. Before implementation, finish the membership/domain constraints, clarify
  full tooth disjointness, linearize the boundary objective, and validate generated cuts exhaustively on tiny cases.

The remote `cutPooling` branch is another explicitly experimental line and is not part of `main`.

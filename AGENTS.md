# Repository guidance

Read `README.md` for the project overview and `REPO-MAP.md` before changing solver internals.

This is a Java 21/Maven project containing heuristic solvers and exact branch-and-cut solvers. Correctness and proof
status matter more than performance: `OPTIMAL` is a mathematical claim, not merely the best result found so far.

## Build and test

Use the checked-in Maven wrapper.

- Windows: `.\mvnw.cmd test`
- macOS/Linux: `./mvnw test`
- Focused test on Windows: `.\mvnw.cmd -Dtest=ClassName test`
- Package: `.\mvnw.cmd package`
- Coverage and reports: `.\mvnw.cmd -Pcoverage clean test site`

Run focused deterministic tests while iterating, then the full active test suite. Some large-instance tests are disabled
benchmarks; do not routinely enable them. Do not use wall-clock races or sleeps as correctness regressions when a
deterministic seam or tiny brute-force oracle can test the same state transition.

## Solver correctness

- Never interpret a timeout, interrupted cut separation, solver failure, or other non-optimal ojAlgo result as proof
  that a node is infeasible or fully explored.
- An empty work queue proves completion only when every removed node was resolved by infeasibility, a valid bound,
  branching, or a validated feasible solution. Preserve incomplete-search information in the final result state.
- Validate an integer candidate against all required constraints, including route capacity, before reporting it.
- Every feasible CVRP result must visit each customer exactly once, start each route at depot `0`, respect capacity and
  the minimum vehicle count, and report an objective consistent with its routes.
- Preserve the input conventions: CVRP costs are symmetric and lower-triangular; ATSP costs use a full square matrix.
- Treat packed lower-triangle variable indexing, cut direction, result-state handling, deadlines, and shared mutable
  callback state as high-risk code. Prefer small, independently checkable changes there.

For a correctness bug, add a regression that fails for the intended reason, then make the smallest sound fix. Keep the
regression and its fix together in one green atomic commit unless the user explicitly requests otherwise. Do not mix
cleanup or performance experiments into a correctness commit.

## Working-tree safety

Inspect `git status --short --branch` before and after working. Existing staged, unstaged, and untracked files belong to
the user.

- Do not reset, restore, stash, overwrite, stage, or commit unrelated changes.
- Keep edits narrowly scoped and review `git diff` and `git diff --check`.
- Do not commit or push unless the user explicitly asks.
- `target/` is generated output and should remain untracked.

## Commit messages

Every commit message must use this structure:

```text
Short imperative header

Explain what changed and why, with a little more detail. Mention important correctness implications and verification
when relevant.

Co-authored-by: Codex (GPT-5.6 Sol) <codex@openai.com>
```

Keep the header concise, separate the header, body, and trailer with blank lines, and make the co-author trailer the
final line.

# Solver experiments

The `benchmarkEil33` and `benchmarkEil51` tests are disabled unless `vrp.benchmark` selects the corresponding instance
(or `all`). They emit one CSV row per trial. Useful Maven properties are:

- `vrp.trials` (default `3`)
- `vrp.timeoutMillis` (default `300000`)
- `vrp.searchStrategy`: `AUTO`, `DEPTH_FIRST`, or `BEST_FIRST`
- `vrp.rccMinNodeDepth`: minimum non-root branch depth for full RCC separation; the root is always considered
- `vrp.rccDepth`: maximum branch depth for full RCC separation; `0` means root only
- `vrp.rccMillis`: budget for each RCC separation call; `0` disables full RCC and `Long.MAX_VALUE` is limited only by
  the solve deadline
- `vrp.heuristicCombMillis`: total root budget for heuristic strengthened-comb separation; `0` disables it (the default)
- `vrp.multistarMillis`: total root budget for exact generalized-large-multistar separation; `0` disables it (the
  default)
- `vrp.threeToothMillis`: total root budget for the restricted three-tooth separator; `0` disables it (the default)
- `vrp.capacity`: override the benchmark instance's vehicle capacity (`8000` for `eil33`, `160` for `eil51`)
- `vrp.roundCosts`: use integer-rounded edge costs for `eil33`; `eil51` is always rounded
- `vrp.bestFirstRatio` and `vrp.bestFirstMillis`: thresholds used only by `AUTO`

For example, from PowerShell:

```powershell
.\mvnw.cmd "-Dtest=OjAlgoCVRPSolverTest#benchmarkEil51" "-Dvrp.benchmark=eil51" `
  "-Dvrp.trials=3" "-Dvrp.timeoutMillis=60000" "-Dvrp.searchStrategy=BEST_FIRST" `
  "-Dvrp.rccDepth=0" "-Dvrp.rccMillis=1000" "-Dvrp.heuristicCombMillis=10000" `
  "-Dvrp.multistarMillis=2000" test
```

## Initial observations (2026-08-15)

These are exploratory measurements on one Raptor Lake development machine, not stable performance claims. Trials in
the tables ran sequentially in one Maven/JVM invocation, so the first trial includes more JVM and solver warm-up. Both
ojAlgo and the parallel search show substantial run-to-run variability.

| Instance/configuration | Trials | Root bound | Median nodes | Median final cuts | Median runtime | Outcome |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| eil33, best-first, root RCC unrestricted | 5 | 835.227782375 | 448 | 205 | 1.27 s | all optimal |
| eil33, best-first, root RCC 100 ms/call | 5 | 835.227782375 in 4/5; 832.660041690 once | 452 | 224 | 1.36 s | all optimal |
| eil33, best-first, RCC through depth 1, 100 ms/call | 5 | 835.227782375 in 4/5; 828.865119905 once | 577 | 314 | 2.38 s | all optimal |
| eil33, depth-first, root RCC unrestricted | 5 | 835.227782375 | 4,952 | 447 | 8.48 s | 4 optimal; 1 timed out at 30 s |
| eil33, best-first, full RCC disabled | 3 | 691.209970275 | 3,110 | 631 | 30.05 s | all timed out with heuristic incumbent |
| eil51, best-first, root RCC unrestricted | 3 | about 514.45454550 | 5,728 | 207 | 21.26 s | all optimal |
| eil51, best-first, root RCC 100 ms/call | 3 | 509.568750023 to 512.027732473 | 15,681 | 289 | 60.02 s | all timed out with heuristic incumbent |
| eil51, best-first, root RCC 1 s/call | 1 | 514.454545509 | 4,623 | 185 | 14.77 s | optimal |

Five separate pre-change Maven/JVM runs of the default `eil33` test took 1.69-2.30 seconds (median 1.93 seconds) and
searched 209-514 nodes. That is a useful reminder that same-JVM medians above are suitable for comparing configurations
within one run, but not directly comparable to cold invocations or IDE timing.

The early signal is that root RCC quality is load-bearing for both instances. A fixed short budget has a threshold
effect rather than a smooth runtime/quality tradeoff: 100 ms was usually enough for `eil33` but not for `eil51`, while a
single 1-second `eil51` trial recovered the unrestricted root bound. Shallow node RCC did not pay for itself on `eil33`
in this small sample. More trials, randomized configuration order, and additional instances are needed before changing
defaults.

## Stoer-Wagner connectivity separation (2026-08-15)

After adding the exact global-minimum-cut fallback to `SubtourCuts`, three default `AUTO`/root-RCC-unrestricted trials
gave:

| Instance | Trials | Root bound | Median nodes | Median final cuts | Median runtime | Outcome |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| eil33 | 3 | 835.227782375 | 350 | 240 | 1.41 s | all optimal |
| eil51 | 3 | 514.523809526 to 514.523809533 | 4,455 | 278 | 20.53 s | all optimal |

The `eil33` root bound is unchanged, and its node and runtime differences are well inside the variability seen above.
For `eil51`, the stronger root bound appeared in every trial; median nodes fell from 5,728 to 4,455 while the additional
cuts left median runtime essentially unchanged in this small sample. The reduced `eil33` root-bound test also moved
from 422.810195025 to its known optimum, 428.7145713, reproducibly across three separate Maven runs. These results
support keeping the separator enabled for the default `AUTO` configuration, but they are still exploratory rather
than a broad performance claim. On both benchmark instances, `AUTO` switches to best-first immediately after the root,
so these results are comparable to the earlier best-first rows.

Depth-first spot checks were dominated by the solver's long-tail variability. One controlled three-trial benchmark
favored the new separator (median 4.36 seconds and 2,223 nodes versus 10.53 seconds and 4,282 nodes at the preceding
commit), while three separate cold/debug JVM runs reversed the medians (12.49 seconds and 3,467 nodes versus 3.70
seconds and 2,613 nodes) and timed out once on each version. That is not evidence for a stable speedup or regression;
larger interleaved samples would be needed before changing depth-first behavior.

## Restricted three-tooth experiment (2026-08-18)

The first root-only fixed-three-tooth implementation was tested on `eil51` with `BEST_FIRST`, unrestricted root RCC,
and a 15-second total three-tooth budget. One no-comb control and four separate comb-enabled trials gave:

| Configuration | Trials | Root bound | Median root time | Median nodes | Median final cuts | Median runtime | Outcome |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| no comb | 1 | 514.523809533 | 15.20 s | 3,989 | 302 | 27.36 s | optimal |
| three-tooth, 15 s | 4 | 515.913043465 to 516.142857165 | 31.14 s | 1,353 | 246 | 35.83 s | all optimal |

The stronger bound appeared in every enabled trial and substantially reduced the search tree and final cut count. The
extra root work did not yet pay back consistently in elapsed time: one enabled trial spent 90 seconds at the root,
apparently in the familiar ojAlgo/RCC long tail after a comb cut caused the separation loop to restart. This is strong
evidence that the restricted family adds polyhedral strength on `eil51`, but not enough runtime data to enable it by
default. Smaller budgets and better control of the RCC re-solves are the next useful experiments.

A subsequent `eilD76_k4` spot check used depth-5 node RCC and a 60-second three-tooth budget. Two comb cuts raised the
rounded root bound from 586 to 587, but root time increased from 20.8 to 73.6 seconds. The known 593 solution appeared
after 1,144 seconds and 1,399 nodes, compared with 945 seconds and 1,063 nodes in the earlier no-comb run; neither run
proved optimality within 30 minutes. This is only a pair of highly variable runs, but it reinforces the need for a
strict separator budget and gives no reason to enable exact three-tooth separation by default.

## Heuristic strengthened-comb smoke tests (2026-08-19)

Two `eil51` smoke trials used `BEST_FIRST`, unrestricted root RCC, a 10-second total heuristic-comb budget, and no exact
three-tooth separation:

| Conditions | Root bound | Root time | Nodes | Final cuts | Total time | Outcome |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| competing heavy CPU task | 516.379310338 | 28.83 s | 1,928 | 346 | 69.27 s | optimal |
| otherwise idle machine | 516.333333339 | 29.26 s | 553 | 234 | 34.53 s | optimal |

Both bounds are stronger than all four restricted-exact trials above, which is encouraging evidence that broader teeth
matter. Nearly identical root times but sharply different search times also fit this repository's high parallel-search
variability. The clean trial was slightly faster than the restricted-exact median and substantially reduced its median
node count, but these are not interleaved samples; more trials are still required before claiming a speedup.

## Exact generalized-large-multistar smoke tests (2026-09-01)

The initial GLM implementation separated one most-violated nucleus at a time by an exact directed minimum cut. The
final prototype also extracts a linear-size canonical subset of tied minimum cuts from the residual graph, then adds
the equally strongest GLMs as one batch before re-solving. It remains root-only and disabled by default. The base model
in both the disabled and enabled rows below also fixed customer edges to zero when their endpoint demands together
exceed vehicle capacity; comparisons with older commits therefore cannot isolate that valid preprocessing change.

These are single same-worktree smoke trials, not runtime claims. They used `BEST_FIRST`, unrestricted root RCC, no comb
or three-tooth separator, and integer-rounded costs. The rounded `eil33` runs used capacity 4000 and timed out; the
`eil51` runs used capacity 160 and proved the known optimum of 521.

| Instance/configuration | Root bound | Root time | Root cuts | Nodes | Total time | Outcome |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| eil51, GLM disabled | 514.523809553 | 12.03 s | 117 | 4,861 | 23.09 s | optimal |
| eil51, GLM 10 s | 514.523809539 | 12.35 s | 125 | 3,153 | 18.93 s | optimal |
| eil33/Q4000, GLM disabled | 1427.166666634 | 5.30 s | 201 | 9,683 | 30.00 s | timed out, incumbent 1531 |
| eil33/Q4000, single-cut GLM 2 s | 1428.745740000 | 7.31 s | 196 | 3,638 | 15.00 s | timed out, incumbent 1531 |
| eil33/Q4000, single-cut GLM 10 s | 1429.652278100 | 14.96 s | 248 | 4,171 | 30.00 s | timed out, incumbent 1531 |
| eil33/Q4000, tied-batch GLM 2 s | 1428.470116448 | 10.10 s | 254 | 1,493 | 15.00 s | timed out, incumbent 1531 |
| eil33/Q4000, tied-batch GLM 10 s | 1430.386693394 | 15.96 s | 270 | 3,502 | 30.00 s | timed out, incumbent 1531 |
| eil51, heuristic comb 10 s + tied-batch GLM 10 s | 516.379310328 | 23.22 s | 195 | 885 | 33.84 s | optimal |

GLM produced no material root-bound gain on `eil51`, either alone or after the heuristic comb separator. On the
higher-vehicle rounded `eil33` formulation, the strongest single run raised the bound by about 3.22. Tied batching did
not improve the two-second result, but did improve the ten-second result over the one-cut prototype. That is a useful
polyhedral-strength signal, not proof of a stable batching advantage: the different timeouts, single trials, and this
solver's large run-to-run variation make the bound, node, and total-time differences unsuitable for performance claims.

A later `eilD76_k4` spot check used capacity 360, depth-5 node RCC, and 60-second budgets for both heuristic comb and
GLM separation. The comb heuristic found 87 cuts and reached a rounded root bound of 588; the exact GLM separator then
finished in 8 milliseconds without finding a cut. Thus GLM provided no incremental strengthening of that post-comb
relaxation.

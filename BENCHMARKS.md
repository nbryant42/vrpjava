# Solver experiments

The `benchmarkEil33` and `benchmarkEil51` tests are disabled unless `vrp.benchmark` selects the corresponding instance
(or `all`). They emit one CSV row per trial. Useful Maven properties are:

- `vrp.trials` (default `3`)
- `vrp.timeoutMillis` (default `300000`)
- `vrp.searchStrategy`: `AUTO`, `DEPTH_FIRST`, or `BEST_FIRST`
- `vrp.rccDepth`: maximum branch depth for full RCC separation; `0` means root only
- `vrp.rccMillis`: budget for each RCC separation call; `0` disables full RCC and `Long.MAX_VALUE` is limited only by
  the solve deadline
- `vrp.bestFirstRatio` and `vrp.bestFirstMillis`: thresholds used only by `AUTO`

For example, from PowerShell:

```powershell
.\mvnw.cmd "-Dtest=OjAlgoCVRPSolverTest#benchmarkEil51" "-Dvrp.benchmark=eil51" `
  "-Dvrp.trials=3" "-Dvrp.timeoutMillis=60000" "-Dvrp.searchStrategy=BEST_FIRST" `
  "-Dvrp.rccDepth=0" "-Dvrp.rccMillis=1000" test
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

# vrpjava

This package contains a small collection of algorithms for the symmetric Capacitated Vehicle Routing Problem (CVRP),
written in pure Java.

There is an implementation of the classic Clarke-Wright savings algorithm, but the main thing of interest here is an
exact branch-and-bound-and-cut solver based on the 2-index Vehicle Flow Formulation and the Pavlikov-Petersen-Sørensen
"RCC-Sep" algorithm.

These algorithms are discussed in some papers:

* https://repub.eur.nl/pub/135594/EI2021-01.pdf
* https://onlinelibrary.wiley.com/doi/10.1002/net.22183

I'm not claiming this is the fastest or best solver in the world; after all, it's pure Java based on
the open-source MILP solver library, ojAlgo. But it should be easy enough to run this on top of CPLEX
or Gurobi via one of ojAlgo's available integrations.

This code is early and I can't guarantee a stable API yet. But it has proven capable enough to solve,
to optimality, some medium-size problems; my main test-case has been the EIL33 case discussed at
https://github.com/IBMDecisionOptimization/Decision-Optimization-with-CPLEX-samples/blob/master/Vehicle-routing.pdf.
This problem has 32 customers and the optimal solution requires 4 vehicles. On my Ryzen 3900X, the code
will typically solve it to optimality in about 20-40 seconds.
(Runtime is a bit unpredictable due to multiple equivalent LP relaxations that define different starting points.)

See the Javadoc for more details; start with the `OjAlgoCVRPSolver` class.

There are at least a couple potential areas for improvement: constraint format could
be optimized, depending on the subset size, as discussed in the RCC-Sep paper.
Multithreading, perhaps. Maybe some more thought to the class and package structure.

But what this needs most, right now, is another pair of eyes to review the code.

## Viewing the Javadoc

Clone the repository, run `./mvnw site`, and point your local browser at `target/site/index.html`.
Look under Project Reports.

## Running the tests

The test coverage report is off by default because it slows the tests down considerably, so you need to enable the
`coverage` profile. Run `./mvnw -Pcoverage clean test site`, and point your local browser at `target/site/index.html`.
Look under Project Reports.

## Building the JAR

`./mvnw package` and look in the `target/` directory.

## Search algorithm

The algorithm starts out with a depth-first branch-and-bound search. After it begins to find
feasible solutions, it switches to a best-first search by changing the node stack to a priority queue
on the fly. But best-first will not work for some of the harder problem instances
(such as EIL33 modified with a vehicle capacity of 4000), so there are two configurable thresholds
to limit whether and when this switch happens.

See the methods `OjAlgoCVRPSolver.setBestFirstMillis()` and `OjAlgoCVRPSolver.setBestFirstRatio()`
for those tunables.

## Why an exact solver, and not heuristic?

The exact solver may be mainly of academic interest, and as a benchmark for heuristics, but it's not entirely out of the
question to use it in production; exact and heuristic solvers are complementary. This solver starts with a configurable
heuristic to define the current best-known-solution. (Right now this defaults to the Clarke-Wright algorithm, but if you
have some code that performs better than the default, a plugin interface is available.)

It will then search for a better solution, skipping parts of the search tree which provably cannot be better than
the current best known solution, until it hits whatever timeout you have set, or it has found the provably optimal
solution. If it hits the timeout first, it may still have found a better solution. So in a sense, it is both exact and
approximate, and, the better the starting heuristic, the faster we can get through the branch-and-bound process.

This can be extremely fast for the easier problem instances, and generally works well as long as the lower-bound
function ("RCC-Sep") performs well enough to exclude enough parts of the search tree. That may not be the case for
harder problems, which include:

* those have many customers
* those that require more vehicles
* or even those that require fewer vehicles, but vehicle(s) are close to full capacity. It feels like any problem that
  is too close to a breaking point between _n_ and _n+1_ vehicles may be harder to solve.

## License

Non-commercial use only, at least for the time being. Academic and student use is OK.

Contact the author at (nbryant at optonline dot net) to discuss an alternative license.
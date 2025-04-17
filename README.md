# vrpjava

This package contains a small collection of algorithms for the symmetric Capacitated Vehicle Routing Problem (CVRP),
written in pure Java.

There is an implementation of the classic Clarke-Wright savings algorithm, but the main thing of interest here is an
exact branch-and-cut solver based on the 2-index Vehicle Flow Formulation and the Pavlikov-Petersen-Sørensen
"RCC-Sep" procedure. These are discussed in some papers:

* https://repub.eur.nl/pub/135594/EI2021-01.pdf
* https://onlinelibrary.wiley.com/doi/10.1002/net.22183

The remainder of this discussion will mostly focus on the exact solver.

This is certainly not the fastest or best solver in the world; there are a number of more advanced techniques it does
not implement, and after all, it's pure Java based on the open-source MILP solver library, ojAlgo. But it should be easy
enough to run this on top of a faster MILP solver via one of ojAlgo's
available [integrations](https://www.ojalgo.org/ojalgo-extensions/).

This code is early and I can't guarantee a stable API yet. But it has proven capable enough to solve,
to optimality, some medium-size problems; my main test-case has been the EIL33 case discussed at
https://github.com/IBMDecisionOptimization/Decision-Optimization-with-CPLEX-samples/blob/master/Vehicle-routing.pdf.
This problem has 32 customers and the optimal solution requires 4 vehicles. Now that the code is fully multithreaded,
this solves in about 2 seconds on a Core i7-14700K. EIL51 solves in about 20 seconds. Most of the runtime is in the
initial cut-generation phase at the root node, which cannot be parallelized at a high level.

See the Javadoc for more details; start with the `OjAlgoCVRPSolver` class.

There are a few potential areas for improvement:

* There's another branching strategy which makes use of the fact that any customer subset must have an even number of
  outbound edges. See [Lysgaard et al.](https://www.lancaster.ac.uk/staff/letchfoa/articles/2004-cvrp-exact.pdf)
  and [Augerat et al.](https://www.osti.gov/etdeweb/servlets/purl/289002.)
* Those papers also describe a few classes of cutting planes that should improve our bounds.
* Tackling some of the very hard problem instances may require some sort of cut-pool management.

But what this code needs most, right now, is another pair of eyes.

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

The algorithm first iterates on the RCC-Sep procedure (which is itself NP-hard) until it's no longer possible to
generate additional cuts. Then it proceeds with a parallel, best-first branch-and-cut search, but the strategy for the
non-root nodes is different from the root node; here, it mostly avoids the RCC-Sep procedure, only using the RCC-Sep
procedure when we have an integer solution that is known to violate one or more capacity constraints.

There are also some tunable parameters that can be used to revert to a depth-first search. The idea is that for the
hardest problem instances, we may not be able to solve to optimality, so the goal would be to merely generate an
improved feasible solution more quickly. This works as an attempt to guess the best strategy based on the problem at
hand, so in this case, the algorithm starts out with a depth-first branch-and-cut search. When the current
best-known-solution is close enough to the lower bound, it switches to a best-first search by changing the node stack to
a priority queue on the fly. There are two configurable thresholds to limit whether and when this switch happens.

See the methods `OjAlgoCVRPSolver.setBestFirstMillis()` and `OjAlgoCVRPSolver.setBestFirstRatio()`
for those parameters.

(This auto-detection logic doesn't work very well, so the default settings are currently biased towards best-first
search. The problem is that the Clarke-Wright heuristic is actually pretty good, so even for some of the hard problems
we now start out with a higher bounds ratio.)

## Why an exact solver, and not heuristic?

This is an educational project. The exact solver may be mainly of academic interest, and as a benchmark for heuristics,
but it's not entirely out of the question to use it in the real world; exact and heuristic solvers are complementary.
This solver starts with a configurable heuristic to define the current best-known-solution. (Right now this defaults to
the Clarke-Wright algorithm, but if you have some code that performs better than the default, a plugin interface is
available.)

It will then search for a better solution, skipping parts of the search tree which provably cannot be better than
the current best known solution, until it hits whatever timeout you have set, or it has found the provably optimal
solution. If it hits the timeout first, it may still have found a better solution. So in a sense, it is both exact and
approximate, and, the better the starting heuristic, the faster we can get through the branch-and-cut process.

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
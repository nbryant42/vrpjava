package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.CVRPSolver.Result;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.AbstractMap;
import java.util.ArrayDeque;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.stream.IntStream;
import java.util.stream.LongStream;

import static com.github.vrpjava.Util.newModel;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.addCuts;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildConstraints;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildVars;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.findCycles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.minimize;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.toSeconds;
import static java.lang.Double.POSITIVE_INFINITY;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;
import static org.ojalgo.optimisation.Optimisation.State.INFEASIBLE;

class Job {
    private final OjAlgoCVRPSolver solver;
    private final long timeout;
    private final int minVehicles;
    private final int maxVehicles;
    private final BigDecimal vehicleCapacity;
    private final BigDecimal[] demands;
    private final BigDecimal[][] costMatrix;

    public Job(OjAlgoCVRPSolver solver, int minVehicles, int maxVehicles, BigDecimal vehicleCapacity,
               BigDecimal[] demands, BigDecimal[][] costMatrix, long timeout) {
        this.solver = solver;
        this.timeout = timeout;
        this.minVehicles = minVehicles;
        this.maxVehicles = maxVehicles;
        this.vehicleCapacity = vehicleCapacity;
        this.demands = demands;
        this.costMatrix = costMatrix;
    }

    Result run() {
        var start = System.currentTimeMillis();
        var deadline = start + timeout;
        var kickstarter = solver.getHeuristic().doSolve(minVehicles, maxVehicles, vehicleCapacity, demands, costMatrix,
                timeout);
        var globalBounds = initBounds(minVehicles, maxVehicles, vehicleCapacity, demands, costMatrix, deadline);
        double bestKnown;

        if (kickstarter.state() != Result.State.HEURISTIC) {
            bestKnown = POSITIVE_INFINITY;

            if (!globalBounds.getResult(deadline).getState().isOptimal()) {
                var myState = globalBounds.getResult(deadline).getState() == INFEASIBLE ? Result.State.INFEASIBLE :
                        Result.State.UNEXPLORED;

                return buildResult(myState, demands, null, kickstarter, 0, globalBounds, bestKnown);
            }
        } else {
            bestKnown = kickstarter.objective();

            if (!globalBounds.getResult(deadline).getState().isOptimal()) {
                return buildResult(Result.State.HEURISTIC, demands, null, kickstarter, 0, globalBounds, bestKnown);
            }
        }

        logCutCount(globalBounds);

        // queue of pending nodes for branch-and-bound search. Each node is represented as a Map
        // of variable IDs and values to be fixed in the model.
        // The queue can be either a LIFO queue (stack) for depth-first search or a priority queue for best-first
        // search. Depth-first search finds many valid solutions more quickly, whereas best-first search is more likely
        // to proceed directly to the better solutions, but will explore more internal nodes before it gets there. We
        // switch strategies based on the problem at hand, so don't assume this will always be a LIFO queue.
        var queue = evaluateStrategy(globalBounds.getResult(deadline).getValue(), bestKnown, start,
                Collections.asLifoQueue(new ArrayDeque<>()), "Initial");
        var nodes = 0;
        var myState = kickstarter.state();
        Optimisation.Result incumbent = null;

        // the root node has no variables fixed.
        queue.add(new Node(globalBounds.getResult(deadline).getValue(), ZERO, Map.of()));

        for (; deadline > System.currentTimeMillis(); nodes++) {
            var node = queue.poll();

            if (node == null) {
                if (myState == Result.State.FEASIBLE) {
                    myState = Result.State.OPTIMAL;
                }
                break;
            }
            // double-check the parent node's bound before we go any further; the best-known solution may have
            // improved since it was queued.
            if (node.bound() >= bestKnown) {
                continue; // fathom the node
            }
            var nodeModel = globalBounds.getModel().copy(true, false);
            node.vars().forEach((k, v) -> nodeModel.getVariable(k).level(v));

            var nodeResult = weakUpdateBounds(vehicleCapacity, demands, nodeModel, globalBounds, deadline);
            var ub = nodeResult.getValue();

            // if it's not optimal, it's probably INFEASIBLE, with nonsense variables, or a timeout.
            if (!nodeResult.getState().isOptimal() || ub >= bestKnown) {
                continue; // fathom the node
            }

            // branch on the fractional variable with the smallest rounding gap to the nearest integer.
            var frac = IntStream.range(0, (int) nodeResult.count())
                    .mapToObj(i -> {
                        var v = nodeResult.get(i);
                        v = v.setScale(0, RoundingMode.HALF_EVEN).subtract(v).abs();
                        return new AbstractMap.SimpleImmutableEntry<>(i, v);
                    })
                    .filter(entry -> entry.getValue().signum() > 0)
                    .min(Map.Entry.comparingByValue());

            if (frac.isPresent()) {
                var k = frac.get().getKey();
                var gap = frac.get().getValue();
                var v = nodeResult.get(k);
                var closest = v.setScale(0, RoundingMode.HALF_EVEN);
                var other = closest.compareTo(v) > 0 ? closest.subtract(ONE) : closest.add(ONE);

                // queue two child nodes with variable fixed to `closest` and `other`.
                // queue the closest gap last, so it's on top of stack (if it's a LIFO queue)
                queueNode(node, ub, k, other, gap, queue);
                queueNode(node, ub, k, closest, gap, queue);
            } else {
                var globalBoundsResult = globalBounds.getResult(deadline);
                var lb = globalBoundsResult.getValue();
                queue = evaluateStrategy(lb, ub, start, queue, "New");

                incumbent = nodeResult;
                bestKnown = ub;
                myState = Result.State.FEASIBLE;

                if (ub <= lb) {
                    myState = Result.State.OPTIMAL;
                    break;
                }
                if (!globalBoundsResult.getState().isOptimal()) {
                    break;
                }
            }
        }

        return buildResult(myState, demands, incumbent, kickstarter, nodes, globalBounds, bestKnown);
    }

    private Result buildResult(Result.State myState,
                               BigDecimal[] demands,
                               Optimisation.Result incumbent,
                               Result kickstarter,
                               int nodes,
                               GlobalBounds globalBounds,
                               double bestKnown) {
        var cycles = incumbent == null ? kickstarter.cycles() : findCycles(demands.length, incumbent);

        solver.debug(nodes + " nodes, " + cycles.size() + " cycles: " + cycles);
        var cycleDemands = cycles.stream()
                .map(cycle -> cycle.stream().map(i -> demands[i]).reduce(ZERO, BigDecimal::add))
                .toList();
        solver.debug("Cycle demands: " + cycleDemands);
        logCutCount(globalBounds);

        return new Result(myState, bestKnown, cycles);
    }

    private void logCutCount(GlobalBounds globalBounds) {
        var count = globalBounds.getModel().getExpressions().stream().filter(e -> e.getName().contains("cut:")).count();
        solver.debug("Currently " + count + " cuts.");
    }

    private static void queueNode(Node parent, double ub, Integer k, BigDecimal v, BigDecimal gap,
                                  Queue<Node> queue) {
        var childVars = new HashMap<>(parent.vars());
        childVars.put(k, v);
        queue.add(new Node(ub, gap, childVars));
    }

    static GlobalBounds initBounds(int minVehicles,
                                   int maxVehicles,
                                   BigDecimal vehicleCapacity,
                                   BigDecimal[] demands,
                                   BigDecimal[][] costMatrix,
                                   long deadline) {
        var model = newModel(deadline);
        var vars = buildVars(costMatrix, model);

        buildConstraints(model, minVehicles, maxVehicles, vars);
        model.relax();

        return new GlobalBounds(model, updateBounds(vehicleCapacity, demands, model, null, deadline));
    }

    private static Optimisation.Result updateBounds(BigDecimal vehicleCapacity,
                                                    BigDecimal[] demands,
                                                    ExpressionsBasedModel model,
                                                    GlobalBounds globalBounds,
                                                    long deadline) {
        var result = minimize(model, deadline);
        var cuts = new HashSet<Set<Integer>>();
        var size = demands.length;

        for (Set<OjAlgoCVRPSolver.Cut> rccCuts; result.getState().isOptimal() &&
                (rccCuts = RccSepCVRPCuts.generate(vehicleCapacity, demands, result, deadline)) != null; ) {
            if (addCuts(rccCuts, cuts, model, result, globalBounds, size)) {
                result = minimize(model, deadline);
                continue;
            }
            var subtourCuts = SubtourCuts.generate(vehicleCapacity, demands, result);

            if (addCuts(subtourCuts, cuts, model, result, globalBounds, size)) {
                result = minimize(model, deadline);
                continue;
            }

            // no more cuts to add. done.
            break;
        }

        // done or timed out.
        return result;
    }

    /**
     * Update the bounds model, when called from a search node. This is similar to
     * {@link #updateBounds(BigDecimal, BigDecimal[], ExpressionsBasedModel, GlobalBounds, long)},
     * but does not solve the RCC-Sep model unless absolutely necessary.
     * (we need to check validity. when the node solution is integer)
     */
    private static Optimisation.Result weakUpdateBounds(BigDecimal vehicleCapacity,
                                                        BigDecimal[] demands,
                                                        ExpressionsBasedModel model,
                                                        GlobalBounds globalBounds,
                                                        long deadline) {
        var result = minimize(model, deadline);
        var cuts = new HashSet<Set<Integer>>();
        var size = demands.length;

        while (result.getState().isOptimal()) {
            var subtourCuts = SubtourCuts.generate(vehicleCapacity, demands, result);

            if (addCuts(subtourCuts, cuts, model, result, globalBounds, size)) {
                result = minimize(model, deadline);
                continue;
            }

            // no more cuts to add. done.
            // if there are no fractional variables, this is a candidate solution, but we don't know for sure until
            // we've validated that it satisfies the full set of constraints, so iterate on additional cuts.
            // This needs to be done on a fast path, so don't run the RCC-Sep ILP model unless confirmed invalid.
            return isInvalidIntegerSolution(vehicleCapacity, demands, result) ?
                    updateBounds(vehicleCapacity, demands, model, globalBounds, deadline) :
                    result;
        }

        return result; // infeasible or timed out.
    }

    /**
     * Possibly update the search strategy from depth-first to best-first.
     *
     * @return either an updated queue (if changing the strategy) or the existing queue unmodified
     */
    private Queue<Node> evaluateStrategy(double lb, double ub, long start, Queue<Node> queue, String descriptor) {
        var ratio = lb / ub;
        var elapsed = System.currentTimeMillis() - start;
        var suffix = (float) lb + "/" + (float) ub + " (" +
                BigDecimal.valueOf(ratio * 100.0).setScale(2, RoundingMode.HALF_EVEN) + "%)";

        if (!(queue instanceof PriorityQueue<Node>) && ratio > solver.getBestFirstRatio() &&
                elapsed < solver.getBestFirstMillis()) {
            // Found a feasible solution, and bounds are tight enough that best-first search may help.
            // Switch to best-first search.
            // Also, if it's taken us more than X amount of time to get here, then we're working a
            // hard problem, and it's not a good idea to switch.
            queue = new PriorityQueue<>(queue);
            solver.debug("[" + toSeconds(elapsed) + "s]: Switching to best-first search. Bounds now " + suffix);
        } else {
            solver.debug("[" + toSeconds(elapsed) + "s]: " + descriptor + " solution. Bounds now " + suffix);
        }
        return queue;
    }

    // Warning -- this does not check for sub-tours -- that's assumed to be handled elsewhere.
    private static boolean isInvalidIntegerSolution(BigDecimal vehicleCapacity,
                                                    BigDecimal[] demands,
                                                    Optimisation.Result result) {
        return isIntegerSolution(result) && findCycles(demands.length, result).stream().anyMatch(cycle ->
                cycle.stream().map(i -> demands[i])
                        .reduce(ZERO, BigDecimal::add)
                        .compareTo(vehicleCapacity) > 0);
    }

    private static boolean isIntegerSolution(Optimisation.Result result) {
        return LongStream.range(0, result.count()).allMatch(i -> {
            var v = result.get(i);
            return v.setScale(0, RoundingMode.HALF_EVEN).compareTo(v) == 0;
        });
    }
}

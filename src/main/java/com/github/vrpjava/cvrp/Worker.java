package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.ExperimentalParameters;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.IntStream;
import java.util.stream.LongStream;

import static com.github.vrpjava.cvrp.Job.addCuts;
import static com.github.vrpjava.cvrp.CVRPSolver.minVehicles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.findCycles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.minimize;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.toSeconds;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;
import static java.util.Comparator.comparing;
import static java.util.Map.Entry.comparingByValue;
import static java.util.stream.Collectors.toSet;
import static org.ojalgo.optimisation.Optimisation.State.INFEASIBLE;

/**
 * Node-processing logic executed by {@link Scheduler} worker threads.
 */
final class Worker {
    private static final int NUMERICAL_GUARD_DIGITS = 6;
    private final Scheduler scheduler;

    Worker(Scheduler scheduler) {
        this.scheduler = scheduler;
    }

    NodeOutcome process(Job job, Node node) {
        // double-check the parent node's bound before we go any further; the best-known solution may have
        // improved since it was queued.
        if (node.bound() >= job.getBestKnown()) {
            return NodeOutcome.RESOLVED; // fathom the node.
        }
        var nodeModel = job.copyGlobalBoundsModel();
        node.vars().forEach((k, v) -> nodeModel.getVariable(k).level(v));

        var evaluation = weakUpdateBounds(job, nodeModel, node.depth());
        var nodeResult = evaluation.result();

        if (nodeResult.getState() == INFEASIBLE) {
            return NodeOutcome.RESOLVED;
        }
        if (!nodeResult.getState().isOptimal() || !evaluation.complete()) {
            return NodeOutcome.INCOMPLETE;
        }

        var nodeBound = roundBound(nodeResult.getValue(), job.maxScale());
        if (nodeBound >= job.getBestKnown()) {
            return NodeOutcome.RESOLVED;
        }

        var stream = IntStream.range(0, (int) nodeResult.count())
                .mapToObj(i -> {
                    var v = nodeResult.get(i);
                    return new AbstractMap.SimpleImmutableEntry<>(i,
                            v.setScale(0, RoundingMode.HALF_EVEN).subtract(v).abs());
                })
                .filter(entry -> entry.getValue().signum() > 0);

        // If we're in best-first mode, we want to branch on the fractional variable as close to 0.5 as possible and
        // with the largest possible coefficient in the objective function.
        // This should result in a more balanced search tree and more consistent runtime to solve the problem to
        // optimality.
        //
        // But if we're in depth-first mode, use the fractional variable closest to the nearest integer. I don't know...
        // this is the opposite of what most of the published algorithms are doing, but it seems to find interesting
        // local minima more quickly (a variable set to 0.9 in the LP relaxation, for example, seems likely to "want" to
        // be set to 1.0, so, try that first.) "Find local minima quickly" is our goal when we are in depth-first mode
        // because we don't necessarily expect to have time to solve the whole problem to optimality, but we want an
        // improved solution at least.
        var optional = job.isBestFirst() ?
                stream.max(comparing(entry ->
                        entry.getValue().multiply(nodeModel.getVariable(entry.getKey()).getContributionWeight()))) :
                stream.min(comparingByValue());

        if (optional.isPresent()) {
            var k = optional.get().getKey();
            var v = nodeResult.get(k);
            var closest = v.setScale(0, RoundingMode.HALF_EVEN);
            var variable = nodeModel.getVariable(k);
            var values = branchValues(variable.getLowerLimit(), variable.getUpperLimit());

            values.sort((left, right) -> {
                var byDistance = right.subtract(v).abs().compareTo(left.subtract(v).abs());
                if (byDistance != 0) {
                    return byDistance;
                }
                var byPreference = Boolean.compare(left.equals(closest), right.equals(closest));
                if (byPreference != 0) {
                    return byPreference;
                }
                return left.compareTo(right);
            });

            if (values.size() < 2) {
                return NodeOutcome.INCOMPLETE;
            }

            // Queue one child for every integer in the variable's domain. Depot-edge variables have domain
            // {0, 1, 2}, so a two-way exact-value split would not cover the complete search space.
            // Queue the closest value last, so it's on top of the stack (if it's a LIFO queue.)
            //
            // This is a bit subtle, and warrants some discussion: this ordering only seems to affect the algorithm
            // performance when we are in depth-first mode. It's a bit hard to tell; the signal-to-noise ratio is bad,
            // because ojAlgo's performance is highly variable: I suspect that due to multithreading and/or
            // randomization, it never finds the same results in the same amount of time (but that's not necessarily a
            // bug, because the problems we're feeding it often have multiple solutions with the same objective value,
            // especially in the LP relaxation.)
            //
            // When we're in best-first mode, we sort by the lower bound, rather than the fractional rounding gap; the
            // ordering of these child nodes does not affect that, but it does affect depth-first mode.
            scheduler.queueNodes(job, values.stream()
                    .map(value -> new Node(node, nodeBound, k, value))
                    .toArray(Node[]::new));
        } else {
            job.reportSolution(nodeResult);
        }
        return NodeOutcome.RESOLVED;
    }

    static List<BigDecimal> branchValues(BigDecimal lower, BigDecimal upper) {
        var values = new ArrayList<BigDecimal>();
        for (var value = lower.setScale(0, RoundingMode.CEILING);
             value.compareTo(upper) <= 0;
             value = value.add(ONE)) {
            values.add(value);
        }
        if (values.isEmpty()) {
            throw new IllegalArgumentException("Integer variable has an empty domain: [" + lower + ", " + upper + "]");
        }
        return values;
    }

    /**
     * If all costs are integer, the bound can be tightened by rounding up to the nearest integer.
     * (We generalize this to any level of precision.)
     * <p>
     * First round downward at a scale finer than the objective lattice, to prevent small positive solver noise from
     * raising the bound by a full objective unit. The downward step is conservative and does not discard significant
     * integer digits for large objectives.
     */
    static double roundBound(double lb, int scale) {
        return BigDecimal.valueOf(lb)
                .setScale(scale + NUMERICAL_GUARD_DIGITS, RoundingMode.FLOOR)
                .setScale(scale, RoundingMode.CEILING)
                .doubleValue();
    }

    static Optimisation.Result updateBounds(BigDecimal vehicleCapacity,
                                            BigDecimal[] demands,
                                            ExpressionsBasedModel model,
                                            Job job,
                                            long deadline,
                                            ExperimentalParameters parameters) {
        var result = minimize(model, deadline);
        var cuts = new HashSet<Set<Integer>>();
        var size = demands.length;
        var rccEnabled = parameters.rccMillis() > 0;

        while (result.getState().isOptimal()) {
            if (rccEnabled) {
                var rccCuts = RccSepCVRPCuts.generate(vehicleCapacity, demands, result,
                        parameters.rccDeadline(deadline));
                if (rccCuts == null) {
                    // The optional separator exhausted its budget. Continue with the cheap separator and the exact
                    // branch-and-bound search.
                    rccEnabled = false;
                } else if (addCuts(rccCuts, cuts, model, result, job, size) > 0) {
                    result = minimize(model, deadline);
                    continue;
                }
            }
            var subtourCuts = SubtourCuts.generate(vehicleCapacity, demands, result);

            if (addCuts(subtourCuts, cuts, model, result, job, size) > 0) {
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
     * {@link #updateBounds(BigDecimal, BigDecimal[], ExpressionsBasedModel, Job, long, ExperimentalParameters)},
     * but uses directly derived capacity cuts to reject invalid integer routes without solving another ILP.
     */
    private static NodeEvaluation weakUpdateBounds(Job job, ExpressionsBasedModel model, int depth) {
        var result = minimize(model, job.deadline());
        var cuts = new HashSet<Set<Integer>>();
        var rccEnabled = job.useRccAtDepth(depth);

        while (result.getState().isOptimal()) {
            // An incumbent can improve while this node is being solved. Once even the current, incompletely separated
            // relaxation cannot beat it, the node is proven fathomable and further cut generation is wasted work.
            if (boundFathoms(result.getValue(), job.getBestKnown(), job.maxScale())) {
                return new NodeEvaluation(result, true);
            }

            if (rccEnabled) {
                var rccCuts = RccSepCVRPCuts.generate(job.vehicleCapacity(), job.demands(), result,
                        job.rccDeadline());
                if (rccCuts == null) {
                    rccEnabled = false;
                } else if (job.addCuts(rccCuts, cuts, model, result) > 0) {
                    result = minimize(model, job.deadline());
                    continue;
                }
            }
            var subtourCuts = SubtourCuts.generate(job.vehicleCapacity(), job.demands(), result);

            if (job.addCuts(subtourCuts, cuts, model, result) > 0) {
                result = minimize(model, job.deadline());
                continue;
            }

            // If there are no fractional variables, validate the complete route structure and capacities before
            // accepting the candidate. An over-capacity route directly supplies a valid rounded-capacity cut.
            if (isIntegerSolution(result)) {
                var cycles = findCycles(job.demands().length, result);

                if (!isValidIntegerSolution(job.vehicleCapacity(), job.demands(), cycles)) {
                    var capacityCuts = capacityCuts(job.vehicleCapacity(), job.demands(), cycles);

                    if (job.addCuts(capacityCuts, cuts, model, result) > 0) {
                        result = minimize(model, job.deadline());
                        continue;
                    }

                    // The candidate is known to be invalid, but no new valid cut was installed. Do not report it as
                    // a solution or treat this node as proof-complete.
                    return new NodeEvaluation(result, false);
                }
            }

            if (job.useRccAtDepth(depth) && depth == job.getParameters().maxRccDepth()) {
                System.out.println("[" + toSeconds(System.currentTimeMillis() - job.getStart()) +
                        "s]: Found bound " + roundBound(result.getValue(), job.maxScale()) + " at depth " + depth);
            }

            return new NodeEvaluation(result, true);
        }

        return new NodeEvaluation(result, false); // infeasible or incomplete; process() distinguishes the state.
    }

    static boolean boundFathoms(double lowerBound, double incumbent, int scale) {
        return roundBound(lowerBound, scale) >= incumbent;
    }

    static Set<Cut> capacityCuts(BigDecimal vehicleCapacity,
                                 BigDecimal[] demands,
                                 Collection<List<Integer>> cycles) {
        var result = new HashSet<Cut>();

        for (var cycle : cycles) {
            var subset = cycle.stream().filter(node -> node != 0).collect(toSet());
            var totalDemand = subset.stream().map(node -> demands[node]).reduce(ZERO, BigDecimal::add);
            var requiredVehicles = minVehicles(vehicleCapacity, totalDemand);

            if (requiredVehicles > 1) {
                result.add(new Cut(requiredVehicles, subset));
            }
        }
        return result;
    }

    static boolean isValidIntegerSolution(BigDecimal vehicleCapacity,
                                          BigDecimal[] demands,
                                          Collection<List<Integer>> cycles) {
        var remaining = IntStream.range(1, demands.length).boxed().collect(toSet());

        for (var cycle : cycles) {
            if (cycle.size() < 2 || cycle.getFirst() != 0) {
                return false;
            }

            var totalDemand = ZERO;
            for (var i = 1; i < cycle.size(); i++) {
                var customer = cycle.get(i);
                if (customer <= 0 || customer >= demands.length || !remaining.remove(customer)) {
                    return false;
                }
                totalDemand = totalDemand.add(demands[customer]);
            }
            if (totalDemand.compareTo(vehicleCapacity) > 0) {
                return false;
            }
        }
        return remaining.isEmpty();
    }

    private record NodeEvaluation(Optimisation.Result result, boolean complete) {
    }

    private static boolean isIntegerSolution(Optimisation.Result result) {
        return LongStream.range(0, result.count()).allMatch(i -> {
            var v = result.get(i);
            return v.setScale(0, RoundingMode.HALF_EVEN).compareTo(v) == 0;
        });
    }
}

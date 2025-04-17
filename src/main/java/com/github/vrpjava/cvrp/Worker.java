package com.github.vrpjava.cvrp;

import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.AbstractMap;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.IntStream;
import java.util.stream.LongStream;

import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.findCycles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.minimize;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;
import static java.util.Comparator.comparing;
import static java.util.Map.Entry.comparingByValue;

/**
 * Multithreading is not implemented yet, but when it is, this class will contain all the worker-thread logic.
 */
final class Worker {
    private final Scheduler scheduler;

    Worker(Scheduler scheduler) {
        this.scheduler = scheduler;
    }

    void process(Job job, Node node) {
        // double-check the parent node's bound before we go any further; the best-known solution may have
        // improved since it was queued.
        if (node.bound() >= job.getBestKnown()) {
            return; // fathom the node.
        }
        var nodeModel = job.copyGlobalBoundsModel();
        node.vars().forEach((k, v) -> nodeModel.getVariable(k).level(v));

        var nodeResult = weakUpdateBounds(job, nodeModel);
        var nodeBound = roundBound(job, nodeResult.getValue());

        // if it's not optimal, it's probably INFEASIBLE (with nonsense variables), or a timeout.
        if (!nodeResult.getState().isOptimal() || nodeBound >= job.getBestKnown()) {
            return; // fathom the node.
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
            var other = closest.compareTo(v) > 0 ? closest.subtract(ONE) : closest.add(ONE);

            // Queue two child nodes with the decision variable fixed to `closest` and `other`.
            // Also, queue the closest gap last, so it's on top of stack (if it's a LIFO queue.)
            //
            // This is a bit subtle, and warrants some discussion: this ordering only seems to affect the algorithm
            // performance when we are in depth-first mode. It's a bit hard to tell; the signal-to-noise ratio is bad,
            // because ojAlgo's performance is highly variable: I suspect that due to multithreading and/or
            // randomization, it never finds the same results in the same amount of time (but that's not necessarily a
            // bug, because the problems we're feeding it often have multiple solutions with the same objective value,
            // especially in the LP relaxation.)
            //
            // When we're in best-first mode, we sort by the lower bound, rather than the fractional rounding gap; the
            // ordering of these two nodes does not affect that, but it does affect depth-first mode.
            scheduler.queueNodes(job, new Node(node, nodeBound, k, other), new Node(node, nodeBound, k, closest));
        } else {
            job.reportSolution(nodeResult);
        }
    }

    /**
     * If all costs are integer, the bound can be tightened by rounding up to the nearest integer.
     * (We generalize this to any level of precision.)
     */
    private static double roundBound(Job job, double lb) {
        return BigDecimal.valueOf(lb).setScale(job.maxScale(), RoundingMode.CEILING).doubleValue();
    }

    /**
     * Static for a reason: sometimes we pass job as null
     */
    static Optimisation.Result updateBounds(BigDecimal vehicleCapacity,
                                            BigDecimal[] demands,
                                            ExpressionsBasedModel model,
                                            Job job,
                                            long deadline) {
        var result = minimize(model, deadline);
        var cuts = new HashSet<Set<Integer>>();
        var size = demands.length;

        for (Set<OjAlgoCVRPSolver.Cut> rccCuts; result.getState().isOptimal() &&
                (rccCuts = RccSepCVRPCuts.generate(vehicleCapacity, demands, result, deadline)) != null; ) {
            if (Job.addCuts(rccCuts, cuts, model, result, job, size)) {
                result = minimize(model, deadline);
                continue;
            }
            var subtourCuts = SubtourCuts.generate(vehicleCapacity, demands, result);

            if (Job.addCuts(subtourCuts, cuts, model, result, job, size)) {
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
     * {@link #updateBounds(BigDecimal, BigDecimal[], ExpressionsBasedModel, Job, long)},
     * but does not solve the RCC-Sep model unless absolutely necessary.
     * (we need to check validity. when the node solution is integer)
     */
    private static Optimisation.Result weakUpdateBounds(Job job, ExpressionsBasedModel model) {
        var result = minimize(model, job.getDeadline());
        var cuts = new HashSet<Set<Integer>>();
        var size = job.getDemands().length;

        while (result.getState().isOptimal()) {
            var subtourCuts = SubtourCuts.generate(job.getVehicleCapacity(), job.getDemands(), result);

            if (Job.addCuts(subtourCuts, cuts, model, result, job, size)) {
                result = minimize(model, job.getDeadline());
                continue;
            }

            // No more cuts to add. Done?
            // If there are no fractional variables, this is a candidate solution, but we don't know for sure until
            // we've validated that it satisfies the full set of constraints, so iterate on additional cuts.
            // This needs to be done on a fast path, so don't run the RCC-Sep ILP model unless confirmed invalid.
            if (isInvalidIntegerSolution(job, result)) {
                var rccCuts = RccSepCVRPCuts.generate(job.getVehicleCapacity(), job.getDemands(), result,
                        job.getDeadline());

                if (rccCuts != null && Job.addCuts(rccCuts, cuts, model, result, job, size)) {
                    result = minimize(model, job.getDeadline());
                    continue;
                }
            }
            return result;
        }

        return result; // infeasible or timed out.
    }

    // Warning -- this does not check for sub-tours -- that's assumed to be handled elsewhere.
    private static boolean isInvalidIntegerSolution(Job job, Optimisation.Result result) {
        return isIntegerSolution(result) && findCycles(job.getDemands().length, result).stream().anyMatch(cycle ->
                cycle.stream().map(i -> job.getDemands()[i])
                        .reduce(ZERO, BigDecimal::add)
                        .compareTo(job.getVehicleCapacity()) > 0);
    }

    private static boolean isIntegerSolution(Optimisation.Result result) {
        return LongStream.range(0, result.count()).allMatch(i -> {
            var v = result.get(i);
            return v.setScale(0, RoundingMode.HALF_EVEN).compareTo(v) == 0;
        });
    }
}

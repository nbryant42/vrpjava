package com.github.vrpjava.cvrp;

import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.AbstractMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.stream.IntStream;
import java.util.stream.LongStream;

import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.addCuts;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.findCycles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.minimize;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;

final class Worker {
    private final Job job;

    public Worker(Job job) {
        this.job = job;
    }

    void process(Node node) {
        // double-check the parent node's bound before we go any further; the best-known solution may have
        // improved since it was queued.
        if (node.bound() >= job.getBestKnown()) {
            return; // fathom the node.
        }
        var nodeModel = job.copyGlobalBoundsModel();
        node.vars().forEach((k, v) -> nodeModel.getVariable(k).level(v));

        var nodeResult = weakUpdateBounds(nodeModel);
        var ub = nodeResult.getValue();

        // if it's not optimal, it's probably INFEASIBLE (with nonsense variables), or a timeout.
        if (!nodeResult.getState().isOptimal() || ub >= job.getBestKnown()) {
            return; // fathom the node.
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
            job.queueNode(node, ub, k, other, gap);
            job.queueNode(node, ub, k, closest, gap);
        } else {
            job.reportSolution(job.getGlobalBoundsResult(), nodeResult);
        }
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
            if (addCuts(rccCuts, cuts, model, result, job, size)) {
                result = minimize(model, deadline);
                continue;
            }
            var subtourCuts = SubtourCuts.generate(vehicleCapacity, demands, result);

            if (addCuts(subtourCuts, cuts, model, result, job, size)) {
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
    private Optimisation.Result weakUpdateBounds(ExpressionsBasedModel model) {
        var result = minimize(model, job.getDeadline());
        var cuts = new HashSet<Set<Integer>>();
        var size = job.getDemands().length;

        while (result.getState().isOptimal()) {
            var subtourCuts = SubtourCuts.generate(job.getVehicleCapacity(), job.getDemands(), result);

            if (addCuts(subtourCuts, cuts, model, result, job, size)) {
                result = minimize(model, job.getDeadline());
                continue;
            }

            // no more cuts to add. done.
            // if there are no fractional variables, this is a candidate solution, but we don't know for sure until
            // we've validated that it satisfies the full set of constraints, so iterate on additional cuts.
            // This needs to be done on a fast path, so don't run the RCC-Sep ILP model unless confirmed invalid.
            return isInvalidIntegerSolution(result) ?
                    updateBounds(job.getVehicleCapacity(), job.getDemands(), model, job, job.getDeadline()) :
                    result;
        }

        return result; // infeasible or timed out.
    }

    // Warning -- this does not check for sub-tours -- that's assumed to be handled elsewhere.
    private boolean isInvalidIntegerSolution(Optimisation.Result result) {
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

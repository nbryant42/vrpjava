package com.github.vrpjava.cvrp;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.IntStream;

import static java.lang.Math.max;
import static java.math.BigDecimal.ZERO;
import static java.util.stream.Collectors.toCollection;

/**
 * <p>
 * A fast, but very simple heuristic solver for the Capacitated Vehicle Routing Problem.
 * </p><p>
 * The solutions returned are not very good, but they are better than nothing.
 * </p><p>
 * Best used to provide the starting point for a better solver.
 * Used by {@link OjAlgoCVRPSolver} by default.
 * </p>
 *
 * @see OjAlgoCVRPSolver#setHeuristic(CVRPSolver)
 */
public class NearestNeighborCVRPSolver extends CVRPSolver {
    /**
     * Default constructor
     */
    public NearestNeighborCVRPSolver() {
    }

    private static class Cycle {
        private BigDecimal load = ZERO;
        private final List<Integer> nodes = new ArrayList<>();
        private boolean full;

        private Cycle() {
            nodes.add(0);
        }
    }

    @Override
    protected Result doSolve(int minVehicles,
                             int maxVehicles,
                             BigDecimal vehicleCapacity,
                             BigDecimal[] demands,
                             BigDecimal[][] costMatrix,
                             long timeout) {
        var remaining = IntStream.range(1, demands.length).boxed().collect(toCollection(HashSet::new));
        var objective = ZERO;

        minVehicles = max(minVehicles, minVehicles(vehicleCapacity, demands));

        var cycles = IntStream.range(0, minVehicles).mapToObj(i -> new Cycle()).collect(toCollection(ArrayList::new));

        for (List<Cycle> pending; !remaining.isEmpty() &&
                !(pending = cycles.stream().filter(c -> !c.full).toList()).isEmpty(); ) {
            for (var cycle : pending) {
                objective = objective.add(extend(cycle, vehicleCapacity, demands, costMatrix, remaining));
            }
        }

        while (!remaining.isEmpty()) {
            var cycle = new Cycle();
            do {
                objective = objective.add(extend(cycle, vehicleCapacity, demands, costMatrix, remaining));
            } while (!cycle.full);
            cycles.add(cycle);
        }

        for (var cycle : cycles) {
            objective = objective.add(costMatrix[cycle.nodes.getLast()][0]);
        }

        return cycles.size() > maxVehicles ?
                new Result(Result.State.ERROR, Double.POSITIVE_INFINITY, List.of()) :
                new Result(Result.State.HEURISTIC, objective.doubleValue(), cycles.stream().map(c -> c.nodes).toList());
    }

    private static BigDecimal extend(Cycle cycle,
                                     BigDecimal vehicleCapacity,
                                     BigDecimal[] demands,
                                     BigDecimal[][] costMatrix,
                                     HashSet<Integer> remaining) {
        var nodes = cycle.nodes;
        var load = cycle.load;
        var node = nodes.getLast();
        var next = findNearestNeighbor(vehicleCapacity, demands, costMatrix, node, load, remaining);
        if (next > 0) {
            remaining.remove(next);
            nodes.add(next);
            cycle.load = load.add(demands[next]);
            return lookup(node, next, costMatrix);
        } else {
            cycle.full = true;
            return ZERO;
        }
    }

    // Find the nearest neighbor that satisfies all the constraints.
    // Remove it from `remaining` and return it.
    private static int findNearestNeighbor(BigDecimal vehicleCapacity,
                                           BigDecimal[] demands,
                                           BigDecimal[][] costMatrix,
                                           int node,
                                           BigDecimal load,
                                           Set<Integer> remaining) {
        var rval = remaining.stream().filter(i -> load.add(demands[i]).compareTo(vehicleCapacity) <= 0)
                .min(Comparator.comparing(candidate -> lookup(node, candidate, costMatrix)))
                .orElse(0);
        remaining.remove(rval);
        return rval;
    }

    private static BigDecimal lookup(int i, int j, BigDecimal[][] costMatrix) {
        return i > j ? costMatrix[i][j] : costMatrix[j][i];
    }
}
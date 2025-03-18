package com.github.vrpjava.cvrp;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.stream.IntStream;

import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
import static java.util.stream.Collectors.toSet;

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

    @Override
    protected Result doSolve(int minVehicles,
                             int maxVehicles,
                             BigDecimal vehicleCapacity,
                             BigDecimal[] demands,
                             BigDecimal[][] costMatrix,
                             long timeout) {
        var cycles = new ArrayList<List<Integer>>();
        var remaining = IntStream.range(1, demands.length).boxed().collect(toSet());
        var objective = ZERO;

        outerLoop:
        do {
            var cycle = new ArrayList<Integer>();
            var node = 0;
            var load = ZERO;

            cycle.add(0);
            while (true) {
                if (remaining.size() < minVehicles - cycles.size()) {
                    objective = objective.add(lookup(0, cycle.getLast(), costMatrix));
                    cycles.add(cycle);
                    break outerLoop;
                }

                var next = findNearestNeighbor(vehicleCapacity, demands, costMatrix, node, load, remaining);

                if (next <= 0) {
                    break;
                }

                cycle.add(next);
                objective = objective.add(lookup(node, next, costMatrix));
                load = load.add(demands[next]);
            }

            objective = objective.add(lookup(0, cycle.getLast(), costMatrix));
            cycles.add(cycle);
        } while (!remaining.isEmpty());

        // these were left over to satisfy `minVehicles`
        for (var r : remaining) {
            var c = new ArrayList<Integer>();
            c.add(0);
            c.add(r);
            cycles.add(c);
            objective = objective.add(lookup(0, r, costMatrix).multiply(TWO));
        }

        return cycles.size() > maxVehicles ?
                new Result(Result.State.ERROR, Double.POSITIVE_INFINITY, List.of()) :
                new Result(Result.State.HEURISTIC, objective.doubleValue(), cycles);
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
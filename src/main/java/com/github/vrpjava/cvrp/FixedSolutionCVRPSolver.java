package com.github.vrpjava.cvrp;

import java.math.BigDecimal;
import java.util.Collection;
import java.util.List;
import java.util.Set;

import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.HEURISTIC;
import static java.math.BigDecimal.ZERO;

/**
 * Supplies configured routes as a feasible starting solution for {@link OjAlgoCVRPSolver}.
 * Routes are copied on construction and validated against each instance when solving. Their objective is computed
 * from that instance's costs; supplying routes makes no claim of optimality.
 *
 * @see OjAlgoCVRPSolver#setHeuristic(CVRPSolver)
 */
public final class FixedSolutionCVRPSolver extends CVRPSolver {
    private final List<List<Integer>> cycles;

    /**
     * @param cycles routes starting at depot {@code 0}, with at least one customer and no repeated closing depot
     * @throws NullPointerException if the collection, any route, or any vertex is null
     */
    public FixedSolutionCVRPSolver(Collection<? extends List<Integer>> cycles) {
        // Retain duplicate routes until validation, rather than silently collapsing them into a set.
        this.cycles = cycles.stream().map(List::copyOf).toList();
    }

    /**
     * Validates the configured solution and recomputes its cost, even when the time budget is zero.
     *
     * @throws IllegalArgumentException if the routes do not cover every customer exactly once, have invalid depot
     *                                  placement or vertex indexes, exceed capacity, or use too few vehicles
     */
    @Override
    protected Result doSolve(int minVehicles, BigDecimal vehicleCapacity, BigDecimal[] demands,
                             BigDecimal[][] costMatrix, long timeout) {
        if (cycles.size() < minVehicles) {
            throw new IllegalArgumentException("Configured solution uses fewer than minVehicles routes.");
        }
        var visited = new boolean[demands.length];
        for (var cycle : cycles) {
            if (cycle.size() < 2 || cycle.getFirst() != 0) {
                throw new IllegalArgumentException("Each configured route must start at depot 0 and contain a customer.");
            }
            var load = ZERO;
            for (var i = 1; i < cycle.size(); i++) {
                var vertex = cycle.get(i);
                if (vertex <= 0 || vertex >= demands.length) {
                    throw new IllegalArgumentException("Invalid customer index in configured route: " + vertex);
                }
                if (visited[vertex]) {
                    throw new IllegalArgumentException("Customer occurs more than once in configured routes: " + vertex);
                }
                visited[vertex] = true;
                load = load.add(demands[vertex]);
            }
            if (load.compareTo(vehicleCapacity) > 0) {
                throw new IllegalArgumentException("Configured route exceeds vehicle capacity: " + cycle);
            }
        }
        for (var vertex = 1; vertex < visited.length; vertex++) {
            if (!visited[vertex]) {
                throw new IllegalArgumentException("Configured routes omit customer: " + vertex);
            }
        }
        return new Result(HEURISTIC, costMatrix, Set.copyOf(cycles));
    }
}

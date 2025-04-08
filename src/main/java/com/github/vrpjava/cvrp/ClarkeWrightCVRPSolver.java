package com.github.vrpjava.cvrp;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static java.util.stream.Collectors.toSet;

/**
 * <p>
 * An implementation of the Clarke-Wright savings-based algorithm for the CVRP.
 * </p><p>
 * This implementation relies on the assumption that cycles can be reversed,
 * so it's probably only suited for the symmetric problem.
 * </p>
 */
public class ClarkeWrightCVRPSolver extends CVRPSolver {
    /**
     * Default constructor.
     */
    public ClarkeWrightCVRPSolver() {
    }

    private record Savings(int i, int j, BigDecimal savings) implements Comparable<Savings> {
        /**
         * Compares in reverse order based on {@link #savings}
         */
        @Override
        public int compareTo(Savings o) {
            return o.savings.compareTo(savings);
        }
    }

    private static final class Cycle {
        private List<Integer> nodes;
        private BigDecimal load;

        private Cycle(List<Integer> nodes, BigDecimal load) {
            this.nodes = nodes;
            this.load = load;
        }

        private List<Integer> nodes() {
            return nodes;
        }
    }

    @Override
    protected Result doSolve(int minVehicles, BigDecimal vehicleCapacity, BigDecimal[] demands,
                             BigDecimal[][] costMatrix, long timeout) {
        var size = demands.length;
        var cycles = new HashSet<Cycle>();
        var customersToCycles = new Cycle[size];

        // (1) Compute savings, then sort best-first
        var savings = computeSavings(costMatrix, size);

        // (2) Initialize the starting solution, which is `n` vehicles
        // all serving a single customer, then returning to the depot.
        initSolution(demands, size, cycles, customersToCycles);

        // (3) loop over all the sorted savings and merge cycles if it wouldn't violate constraints.
        for (var iterator = savings.iterator(); iterator.hasNext() && cycles.size() > minVehicles; ) {
            var s = iterator.next();
            maybeMergeCycles(s.i, s.j, vehicleCapacity, customersToCycles, cycles);
        }

        // (4) convert solution to the expected format and return.
        return new Result(Result.State.HEURISTIC, costMatrix, cycles.stream().map(cycle -> {
            List<Integer> nodes = cycle.nodes();
            nodes.addFirst(0);
            return nodes;
        }).collect(toSet()));
    }

    private static List<Savings> computeSavings(BigDecimal[][] costMatrix, int size) {
        var savings = new ArrayList<Savings>();

        for (int i = 1; i < size; i++) {
            for (int j = 1; j < i; j++) {
                var s = costMatrix[i][0].add(costMatrix[j][0]).subtract(costMatrix[i][j]);

                if (s.signum() > 0) {
                    savings.add(new Savings(i, j, s));
                }
            }
        }
        savings.sort(null);
        return savings;
    }

    private static void initSolution(BigDecimal[] demands, int size, Set<Cycle> cycles, Cycle[] customersToCycles) {
        for (int i = 1; i < size; i++) {
            var nodes = new ArrayList<Integer>();
            nodes.add(i); // we add the depots in the translation phase (step 4.)

            var cycle = new Cycle(nodes, demands[i]);
            cycles.add(cycle);
            customersToCycles[i] = cycle;
        }
    }

    private static void maybeMergeCycles(int i, int j, BigDecimal vehicleCapacity, Cycle[] customersToCycles,
                                         Set<Cycle> cycles) {
        var iCycle = customersToCycles[i];
        var jCycle = customersToCycles[j];
        var newLoad = iCycle.load.add(jCycle.load);

        if (iCycle == jCycle || newLoad.compareTo(vehicleCapacity) > 0) {
            return;
        }

        if (iCycle.nodes.getFirst() == i) {
            if (jCycle.nodes.getFirst() == j) {
                // Both cycles start with the nodes we're linking. Reverse I and append J to it.
                iCycle.nodes = new ArrayList<>(iCycle.nodes.reversed());
                iCycle.nodes.addAll(jCycle.nodes);
                mergeAndDrop(newLoad, iCycle, jCycle, customersToCycles, cycles);
            } else if (jCycle.nodes.getLast() == j) {
                // iCycle starts with i, jCycle ends with j; concat iCycle to J
                jCycle.nodes.addAll(iCycle.nodes);
                mergeAndDrop(newLoad, jCycle, iCycle, customersToCycles, cycles);
            } // else J is an interior node; can't merge
        } else if (iCycle.nodes.getLast() == i) {
            if (jCycle.nodes.getFirst() == j) {
                // iCycle ends with i, jCycle starts with j; concat jCycle to I.
                iCycle.nodes.addAll(jCycle.nodes);
                mergeAndDrop(newLoad, iCycle, jCycle, customersToCycles, cycles);
            } else if (jCycle.nodes.getLast() == j) {
                // Both cycles end with the nodes we're linking. Reverse I and append it to J.
                jCycle.nodes.addAll(iCycle.nodes.reversed());
                mergeAndDrop(newLoad, jCycle, iCycle, customersToCycles, cycles);
            } // else J is an interior node; can't merge
        } // else I is an interior node; can't merge
    }

    private static void mergeAndDrop(BigDecimal newLoad, Cycle toMerge, Cycle toDrop, Cycle[] customersToCycles,
                                     Set<Cycle> cycles) {
        toMerge.load = newLoad;
        toDrop.nodes.forEach(cust -> customersToCycles[cust] = toMerge);
        cycles.remove(toDrop);
    }
}

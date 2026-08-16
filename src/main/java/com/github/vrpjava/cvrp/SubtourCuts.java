package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.IntStream;

import static com.github.vrpjava.cvrp.CVRPSolver.minVehicles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.findFractionalCycles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.getVariable_noFlip;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;

class SubtourCuts {
    static List<Cut> generate(BigDecimal vehicleCapacity,
                              BigDecimal[] demands,
                              Optimisation.Result result) {
        var components = findFractionalCycles(demands.length, result)
                .stream()
                .filter(component -> !component.contains(0))
                .toList();

        if (!components.isEmpty()) {
            return components.stream().map(component -> cut(vehicleCapacity, demands, component)).toList();
        }
        if (demands.length < 2) {
            return List.of();
        }

        var minCut = StoerWagnerMinimumCut.find(edgeWeights(demands.length, result));
        if (minCut.weight().compareTo(TWO) >= 0) {
            return List.of();
        }

        var subset = customerSide(demands.length, minCut.vertices());
        return subset.isEmpty() ? List.of() : List.of(cut(vehicleCapacity, demands, subset));
    }

    private static Cut cut(BigDecimal vehicleCapacity,
                           BigDecimal[] demands,
                           Collection<Integer> subset) {
        var totalDemand = subset.stream().map(node -> demands[node]).reduce(ZERO, BigDecimal::add);
        return new Cut(minVehicles(vehicleCapacity, totalDemand), new HashSet<>(subset));
    }

    private static Set<Integer> customerSide(int size, Set<Integer> minCutSide) {
        if (!minCutSide.contains(0)) {
            return new HashSet<>(minCutSide);
        }

        var complement = new HashSet<Integer>();
        IntStream.range(1, size).filter(vertex -> !minCutSide.contains(vertex)).forEach(complement::add);
        return complement;
    }

    private static BigDecimal[][] edgeWeights(int size, Optimisation.Result result) {
        var weights = new BigDecimal[size][size];
        for (var row : weights) {
            Arrays.fill(row, ZERO);
        }

        for (var row = 1; row < size; row++) {
            for (var col = 0; col < row; col++) {
                // The mathematical variables are nonnegative. Ignore any tiny negative solver artefact rather than
                // passing an invalid edge weight to Stoer-Wagner.
                var weight = getVariable_noFlip(row, col, result).max(ZERO);
                weights[row][col] = weight;
                weights[col][row] = weight;
            }
        }
        return weights;
    }

    static String formatCut(Collection<Integer> subset) {
        return "cut: " + subset.stream().sorted().filter(t -> t != 0).toList();
    }
}

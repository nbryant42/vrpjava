package org.example;

import org.example.OjAlgoCVRPSolver.Cut;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.util.Collection;
import java.util.List;

import static java.math.BigDecimal.ZERO;
import static org.example.CVRPSolver.minVehicles;
import static org.example.OjAlgoCVRPSolver.findFractionalCycles;

class SubtourCuts {
    static List<Cut> generate(BigDecimal vehicleCapacity,
                              BigDecimal[] demands,
                              Optimisation.Result result) {

        return findFractionalCycles(demands.length, result)
                .stream()
                .filter(cycle -> !cycle.contains(0))
                .map(cycle -> new Cut(
                        minVehicles(vehicleCapacity,
                                cycle.stream().map(node -> demands[node]).reduce(ZERO, BigDecimal::add)),
                        cycle))
                .toList();
    }

    static List<Integer> formatCut(Collection<Integer> subset) {
        return subset.stream().sorted().filter(t -> t != 0).toList();
    }
}

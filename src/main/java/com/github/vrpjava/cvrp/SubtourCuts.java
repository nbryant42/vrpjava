package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.util.Collection;
import java.util.List;

import static java.math.BigDecimal.ZERO;
import static com.github.vrpjava.cvrp.CVRPSolver.minVehicles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.findFractionalCycles;

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

    static String formatCut(Collection<Integer> subset) {
        return "cut: " + subset.stream().sorted().filter(t -> t != 0).toList();
    }
}

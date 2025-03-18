package com.github.vrpjava.cvrp;

import java.math.BigDecimal;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static com.github.vrpjava.Util.isLowerTriangular;
import static com.github.vrpjava.Util.newModel;
import static com.github.vrpjava.Util.setTimeout;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.addCuts;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildConstraints;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildVars;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.findCycles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.minimize;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.toSeconds;
import static java.math.BigDecimal.ZERO;
import static org.ojalgo.optimisation.Optimisation.State.OPTIMAL;

/**
 * <p>
 * This class is mainly of academic interest (unlike the rest of this package!)
 * </p><p>
 * Basically, it's the minimally simple version of the RCC-Sep iterative procedure, but at every iteration it solves to
 * optimality instead of using the LP relaxation. It was a key step in developing the full solver, and at times it may
 * get to the solution quicker, but for the most part it's not very practical because it may fail to find any solution
 * at all within a reasonable timeframe.
 * </p>
 */
@Deprecated
class IterativeRowGenCVRPSolver extends CVRPSolver {
    protected Result doSolve(int minVehicles,
                             int maxVehicles,
                             BigDecimal vehicleCapacity,
                             BigDecimal[] demands,
                             BigDecimal[][] costMatrix,
                             long timeout) {
        if (!isLowerTriangular(costMatrix)) {
            throw new UnsupportedOperationException("costMatrix must be lower-triangular.");
        }
        var start = System.currentTimeMillis();
        var deadline = start + timeout;
        var model = newModel(deadline);
        var vars = buildVars(costMatrix, model);
        var cuts = new HashSet<Set<Integer>>();

        buildConstraints(model, minVehicles, maxVehicles, vars);

        // solve

        var result = minimize(model.snapshot());

        for (var iter = 1; result.getState().isOptimal() && System.currentTimeMillis() < deadline; iter++) {
            setTimeout(deadline, model.options);
            var size = costMatrix.length;
            var rccCuts = RccSepCVRPCuts.generate(vehicleCapacity, demands, result, deadline);

            if (addCuts(rccCuts, cuts, model, result, null, vars.length)) {
                result = minimize(model.snapshot());
                continue;
            }

            var subtourCuts = SubtourCuts.generate(vehicleCapacity, demands, result);

            if (addCuts(subtourCuts, cuts, model, result, null, vars.length)) {
                result = minimize(model.snapshot());
                continue;
            }

            // no more cuts to add. done.

            List<List<Integer>> cycles;
            try {
                cycles = findCycles(size, result);
            } catch (IllegalArgumentException e) {
                var relaxed = result.getValue();
                var substart = System.currentTimeMillis();
                result = minimize(model);
                var now = System.currentTimeMillis();
                System.out.println("[" + toSeconds(now - start) + "s] " + toSeconds(now - substart) +
                        "s discrete re-solve. LP bound: " + (float) relaxed + ", ILP: " +
                        (float) result.getValue());
                continue;
            }

            System.out.println(iter + " iterations, " + cuts.size() + " cuts, " + cycles.size() + " cycles: " + cycles);
            var cycleDemands = cycles.stream()
                    .map(cycle -> cycle.stream().map(i -> demands[i]).reduce(ZERO, BigDecimal::add))
                    .toList();
            System.out.println("Cost " + result.getValue() + ", cycle demands: " + cycleDemands);

            return new Result(result.getValue(), cycles);
        }

        return null;
    }
}

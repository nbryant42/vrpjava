package com.github.vrpjava.cvrp;

import org.ojalgo.netio.BasicLogger;

import java.math.BigDecimal;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static com.github.vrpjava.Util.isLowerTriangular;
import static com.github.vrpjava.Util.newModel;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.addCuts;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildConstraints;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildVars;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.findCycles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.minimize;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.toSeconds;
import static java.math.BigDecimal.ZERO;

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
@SuppressWarnings("DeprecatedIsStillUsed")
@Deprecated
class IterativeRowGenCVRPSolver extends CVRPSolver {
    private boolean debug;

    protected Result doSolve(int minVehicles,
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

        buildConstraints(model, minVehicles, vars);

        // solve

        var result = minimize(model.snapshot(), deadline);

        for (var iter = 1; result.getState().isOptimal() && System.currentTimeMillis() < deadline; iter++) {
            var size = costMatrix.length;
            var rccCuts = RccSepCVRPCuts.generate(vehicleCapacity, demands, result, deadline);

            if (rccCuts == null) {
                return new Result(Result.State.ERROR, Double.POSITIVE_INFINITY, Set.of());
            }
            if (addCuts(rccCuts, cuts, model, result, null, vars.length)) {
                result = minimize(model.snapshot(), deadline);
                continue;
            }

            var subtourCuts = SubtourCuts.generate(vehicleCapacity, demands, result);

            if (addCuts(subtourCuts, cuts, model, result, null, vars.length)) {
                result = minimize(model.snapshot(), deadline);
                continue;
            }

            // no more cuts to add. done.

            Set<List<Integer>> cycles;
            try {
                cycles = findCycles(size, result);
            } catch (IllegalArgumentException e) {
                var relaxed = result.getValue();
                var substart = System.currentTimeMillis();
                result = minimize(model, deadline);
                var now = System.currentTimeMillis();
                debug("[" + toSeconds(now - start) + "s] " + toSeconds(now - substart) +
                        "s discrete re-solve. LP bound: " + (float) relaxed + ", ILP: " +
                        (float) result.getValue());
                continue;
            }

            debug(iter + " iterations, " + cuts.size() + " cuts, " + cycles.size() + " cycles: " + cycles);
            var cycleDemands = cycles.stream()
                    .map(cycle -> cycle.stream().map(i -> demands[i]).reduce(ZERO, BigDecimal::add))
                    .toList();
            debug("Cost " + result.getValue() + ", cycle demands: " + cycleDemands);

            return new Result(Result.State.OPTIMAL, result.getValue(), cycles);
        }

        return null;
    }

    private void debug(String s) {
        if (debug) {
            BasicLogger.debug(s);
        }
    }

    /**
     * Get the debug property
     *
     * @return true if debug logging is enabled
     */
    @SuppressWarnings("unused")
    public boolean isDebug() {
        return debug;
    }

    /**
     * Set the debug property. If enabled, logging works via ojAlgo's {@link BasicLogger} mechanism.
     * You can supply a thin wrapper implementation to redirect it to the logging library of your choice.
     *
     * @param debug true if debug logging is enabled
     */
    @SuppressWarnings("unused")
    public void setDebug(boolean debug) {
        this.debug = debug;
    }
}

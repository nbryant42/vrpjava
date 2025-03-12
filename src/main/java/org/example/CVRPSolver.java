package org.example;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.List;

import static java.lang.Math.max;
import static java.math.BigDecimal.ZERO;
import static java.util.Arrays.stream;
import static java.util.stream.IntStream.range;

/**
 * Solver for the Capacitated Vehicle Routing Problem (CVRP).
 * <p>
 * Assumes that there is one depot, which must be represented as index 0 in the
 * cost matrix. <code>demands</code> should have the same dimension as
 * <code>costMatrix</code>, so the depot will also be at index 0 in the
 * demands, and must be set to 0.
 * <p>
 * The number of vehicles is not held constant unless
 * <code>minVehicles == maxVehicles</code>, so as a side effect of cost
 * minimization, the number of vehicles used will also be minimized.
 */
public abstract class CVRPSolver {
    public record Result(double objective, List<List<Integer>> cycles) {
    }

    public final Result solve(int minVehicles,
                              int maxVehicles,
                              BigDecimal vehicleCapacity,
                              BigDecimal[] demands,
                              BigDecimal[][] costMatrix) {
        return solve(minVehicles, maxVehicles, vehicleCapacity, demands, costMatrix, 1000L * 60L * 60L);
    }

    public final Result solve(int minVehicles,
                              int maxVehicles,
                              BigDecimal vehicleCapacity,
                              BigDecimal[] demands,
                              BigDecimal[][] costMatrix,
                              long timeoutMillis) {
        var size = demands.length;

        if (minVehicles <= 0) {
            throw new IllegalArgumentException("maxVehicles must be a positive integer.");
        }
        if (maxVehicles <= 0) {
            throw new IllegalArgumentException("maxVehicles must be a positive integer.");
        }
        stream(demands).filter(d -> d.compareTo(vehicleCapacity) > 0).forEach(d -> {
            throw new IllegalArgumentException("all demands must be <= vehicleCapacity.");
        });
        minVehicles = max(minVehicles, minVehicles(vehicleCapacity, demands));
        if (minVehicles > maxVehicles) {
            throw new IllegalArgumentException("maxVehicles must be at least " + minVehicles + ".");
        }
        System.out.println("minVehicles = " + minVehicles);
        if (vehicleCapacity.signum() <= 0) {
            throw new IllegalArgumentException("vehicleCapacity must be positive.");
        }
        Util.validate(costMatrix);
        if (size != costMatrix.length) {
            throw new IllegalArgumentException("demands and costMatrix should have the same dimension.");
        }
        if (demands[0].signum() != 0) {
            throw new IllegalArgumentException("demands[0] should be zero.");
        }
        range(1, size).filter(i -> demands[i].signum() <= 0).forEach(i -> {
            throw new IllegalArgumentException("demands must be positive.");
        });

        return doSolve(minVehicles, maxVehicles, vehicleCapacity, demands, costMatrix, timeoutMillis);
    }

    /**
     * Given <code>vehicleCapacity</code>, return the minimum number of vehicles required to
     * satisfy <code>demands</code>.
     * <p>
     * An exact minimum requires solving a bin-packing sub-problem, so this is just an approximate lower bound.
     */
    public static int minVehicles(BigDecimal vehicleCapacity, BigDecimal[] demands) {
        return minVehicles(vehicleCapacity, stream(demands).reduce(ZERO, BigDecimal::add));
    }

    public static int minVehicles(BigDecimal vehicleCapacity, BigDecimal totalDemands) {
        return totalDemands.divide(vehicleCapacity, 0, RoundingMode.CEILING).intValueExact();
    }

    protected abstract Result doSolve(int minVehicles,
                                      int maxVehicles,
                                      BigDecimal vehicleCapacity,
                                      BigDecimal[] demands,
                                      BigDecimal[][] costMatrix,
                                      long timeout);
}

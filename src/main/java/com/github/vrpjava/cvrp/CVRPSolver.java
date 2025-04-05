package com.github.vrpjava.cvrp;

import com.github.vrpjava.Util;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.List;
import java.util.Set;

import static com.github.vrpjava.Util.lookup;
import static java.lang.Math.max;
import static java.math.BigDecimal.ZERO;
import static java.util.Arrays.stream;
import static java.util.stream.IntStream.range;

/**
 * Abstract superclass of solvers for the Capacitated Vehicle Routing Problem (CVRP).
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
    /**
     * Default constructor.
     */
    protected CVRPSolver() {
    }

    /**
     * Record type to hold a solution to the problem.
     *
     * @param state     reports status, whether {@link State#OPTIMAL}, {@link State#FEASIBLE}, etc.
     * @param objective the total travel cost (or distance) of the solution
     * @param cycles    a list of "cycles" for each vehicle, so called because they represent a Hamiltonian Cycle.
     *                  Each cycle is a list of locations visited by the vehicle, represented as integer array indexes
     *                  into the corresponding problem spec, starting with zero (the depot.)
     */
    public record Result(State state, double objective, Set<List<Integer>> cycles) {
        /**
         * Convenience constructor to compute the objective based on the costs.
         *
         * @param state      reports status, whether {@link State#OPTIMAL}, {@link State#FEASIBLE}, etc.
         * @param costMatrix the travel distances as a lower-triangular matrix
         * @param cycles     a list of "cycles" for each vehicle, so called because they represent a Hamiltonian Cycle.
         *                   Each cycle is a list of locations visited by the vehicle, represented as integer array
         *                   indexes into the corresponding problem spec, starting with zero (the depot.)
         */
        public Result(State state, BigDecimal[][] costMatrix, Set<List<Integer>> cycles) {
            this(state, totalCosts(costMatrix, cycles), cycles);
        }

        private static double totalCosts(BigDecimal[][] costMatrix, Set<List<Integer>> cycles) {
            var obj = ZERO;

            for (var cycle : cycles) {
                var prev = cycle.getFirst();

                for (int i = 1; i < cycle.size(); i++) {
                    var cur = cycle.get(i);

                    obj = obj.add(lookup(prev, cur, costMatrix));
                    prev = cur;
                }
                obj = obj.add(costMatrix[prev][0]);
            }
            return obj.doubleValue();
        }

        /**
         * These states generally have about the same meaning as in {@link Optimisation.State}, except we have an
         * additional {@link #HEURISTIC} status which would be equivalent to {@link Optimisation.State#FEASIBLE}
         */
        public enum State {
            /**
             * The heuristic couldn't find a solution,
             * and we timed out before even starting the branch-and-bound search.
             */
            UNEXPLORED(false),
            /**
             * The heuristic found a feasible solution, but branch-and-bound found nothing better.
             */
            HEURISTIC(true),
            /**
             * Branch-and-bound found a solution, but it may not be optimal.
             */
            FEASIBLE(true),
            /**
             * We solved the problem to optimality!
             */
            OPTIMAL(true),
            /**
             * The problem is believed to be infeasible.
             */
            INFEASIBLE(false),
            /**
             * The heuristic couldn't find a solution, and neither could branch-and-bound
             * (probably timeout, possibly infeasible)
             */
            ERROR(false);

            private final boolean feasible;

            State(boolean feasible) {
                this.feasible = feasible;
            }

            /**
             * {@link #HEURISTIC} or {@link #FEASIBLE} are both considered feasible solutions.
             *
             * @return true if this state is one of these.
             */
            public boolean isFeasible() {
                return feasible;
            }
        }
    }

    /**
     * Equivalent to <code>solve(minVehicles, maxVehicles, vehicleCapacity, demands, costMatrix,
     * 1000L * 60L * 60L)</code>
     *
     * @param minVehicles     Require a solution using at least this many vehicles.
     *                        Typically set to 1.
     * @param vehicleCapacity The vehicle capacity limit, in units of <code>demands</code>
     * @param demands         Array of customer demands. Index zero is the depot, so should be set to zero.
     * @param costMatrix      A lower-triangular matrix of distances between locations. Column zero is the depot.
     * @return the solution, which may be exact or approximate
     * @see #solve(int, BigDecimal, BigDecimal[], BigDecimal[][], long)
     */
    @SuppressWarnings("unused")
    public final Result solve(int minVehicles,
                              BigDecimal vehicleCapacity,
                              BigDecimal[] demands,
                              BigDecimal[][] costMatrix) {
        return solve(minVehicles, vehicleCapacity, demands, costMatrix, 1000L * 60L * 60L);
    }

    /**
     * Solve the symmetric Capacitated Vehicle Routing Problem (CVRP)
     *
     * @param minVehicles     Require a solution using at least this many vehicles.
     *                        Typically set to 1.
     * @param vehicleCapacity The vehicle capacity limit, in units of <code>demands</code>
     * @param demands         Array of customer demands. Index zero is the depot, so should be set to zero.
     * @param costMatrix      A lower-triangular matrix of distances between locations. Column zero is the depot.
     * @param timeoutMillis   Maximum wall-clock time in milliseconds.
     * @return the solution, which may be exact or approximate
     */
    public final Result solve(int minVehicles,
                              BigDecimal vehicleCapacity,
                              BigDecimal[] demands,
                              BigDecimal[][] costMatrix,
                              long timeoutMillis) {
        var size = demands.length;

        if (minVehicles <= 0) {
            throw new IllegalArgumentException("maxVehicles must be a positive integer.");
        }
        stream(demands).filter(d -> d.compareTo(vehicleCapacity) > 0).forEach(d -> {
            throw new IllegalArgumentException("all demands must be <= vehicleCapacity.");
        });
        minVehicles = max(minVehicles, minVehicles(vehicleCapacity, demands));
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

        return doSolve(minVehicles, vehicleCapacity, demands, costMatrix, timeoutMillis);
    }

    /**
     * <p>
     * Given <code>vehicleCapacity</code>, return the minimum number of vehicles required to
     * satisfy <code>demands</code>.
     * </p><p>
     * An exact minimum requires solving a bin-packing sub-problem, so this is just an approximate lower bound.
     * </p>
     *
     * @param vehicleCapacity maximum vehicle capacity in <code>demands</code> units
     * @param demands         array of customer demands
     * @return an integer
     */
    public static int minVehicles(BigDecimal vehicleCapacity, BigDecimal[] demands) {
        return minVehicles(vehicleCapacity, stream(demands).reduce(ZERO, BigDecimal::add));
    }

    /**
     * <p>
     * Given <code>vehicleCapacity</code>, return the minimum number of vehicles required to
     * satisfy <code>totalDemands</code>.
     * </p><p>
     * An exact minimum requires solving a bin-packing sub-problem, so this is just an approximate lower bound.
     * </p>
     *
     * @param vehicleCapacity maximum vehicle capacity in <code>totalDemands</code> units
     * @param totalDemands    total customer demands
     * @return an integer
     */
    public static int minVehicles(BigDecimal vehicleCapacity, BigDecimal totalDemands) {
        return totalDemands.divide(vehicleCapacity, 0, RoundingMode.CEILING).intValueExact();
    }

    /**
     * Must be implemented by subclasses. Called by
     * {@link #solve(int, BigDecimal, BigDecimal[], BigDecimal[][], long)} after performing parameter validations.
     *
     * @param minVehicles     require a solution using at least this many vehicles.
     *                        Typically set to 1.
     * @param vehicleCapacity The vehicle capacity limit, in units of <code>demands</code>
     * @param demands         Array of customer demands. Index zero is the depot, so should be set to zero.
     * @param costMatrix      A lower-triangular matrix of distances between locations. Column zero is the depot.
     * @param timeout         Maximum wall-clock time in milliseconds. The implementation should do its best to observe
     *                        this.
     * @return the solution, which may be exact or approximate
     */
    protected abstract Result doSolve(int minVehicles,
                                      BigDecimal vehicleCapacity,
                                      BigDecimal[] demands,
                                      BigDecimal[][] costMatrix,
                                      long timeout);
}

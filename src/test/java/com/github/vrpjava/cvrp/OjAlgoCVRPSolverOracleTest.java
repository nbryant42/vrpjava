package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.math.BigDecimal;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import static com.github.vrpjava.Util.lookup;
import static com.github.vrpjava.Util.setUpHardware_raptorLake;
import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.OPTIMAL;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.ExperimentalParameters;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.SearchStrategy.BEST_FIRST;
import static java.math.BigDecimal.ZERO;
import static java.math.BigDecimal.valueOf;
import static java.util.stream.IntStream.range;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class OjAlgoCVRPSolverOracleTest {
    private static final long SEED = 0x5EED_C0DEL;

    @BeforeAll
    static void setUp() {
        setUpHardware_raptorLake();
    }

    @Test
    void agreesWithBruteForceOnTinyInstances() {
        var random = new Random(SEED);
        var problems = range(0, 6).mapToObj(ignored -> randomProblem(random, 5)).toList();

        try (var solver = new OjAlgoCVRPSolver()) {
            assertAll("seeded tiny CVRP instances", range(0, problems.size()).mapToObj(caseNumber -> () -> {
                var problem = problems.get(caseNumber);
                var minVehicles = 1 + caseNumber % 2;
                var expected = bruteForce(problem, minVehicles);
                var actual = solver.solve(minVehicles, problem.capacity(), problem.demands(), problem.costs(), 30_000L);
                var message = describe(caseNumber, problem);

                assertFeasible(problem, minVehicles, actual, message);
                assertEquals(expected.doubleValue(), actual.objective(), message);
                assertEquals(OPTIMAL, actual.state(), message);
            }));
        }
    }

    @Test
    void reportsOptimalWhenTheHeuristicIncumbentIsProven() {
        var demands = new BigDecimal[]{ZERO, valueOf(1), valueOf(1)};
        var costs = new BigDecimal[][]{
                {ZERO, ZERO, ZERO},
                {valueOf(1), ZERO, ZERO},
                {valueOf(1), valueOf(1), ZERO}};

        try (var solver = new OjAlgoCVRPSolver()) {
            var statistics = new AtomicReference<OjAlgoCVRPSolver.SolveStatistics>();
            var parameters = new ExperimentalParameters(BEST_FIRST, 1, 0, 0L, 0.85, 30_000L);
            solver.setExperimentalParameters(parameters);
            solver.setStatisticsConsumer(statistics::set);
            var actual = solver.solve(2, valueOf(2), demands, costs, 10_000L);
            var stats = statistics.get();

            assertEquals(4.0, actual.objective());
            assertEquals(OPTIMAL, actual.state());
            assertNotNull(stats);
            assertEquals(parameters, stats.parameters());
            assertTrue(stats.rootState().isOptimal());
            assertEquals(4.0, stats.rootBound());
            assertTrue(stats.elapsedMillis() >= 0L);
        }
    }

    private static Problem randomProblem(Random random, int size) {
        var capacity = valueOf(6);
        var demands = new BigDecimal[size];
        demands[0] = ZERO;
        for (var i = 1; i < size; i++) {
            demands[i] = valueOf(1 + random.nextInt(4));
        }

        var costs = new BigDecimal[size][size];
        for (var row = 0; row < size; row++) {
            for (var col = 0; col < size; col++) {
                costs[row][col] = row <= col ? ZERO : valueOf(1 + random.nextInt(20));
            }
        }
        return new Problem(capacity, demands, costs);
    }

    private static BigDecimal bruteForce(Problem problem, int minVehicles) {
        var order = new int[problem.demands().length - 1];
        for (var i = 0; i < order.length; i++) {
            order[i] = i + 1;
        }

        var best = new BigDecimal[1];
        permute(problem, minVehicles, order, 0, best);
        return best[0];
    }

    private static void permute(Problem problem, int minVehicles, int[] order, int index, BigDecimal[] best) {
        if (index == order.length) {
            evaluatePartitions(problem, minVehicles, order, best);
            return;
        }

        for (var i = index; i < order.length; i++) {
            swap(order, index, i);
            permute(problem, minVehicles, order, index + 1, best);
            swap(order, index, i);
        }
    }

    private static void evaluatePartitions(Problem problem, int minVehicles, int[] order, BigDecimal[] best) {
        var gapCount = order.length - 1;
        for (var separators = 0; separators < 1 << gapCount; separators++) {
            if (Integer.bitCount(separators) + 1 < minVehicles) {
                continue;
            }

            var load = ZERO;
            var cost = ZERO;
            var previous = 0;
            var feasible = true;

            for (var i = 0; i < order.length; i++) {
                var customer = order[i];
                load = load.add(problem.demands()[customer]);
                if (load.compareTo(problem.capacity()) > 0) {
                    feasible = false;
                    break;
                }

                cost = cost.add(lookup(previous, customer, problem.costs()));
                previous = customer;
                if (i == order.length - 1 || (separators & 1 << i) != 0) {
                    cost = cost.add(lookup(previous, 0, problem.costs()));
                    load = ZERO;
                    previous = 0;
                }
            }

            if (feasible && (best[0] == null || cost.compareTo(best[0]) < 0)) {
                best[0] = cost;
            }
        }
    }

    private static void assertFeasible(Problem problem, int minVehicles, CVRPSolver.Result result, String message) {
        assertTrue(result.cycles().size() >= minVehicles, message);

        var customers = new HashSet<Integer>();
        var objective = ZERO;
        for (var cycle : result.cycles()) {
            assertEquals(0, cycle.getFirst(), message);
            assertTrue(cycle.size() > 1, message + ": empty route " + cycle);
            var load = ZERO;
            var previous = 0;
            for (var i = 1; i < cycle.size(); i++) {
                var customer = cycle.get(i);
                assertTrue(customer > 0 && customer < problem.demands().length, message);
                assertTrue(customers.add(customer), message + ": duplicate customer " + customer);
                load = load.add(problem.demands()[customer]);
                objective = objective.add(lookup(previous, customer, problem.costs()));
                previous = customer;
            }
            assertTrue(load.compareTo(problem.capacity()) <= 0, message + ": overloaded route " + cycle);
            objective = objective.add(lookup(previous, 0, problem.costs()));
        }

        assertEquals(problem.demands().length - 1, customers.size(), message);
        assertEquals(objective.doubleValue(), result.objective(), message);
    }

    private static String describe(int caseNumber, Problem problem) {
        return "seed=" + Long.toHexString(SEED) + ", case=" + caseNumber +
                ", demands=" + Arrays.toString(problem.demands()) +
                ", costs=" + Arrays.deepToString(problem.costs());
    }

    private static void swap(int[] values, int left, int right) {
        var value = values[left];
        values[left] = values[right];
        values[right] = value;
    }

    private record Problem(BigDecimal capacity, BigDecimal[] demands, BigDecimal[][] costs) {
    }
}

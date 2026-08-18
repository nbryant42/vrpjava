package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.Test;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static com.github.vrpjava.cvrp.ThreeToothCuts.Edge;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.ojalgo.optimisation.Optimisation.State.OPTIMAL;

class ThreeToothCutsTest {
    private static final BigDecimal CAPACITY = BigDecimal.valueOf(6);
    private static final BigDecimal[] UNIT_DEMANDS = {ZERO, ONE, ONE, ONE, ONE, ONE, ONE};

    @Test
    void findsTheExhaustiveOptimumBeyondTheRciClosure() {
        var base = violatedPoint();
        var permutations = List.of(
                new int[]{0, 1, 2, 3, 4, 5, 6},
                new int[]{0, 2, 3, 1, 5, 6, 4},
                new int[]{0, 6, 4, 5, 3, 1, 2},
                new int[]{0, 4, 1, 5, 2, 6, 3});

        for (var permutation : permutations) {
            var edges = permute(base, permutation);
            assertCustomerDegrees(edges);
            assertRciClosure(edges);
            var parent = result(edges);
            var expected = exhaustiveBestViolation(CAPACITY, UNIT_DEMANDS, parent);

            var separation = ThreeToothCuts.generate(CAPACITY, UNIT_DEMANDS, parent, Long.MAX_VALUE);

            assertTrue(separation.complete());
            assertTrue(separation.cut().isPresent());
            var cut = separation.cut().orElseThrow();
            assertTrue(ThreeToothCuts.isStructurallyValid(cut, CAPACITY, UNIT_DEMANDS));
            assertEquals(0, expected.compareTo(ThreeToothCuts.violation(cut, parent)));
            assertEquals(0, BigDecimal.valueOf(0.5).compareTo(expected));
        }
    }

    @Test
    void agreesWithTheOracleOnMixedPointsAndNonuniformDemands() {
        var base = violatedPoint();
        var tightTour = tightTour();

        for (var baseWeight : new double[]{0.25, 0.5, 0.75}) {
            var edges = mix(base, tightTour, baseWeight);
            assertCustomerDegrees(edges);
            assertRciClosure(edges);
            assertMatchesOracle(CAPACITY, UNIT_DEMANDS, edges, BigDecimal.valueOf(baseWeight * 0.5));
        }

        var nonuniformDemands = new BigDecimal[]{ZERO, ONE, ONE, ONE,
                BigDecimal.valueOf(2), BigDecimal.valueOf(2), BigDecimal.valueOf(2)};
        assertMatchesOracle(BigDecimal.valueOf(3), nonuniformDemands, base, BigDecimal.valueOf(0.5));
    }

    @Test
    void reportsNoCutAtAnIntegralTour() {
        var edges = graph(7);
        edge(edges, 0, 1, 1.0);
        edge(edges, 1, 2, 1.0);
        edge(edges, 2, 3, 1.0);
        edge(edges, 3, 4, 1.0);
        edge(edges, 4, 5, 1.0);
        edge(edges, 5, 6, 1.0);
        edge(edges, 6, 0, 1.0);
        var parent = result(edges);

        var separation = ThreeToothCuts.generate(CAPACITY, UNIT_DEMANDS, parent, Long.MAX_VALUE);

        assertTrue(separation.complete());
        assertTrue(separation.cut().isEmpty());
        assertTrue(exhaustiveBestViolation(CAPACITY, UNIT_DEMANDS, parent).signum() <= 0);
    }

    @Test
    void completesWithoutAModelWhenThreeDisjointTeethAreImpossible() {
        var demands = new BigDecimal[]{ZERO, ONE, ONE, ONE, ONE, ONE};
        var parent = result(graph(demands.length));

        var separation = ThreeToothCuts.generate(CAPACITY, demands, parent, System.currentTimeMillis());

        assertTrue(separation.complete());
        assertTrue(separation.cut().isEmpty());
    }

    @Test
    void anExpiredBudgetIsIncompleteRatherThanExhausted() {
        var separation = ThreeToothCuts.generate(CAPACITY, UNIT_DEMANDS, result(violatedPoint()),
                System.currentTimeMillis() - 1L);

        assertFalse(separation.complete());
        assertTrue(separation.cut().isEmpty());
    }

    @Test
    void restrictedCutsHoldForEveryTinyFeasibleRouting() {
        var capacity = BigDecimal.valueOf(3);
        var candidates = exhaustiveCuts(capacity, UNIT_DEMANDS);
        assertEquals(120, candidates.size());

        var customers = new int[]{1, 2, 3, 4, 5, 6};
        var permutations = new ArrayList<int[]>();
        permutations(customers, 0, permutations);
        var checked = 0;

        for (var permutation : permutations) {
            for (var breaks = 0; breaks < 1 << (customers.length - 1); breaks++) {
                if (!respectsCapacity(permutation, breaks, 3)) {
                    continue;
                }
                var routing = result(routing(permutation, breaks));
                for (var cut : candidates) {
                    assertTrue(ThreeToothCuts.violation(cut, routing).signum() <= 0);
                }
                checked++;
            }
        }
        assertEquals(17_280, checked);
    }

    @Test
    void installedExpressionRejectsTheSeparatedPoint() {
        var parent = result(violatedPoint());
        var cut = ThreeToothCuts.generate(CAPACITY, UNIT_DEMANDS, parent, Long.MAX_VALUE).cut().orElseThrow();
        var model = new ExpressionsBasedModel();
        for (var index = 0; index < parent.count(); index++) {
            model.newVariable("x" + index);
        }

        ThreeToothCuts.addTo(model, cut);

        assertFalse(model.validate(parent));
    }

    private static double[][] violatedPoint() {
        var edges = graph(7);
        edge(edges, 1, 2, 0.5);
        edge(edges, 1, 3, 0.5);
        edge(edges, 2, 3, 0.5);
        edge(edges, 1, 4, 1.0);
        edge(edges, 2, 5, 1.0);
        edge(edges, 3, 6, 1.0);
        edge(edges, 0, 4, 1.0);
        edge(edges, 0, 5, 1.0);
        edge(edges, 0, 6, 1.0);
        return edges;
    }

    private static double[][] tightTour() {
        var edges = graph(7);
        edge(edges, 0, 4, 1.0);
        edge(edges, 4, 1, 1.0);
        edge(edges, 1, 2, 1.0);
        edge(edges, 2, 5, 1.0);
        edge(edges, 5, 6, 1.0);
        edge(edges, 6, 3, 1.0);
        edge(edges, 3, 0, 1.0);
        return edges;
    }

    private static double[][] mix(double[][] first, double[][] second, double firstWeight) {
        var result = graph(first.length);
        for (var row = 0; row < first.length; row++) {
            for (var col = 0; col < row; col++) {
                edge(result, row, col,
                        firstWeight * first[row][col] + (1.0 - firstWeight) * second[row][col]);
            }
        }
        return result;
    }

    private static void assertMatchesOracle(BigDecimal vehicleCapacity,
                                            BigDecimal[] demands,
                                            double[][] edges,
                                            BigDecimal expectedViolation) {
        var parent = result(edges);
        var expected = exhaustiveBestViolation(vehicleCapacity, demands, parent);
        var separation = ThreeToothCuts.generate(vehicleCapacity, demands, parent, Long.MAX_VALUE);

        assertTrue(separation.complete());
        var cut = separation.cut().orElseThrow();
        assertTrue(ThreeToothCuts.isStructurallyValid(cut, vehicleCapacity, demands));
        assertEquals(0, expected.compareTo(ThreeToothCuts.violation(cut, parent)));
        assertEquals(0, expectedViolation.compareTo(expected));
    }

    private static void assertCustomerDegrees(double[][] edges) {
        for (var customer = 1; customer < edges.length; customer++) {
            var degree = ZERO;
            for (var other = 0; other < edges.length; other++) {
                degree = degree.add(BigDecimal.valueOf(edges[customer][other]));
            }
            assertEquals(0, BigDecimal.valueOf(2).compareTo(degree));
        }
    }

    private static void assertRciClosure(double[][] edges) {
        var customerCount = edges.length - 1;
        for (var mask = 1; mask < 1 << customerCount; mask++) {
            var boundary = ZERO;
            for (var first = 1; first < edges.length; first++) {
                if ((mask & 1 << (first - 1)) == 0) {
                    continue;
                }
                for (var second = 0; second < edges.length; second++) {
                    if (second == 0 || (mask & 1 << (second - 1)) == 0) {
                        boundary = boundary.add(BigDecimal.valueOf(edges[first][second]));
                    }
                }
            }
            var checkedMask = mask;
            var checkedBoundary = boundary;
            assertTrue(checkedBoundary.compareTo(BigDecimal.valueOf(2)) >= 0,
                    () -> "RCI violation for customer mask " + checkedMask + ": " + checkedBoundary);
        }
    }

    private static BigDecimal exhaustiveBestViolation(BigDecimal vehicleCapacity,
                                                       BigDecimal[] demands,
                                                       Optimisation.Result result) {
        return exhaustiveCuts(vehicleCapacity, demands).stream()
                .map(cut -> ThreeToothCuts.violation(cut, result))
                .max(BigDecimal::compareTo)
                .orElseThrow();
    }

    private static List<ThreeToothCuts.Cut> exhaustiveCuts(BigDecimal vehicleCapacity, BigDecimal[] demands) {
        var customerCount = demands.length - 1;
        var result = new ArrayList<ThreeToothCuts.Cut>();

        for (var mask = 1; mask < 1 << customerCount; mask++) {
            var handle = new HashSet<Integer>();
            var handleDemand = ZERO;
            for (var vertex = 1; vertex < demands.length; vertex++) {
                if ((mask & 1 << (vertex - 1)) != 0) {
                    handle.add(vertex);
                    handleDemand = handleDemand.add(demands[vertex]);
                }
            }
            if (handleDemand.compareTo(vehicleCapacity) > 0) {
                continue;
            }

            var eligible = new ArrayList<Edge>();
            for (var row = 2; row < demands.length; row++) {
                for (var col = 1; col < row; col++) {
                    if (handle.contains(row) != handle.contains(col) &&
                            demands[row].add(demands[col]).compareTo(vehicleCapacity) <= 0) {
                        eligible.add(new Edge(row, col));
                    }
                }
            }

            for (var first = 0; first < eligible.size(); first++) {
                for (var second = first + 1; second < eligible.size(); second++) {
                    for (var third = second + 1; third < eligible.size(); third++) {
                        var teeth = Set.of(eligible.get(first), eligible.get(second), eligible.get(third));
                        var endpoints = new HashSet<Integer>();
                        if (teeth.stream().allMatch(edge ->
                                endpoints.add(edge.first()) && endpoints.add(edge.second()))) {
                            result.add(new ThreeToothCuts.Cut(handle, teeth));
                        }
                    }
                }
            }
        }
        return result;
    }

    private static boolean respectsCapacity(int[] permutation, int breaks, int maxRouteSize) {
        var routeSize = 0;
        for (var index = 0; index < permutation.length; index++) {
            routeSize++;
            if (index == permutation.length - 1 || (breaks & 1 << index) != 0) {
                if (routeSize > maxRouteSize) {
                    return false;
                }
                routeSize = 0;
            }
        }
        return true;
    }

    private static double[][] routing(int[] permutation, int breaks) {
        var edges = graph(permutation.length + 1);
        var routeStart = 0;
        for (var index = 0; index < permutation.length; index++) {
            if (index == routeStart) {
                addEdge(edges, 0, permutation[index], 1.0);
            } else {
                addEdge(edges, permutation[index - 1], permutation[index], 1.0);
            }
            if (index == permutation.length - 1 || (breaks & 1 << index) != 0) {
                addEdge(edges, permutation[index], 0, 1.0);
                routeStart = index + 1;
            }
        }
        return edges;
    }

    private static void permutations(int[] values, int index, List<int[]> result) {
        if (index == values.length) {
            result.add(values.clone());
            return;
        }
        for (var next = index; next < values.length; next++) {
            var swap = values[index];
            values[index] = values[next];
            values[next] = swap;
            permutations(values, index + 1, result);
            swap = values[index];
            values[index] = values[next];
            values[next] = swap;
        }
    }

    private static double[][] permute(double[][] edges, int[] permutation) {
        var result = graph(edges.length);
        for (var row = 0; row < edges.length; row++) {
            for (var col = 0; col < row; col++) {
                edge(result, permutation[row], permutation[col], edges[row][col]);
            }
        }
        return result;
    }

    private static double[][] graph(int size) {
        return new double[size][size];
    }

    private static void edge(double[][] edges, int first, int second, double weight) {
        edges[first][second] = weight;
        edges[second][first] = weight;
    }

    private static void addEdge(double[][] edges, int first, int second, double weight) {
        edges[first][second] += weight;
        edges[second][first] += weight;
    }

    private static Optimisation.Result result(double[][] edges) {
        var values = new double[edges.length * (edges.length - 1) / 2];
        var index = 0;
        for (var row = 1; row < edges.length; row++) {
            for (var col = 0; col < row; col++) {
                values[index++] = edges[row][col];
            }
        }
        return Optimisation.Result.of(OPTIMAL, values);
    }
}

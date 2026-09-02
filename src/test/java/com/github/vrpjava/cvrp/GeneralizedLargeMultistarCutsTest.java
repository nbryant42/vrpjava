package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.Test;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.ojalgo.optimisation.Optimisation.State.OPTIMAL;

class GeneralizedLargeMultistarCutsTest {
    private static final BigDecimal TEN = BigDecimal.TEN;

    @Test
    void minimumCutMatchesExhaustiveSeparation() {
        var demands = demands(4, 4, 4, 4);
        var parent = result(overloadedTour());
        var expected = exhaustiveBestViolation(TEN, demands, parent);

        var separation = GeneralizedLargeMultistarCuts.generate(TEN, demands, parent, Long.MAX_VALUE);

        assertFalse(separation.timedOut());
        assertEquals(3, separation.cuts().size());
        for (var cut : separation.cuts()) {
            assertTrue(GeneralizedLargeMultistarCuts.isStructurallyValid(cut, demands));
            assertEquals(0, expected.compareTo(
                    GeneralizedLargeMultistarCuts.violation(cut, TEN, demands, parent)));
        }
        assertEquals(0, BigDecimal.valueOf(6).compareTo(expected));
    }

    @Test
    void minimumCutMatchesExhaustiveSeparationWithCapacityIncompatibleEdgesFixedToZero() {
        var random = new Random(0x61_4D_5E_7AL);

        for (var customerCount = 2; customerCount <= 7; customerCount++) {
            for (var trial = 0; trial < 20; trial++) {
                var demands = new BigDecimal[customerCount + 1];
                demands[0] = ZERO;
                for (var customer = 1; customer <= customerCount; customer++) {
                    demands[customer] = BigDecimal.valueOf(customer <= 2 ? 6 : 1 + random.nextInt(9));
                }
                var parent = result(randomDegreeFeasiblePoint(customerCount, demands, TEN, random));
                var expected = exhaustiveBestViolation(TEN, demands, parent);
                var separation = GeneralizedLargeMultistarCuts.generate(TEN, demands, parent, Long.MAX_VALUE);

                assertFalse(separation.timedOut());
                if (expected.compareTo(TEN.multiply(new BigDecimal("0.000001"))) > 0) {
                    assertFalse(separation.cuts().isEmpty());
                    for (var actual : separation.cuts()) {
                        assertEquals(0, expected.compareTo(
                                        GeneralizedLargeMultistarCuts.violation(actual, TEN, demands, parent)),
                                "best violation for customers=" + customerCount + ", trial=" + trial);
                    }
                } else {
                    assertTrue(separation.cuts().isEmpty());
                }
            }
        }
    }

    @Test
    void separatesAnOverloadedTourWithoutUsingCapacityIncompatibleEdges() {
        var demands = demands(6, 6, 4, 4);
        var edges = graph(5);
        edge(edges, 0, 1, 1.0);
        edge(edges, 1, 3, 1.0);
        edge(edges, 3, 4, 1.0);
        edge(edges, 4, 2, 1.0);
        edge(edges, 2, 0, 1.0);
        var parent = result(edges);
        var expected = exhaustiveBestViolation(TEN, demands, parent);

        var separation = GeneralizedLargeMultistarCuts.generate(TEN, demands, parent, Long.MAX_VALUE);

        assertTrue(expected.compareTo(TEN.multiply(new BigDecimal("0.000001"))) > 0);
        assertFalse(separation.cuts().isEmpty());
        for (var cut : separation.cuts()) {
            assertEquals(0, expected.compareTo(
                    GeneralizedLargeMultistarCuts.violation(cut, TEN, demands, parent)));
        }
    }

    @Test
    void usesTheOutsideEndpointsDemandOnCrossingEdges() {
        var demands = demands(1, 1, 9, 9);
        var edges = graph(5);
        edge(edges, 1, 2, 1.0);
        edge(edges, 1, 3, 0.5);
        edge(edges, 2, 4, 0.5);
        var cut = new GeneralizedLargeMultistarCuts.Cut(Set.of(1, 2));

        assertEquals(0, ONE.compareTo(
                GeneralizedLargeMultistarCuts.violation(cut, TEN, demands, result(edges))));
    }

    @Test
    void installedExpressionRejectsTheSeparatedPoint() {
        var demands = demands(4, 4, 4, 4);
        var parent = result(overloadedTour());
        var cut = GeneralizedLargeMultistarCuts.generate(TEN, demands, parent, Long.MAX_VALUE)
                .cuts().getFirst();
        var model = model(parent);

        GeneralizedLargeMultistarCuts.addTo(model, cut, TEN, demands);

        assertFalse(model.validate(parent));
    }

    @Test
    void everyTinyFeasibleRoutingSatisfiesEveryNucleus() {
        var capacity = BigDecimal.valueOf(7);
        var demands = demands(1, 2, 3, 4, 5);
        var customers = new int[]{1, 2, 3, 4, 5};
        var permutations = new ArrayList<int[]>();
        permutations(customers, 0, permutations);
        var cuts = allCuts(customers.length);
        var emptyRouting = result(graph(customers.length + 1));
        var installedCuts = cuts.stream().map(cut -> {
            var model = model(emptyRouting);
            GeneralizedLargeMultistarCuts.addTo(model, cut, capacity, demands);
            return new InstalledCut(cut, model);
        }).toList();
        var checked = 0;

        for (var permutation : permutations) {
            for (var breaks = 0; breaks < 1 << (customers.length - 1); breaks++) {
                if (!respectsCapacity(permutation, breaks, capacity, demands)) {
                    continue;
                }
                var routing = result(routing(permutation, breaks));
                for (var installed : installedCuts) {
                    assertTrue(GeneralizedLargeMultistarCuts.violation(
                                    installed.cut(), capacity, demands, routing).signum() <= 0,
                            () -> "Direct inequality violated by " + installed.cut() + " at " + routing);
                    assertTrue(installed.model().validate(routing),
                            () -> "Installed inequality violated by " + installed.cut() + " at " + routing);
                }
                checked++;
            }
        }
        assertEquals(792, checked);
    }

    @Test
    void expiredBudgetReturnsNoCut() {
        var separation = GeneralizedLargeMultistarCuts.generate(TEN, demands(4, 4, 4, 4),
                result(overloadedTour()), System.currentTimeMillis() - 1L);

        assertTrue(separation.timedOut());
        assertTrue(separation.cuts().isEmpty());
    }

    @Test
    void baseModelEliminatesCapacityIncompatibleCustomerEdges() {
        var capacity = BigDecimal.valueOf(10);
        var demands = demands(6, 4, 5);
        var costs = new BigDecimal[][]{
                {ZERO, ZERO, ZERO, ZERO},
                {ONE, ZERO, ZERO, ZERO},
                {ONE, ONE, ZERO, ZERO},
                {ONE, ONE, ONE, ZERO}
        };
        var model = new ExpressionsBasedModel();

        var variables = OjAlgoCVRPSolver.buildVars(capacity, demands, costs, model);

        assertEquals(0, ONE.compareTo(variables[2][1].getUpperLimit()));
        assertEquals(0, ZERO.compareTo(variables[3][1].getUpperLimit()));
        assertEquals(0, ONE.compareTo(variables[3][2].getUpperLimit()));
    }

    private static BigDecimal exhaustiveBestViolation(BigDecimal capacity,
                                                       BigDecimal[] demands,
                                                       Optimisation.Result parent) {
        return allCuts(demands.length - 1).stream()
                .map(cut -> GeneralizedLargeMultistarCuts.violation(cut, capacity, demands, parent))
                .max(BigDecimal::compareTo)
                .orElseThrow();
    }

    private static List<GeneralizedLargeMultistarCuts.Cut> allCuts(int customerCount) {
        var cuts = new ArrayList<GeneralizedLargeMultistarCuts.Cut>();
        for (var mask = 1; mask < 1 << customerCount; mask++) {
            var nucleus = new HashSet<Integer>();
            for (var vertex = 1; vertex <= customerCount; vertex++) {
                if ((mask & 1 << (vertex - 1)) != 0) {
                    nucleus.add(vertex);
                }
            }
            cuts.add(new GeneralizedLargeMultistarCuts.Cut(nucleus));
        }
        return cuts;
    }

    private static double[][] overloadedTour() {
        var edges = graph(5);
        edge(edges, 0, 3, 1.0);
        edge(edges, 3, 1, 1.0);
        edge(edges, 1, 2, 1.0);
        edge(edges, 2, 4, 1.0);
        edge(edges, 4, 0, 1.0);
        return edges;
    }

    private static double[][] randomDegreeFeasiblePoint(int customerCount,
                                                         BigDecimal[] demands,
                                                         BigDecimal capacity,
                                                         Random random) {
        var result = graph(customerCount + 1);
        var weights = new double[]{0.2, 0.3, 0.5};
        for (var weight : weights) {
            var customers = new ArrayList<Integer>();
            for (var customer = 1; customer <= customerCount; customer++) {
                customers.add(customer);
            }
            Collections.shuffle(customers, random);
            var permutation = customers.stream().mapToInt(Integer::intValue).toArray();
            var breaks = randomFeasibleBreaks(permutation, demands, capacity, random);
            var routing = routing(permutation, breaks);
            for (var row = 1; row < routing.length; row++) {
                for (var col = 0; col < row; col++) {
                    addEdge(result, row, col, weight * routing[row][col]);
                }
            }
        }
        return result;
    }

    private static int randomFeasibleBreaks(int[] permutation,
                                             BigDecimal[] demands,
                                             BigDecimal capacity,
                                             Random random) {
        var breaks = 0;
        var routeDemand = ZERO;
        for (var index = 0; index < permutation.length; index++) {
            var demand = demands[permutation[index]];
            if (routeDemand.signum() > 0 && routeDemand.add(demand).compareTo(capacity) > 0) {
                breaks |= 1 << (index - 1);
                routeDemand = ZERO;
            }
            routeDemand = routeDemand.add(demand);
            if (index < permutation.length - 1 && random.nextInt(4) == 0) {
                breaks |= 1 << index;
                routeDemand = ZERO;
            }
        }
        return breaks;
    }

    private static boolean respectsCapacity(int[] permutation,
                                            int breaks,
                                            BigDecimal capacity,
                                            BigDecimal[] demands) {
        var routeDemand = ZERO;
        for (var index = 0; index < permutation.length; index++) {
            routeDemand = routeDemand.add(demands[permutation[index]]);
            if (index == permutation.length - 1 || (breaks & 1 << index) != 0) {
                if (routeDemand.compareTo(capacity) > 0) {
                    return false;
                }
                routeDemand = ZERO;
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

    private static BigDecimal[] demands(int... values) {
        var result = new BigDecimal[values.length + 1];
        result[0] = ZERO;
        for (var index = 0; index < values.length; index++) {
            result[index + 1] = BigDecimal.valueOf(values[index]);
        }
        return result;
    }

    private static ExpressionsBasedModel model(Optimisation.Result parent) {
        var model = new ExpressionsBasedModel();
        for (var index = 0; index < parent.count(); index++) {
            model.newVariable("x" + index);
        }
        return model;
    }

    private static double[][] graph(int size) {
        return new double[size][size];
    }

    private static void edge(double[][] edges, int first, int second, double value) {
        edges[first][second] = value;
        edges[second][first] = value;
    }

    private static void addEdge(double[][] edges, int first, int second, double value) {
        edges[first][second] += value;
        edges[second][first] += value;
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

    private record InstalledCut(GeneralizedLargeMultistarCuts.Cut cut, ExpressionsBasedModel model) {
    }
}

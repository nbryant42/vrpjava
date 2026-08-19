package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.Test;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.ojalgo.optimisation.Optimisation.State.OPTIMAL;

class HeuristicCombCutsTest {
    private static final BigDecimal SIX = BigDecimal.valueOf(6);
    private static final BigDecimal[] UNIT_DEMANDS = {ZERO, ONE, ONE, ONE, ONE, ONE, ONE};

    @Test
    void findsAnOrdinaryCombFromAFractionalSupportBlock() {
        var parent = result(ordinaryCombPoint());

        var separation = HeuristicCombCuts.generate(SIX, UNIT_DEMANDS, parent, Long.MAX_VALUE);

        assertFalse(separation.timedOut());
        assertFalse(separation.cuts().isEmpty());
        assertTrue(separation.cuts().stream().allMatch(cut ->
                HeuristicCombCuts.isStructurallyValid(cut, SIX, UNIT_DEMANDS) &&
                        HeuristicCombCuts.violation(cut, parent).signum() > 0));
        assertTrue(separation.cuts().stream().anyMatch(cut ->
                cut.handle().equals(Set.of(1, 2, 3)) &&
                        cut.teeth().equals(Set.of(Set.of(1, 4), Set.of(2, 5), Set.of(3, 6))) &&
                        cut.rightHandSide() == 10));
    }

    @Test
    void greedilyEnlargesATightOrdinaryCombIntoAViolatedStrengthenedComb() {
        var demands = new BigDecimal[]{ZERO, ONE, ONE, ONE, ONE, ONE, ONE, SIX};
        var parent = result(enlargedCombPoint());

        var separation = HeuristicCombCuts.generate(SIX, demands, parent, Long.MAX_VALUE);

        assertFalse(separation.timedOut());
        var cut = separation.cuts().stream()
                .filter(candidate -> candidate.handle().equals(Set.of(1, 2, 3)))
                .filter(candidate -> candidate.teeth().contains(Set.of(1, 4, 7)))
                .findFirst().orElseThrow();
        assertEquals(12, cut.rightHandSide());
        assertEquals(0, BigDecimal.valueOf(2).compareTo(HeuristicCombCuts.violation(cut, parent)));
        assertTrue(HeuristicCombCuts.isStructurallyValid(cut, SIX, demands));

        var model = model(parent);
        HeuristicCombCuts.addTo(model, cut, demands.length);
        assertFalse(model.validate(parent));
    }

    @Test
    void reportsNoCutAtAnIntegralTour() {
        var edges = graph(7);
        for (var vertex = 0; vertex < 7; vertex++) {
            edge(edges, vertex, (vertex + 1) % 7, 1.0);
        }

        var separation = HeuristicCombCuts.generate(SIX, UNIT_DEMANDS, result(edges), Long.MAX_VALUE);

        assertFalse(separation.timedOut());
        assertTrue(separation.cuts().isEmpty());
    }

    @Test
    void expiredBudgetReturnsNoUnvalidatedCuts() {
        var separation = HeuristicCombCuts.generate(SIX, UNIT_DEMANDS, result(ordinaryCombPoint()),
                System.currentTimeMillis() - 1L);

        assertTrue(separation.timedOut());
        assertTrue(separation.cuts().isEmpty());
    }

    @Test
    void installedExpressionRejectsTheSeparatedPoint() {
        var parent = result(ordinaryCombPoint());
        var cut = HeuristicCombCuts.generate(SIX, UNIT_DEMANDS, parent, Long.MAX_VALUE)
                .cuts().getFirst();
        var model = model(parent);

        HeuristicCombCuts.addTo(model, cut, UNIT_DEMANDS.length);

        assertFalse(model.validate(parent));
    }

    private static ExpressionsBasedModel model(Optimisation.Result parent) {
        var model = new ExpressionsBasedModel();
        for (var index = 0; index < parent.count(); index++) {
            model.newVariable("x" + index);
        }
        return model;
    }

    @Test
    void enlargedCutHoldsForEveryTinyFeasibleRouting() {
        var demands = new BigDecimal[]{ZERO, ONE, ONE, ONE, ONE, ONE, ONE, SIX};
        var cut = new HeuristicCombCuts.Cut(Set.of(1, 2, 3),
                Set.of(Set.of(1, 4, 7), Set.of(2, 5), Set.of(3, 6)), 12);
        var permutations = new ArrayList<int[]>();
        permutations(new int[]{1, 2, 3, 4, 5, 6}, 0, permutations);
        var checked = 0;

        for (var permutation : permutations) {
            for (var breaks = 0; breaks < 1 << 5; breaks++) {
                var edges = routing(permutation, breaks, 8);
                addEdge(edges, 0, 7, 2.0);
                assertTrue(HeuristicCombCuts.violation(cut, result(edges)).signum() <= 0);
                checked++;
            }
        }
        assertEquals(23_040, checked);
        assertTrue(HeuristicCombCuts.isStructurallyValid(cut, SIX, demands));
    }

    private static double[][] ordinaryCombPoint() {
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

    private static double[][] enlargedCombPoint() {
        var edges = graph(8);
        edge(edges, 1, 2, 0.5);
        edge(edges, 1, 3, 0.5);
        edge(edges, 2, 3, 0.5);
        edge(edges, 1, 4, 0.5);
        edge(edges, 1, 7, 0.5);
        edge(edges, 4, 7, 0.5);
        edge(edges, 2, 5, 1.0);
        edge(edges, 3, 6, 1.0);
        edge(edges, 0, 4, 1.0);
        edge(edges, 0, 5, 1.0);
        edge(edges, 0, 6, 1.0);
        edge(edges, 0, 7, 1.0);
        return edges;
    }

    private static double[][] routing(int[] permutation, int breaks, int size) {
        var edges = graph(size);
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
}

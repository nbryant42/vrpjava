package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import org.junit.jupiter.api.Test;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.TEN;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.ojalgo.optimisation.Optimisation.State.OPTIMAL;

class SubtourCutsTest {
    @Test
    void findsAConnectedFractionalBottleneckAndStrengthensItsCut() {
        var edges = connectedFractionalBottleneck();
        var demands = new BigDecimal[]{ZERO, BigDecimal.valueOf(4), BigDecimal.valueOf(4),
                BigDecimal.valueOf(4), ONE, ONE};

        assertEquals(List.of(new Cut(2, Set.of(1, 2, 3))),
                SubtourCuts.generate(TEN, demands, result(edges)));
    }

    @Test
    void findsTheConnectedK1ConnectivityGap() {
        var edges = connectedFractionalBottleneck();
        var demands = new BigDecimal[]{ZERO, BigDecimal.valueOf(3), BigDecimal.valueOf(3),
                BigDecimal.valueOf(3), ONE, ONE};

        assertEquals(List.of(new Cut(1, Set.of(1, 2, 3))),
                SubtourCuts.generate(TEN, demands, result(edges)));
    }

    @Test
    void preservesAllDisconnectedComponentCuts() {
        var edges = graph(10);
        triangle(edges, 1, 2, 3);
        triangle(edges, 4, 5, 6);
        edge(edges, 0, 7, 2.0);
        edge(edges, 0, 8, 2.0);
        edge(edges, 0, 9, 2.0);
        var demands = new BigDecimal[]{ZERO, TWO, TWO, TWO, ONE, ONE, ONE, ONE, ONE, ONE};

        var cuts = new HashSet<>(SubtourCuts.generate(BigDecimal.valueOf(5), demands, result(edges)));

        assertEquals(Set.of(new Cut(2, Set.of(1, 2, 3)), new Cut(1, Set.of(4, 5, 6))), cuts);
    }

    @Test
    void doesNotCutAConnectedSolutionWhoseMinimumBoundaryIsTwo() {
        var edges = graph(4);
        edge(edges, 0, 1, 1.0);
        edge(edges, 1, 2, 1.0);
        edge(edges, 2, 3, 1.0);
        edge(edges, 3, 0, 1.0);
        var demands = new BigDecimal[]{ZERO, ONE, ONE, ONE};

        assertTrue(SubtourCuts.generate(TEN, demands, result(edges)).isEmpty());
    }

    @Test
    void ignoresNegativeSolverArtifactsWhenBuildingTheCutGraph() {
        var edges = graph(4);
        edge(edges, 0, 1, 1.0);
        edge(edges, 1, 2, 1.0);
        edge(edges, 2, 3, 1.0);
        edge(edges, 3, 0, 1.0);
        edge(edges, 0, 2, -1.0e-12);
        var demands = new BigDecimal[]{ZERO, ONE, ONE, ONE};

        assertTrue(SubtourCuts.generate(TEN, demands, result(edges)).isEmpty());
    }

    private static double[][] connectedFractionalBottleneck() {
        var edges = graph(6);
        edge(edges, 0, 1, 0.4);
        edge(edges, 0, 2, 0.4);
        edge(edges, 0, 3, 0.4);
        edge(edges, 0, 4, 1.4);
        edge(edges, 0, 5, 1.4);
        edge(edges, 1, 2, 0.8);
        edge(edges, 1, 3, 0.8);
        edge(edges, 2, 3, 0.8);
        edge(edges, 4, 5, 0.6);
        return edges;
    }

    private static double[][] graph(int size) {
        return new double[size][size];
    }

    private static void triangle(double[][] edges, int first, int second, int third) {
        edge(edges, first, second, 1.0);
        edge(edges, first, third, 1.0);
        edge(edges, second, third, 1.0);
    }

    private static void edge(double[][] edges, int first, int second, double weight) {
        edges[first][second] = weight;
        edges[second][first] = weight;
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

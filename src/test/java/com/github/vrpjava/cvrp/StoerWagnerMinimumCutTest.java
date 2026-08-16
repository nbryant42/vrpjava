package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.Test;

import java.math.BigDecimal;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;

import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

class StoerWagnerMinimumCutTest {
    @Test
    void agreesWithExhaustiveEnumerationOnSmallGraphs() {
        var random = new Random(0x5A0E7L);

        for (var size = 2; size <= 8; size++) {
            for (var trial = 0; trial < 25; trial++) {
                var graph = randomGraph(size, random);
                var expected = exhaustiveMinimumCut(graph);
                var actual = StoerWagnerMinimumCut.find(graph);

                assertFalse(actual.vertices().isEmpty());
                assertFalse(actual.vertices().size() == size);
                assertEquals(0, expected.compareTo(actual.weight()),
                        "minimum-cut weight for size=" + size + ", trial=" + trial);
                assertEquals(0, actual.weight().compareTo(boundary(graph, actual.vertices())),
                        "reported partition weight for size=" + size + ", trial=" + trial);
            }
        }
    }

    private static BigDecimal[][] randomGraph(int size, Random random) {
        var graph = new BigDecimal[size][size];

        for (var row = 0; row < size; row++) {
            graph[row][row] = ZERO;
            for (var col = 0; col < row; col++) {
                var weight = BigDecimal.valueOf(random.nextInt(21), 1);
                graph[row][col] = weight;
                graph[col][row] = weight;
            }
        }
        return graph;
    }

    private static BigDecimal exhaustiveMinimumCut(BigDecimal[][] graph) {
        BigDecimal best = null;
        var customerCount = graph.length - 1;

        // Vertex zero stays outside the enumerated side, eliminating complementary duplicates.
        for (var mask = 1; mask < 1 << customerCount; mask++) {
            var subset = new HashSet<Integer>();
            for (var offset = 0; offset < customerCount; offset++) {
                if ((mask & 1 << offset) != 0) {
                    subset.add(offset + 1);
                }
            }

            var weight = boundary(graph, subset);
            if (best == null || weight.compareTo(best) < 0) {
                best = weight;
            }
        }
        return best;
    }

    private static BigDecimal boundary(BigDecimal[][] graph, Set<Integer> subset) {
        var total = ZERO;

        for (var inside : subset) {
            for (var outside = 0; outside < graph.length; outside++) {
                if (!subset.contains(outside)) {
                    total = total.add(graph[inside][outside]);
                }
            }
        }
        return total;
    }
}

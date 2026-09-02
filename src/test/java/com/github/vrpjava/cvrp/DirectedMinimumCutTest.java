package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.Test;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class DirectedMinimumCutTest {
    @Test
    void supportsParallelAndAntiparallelArcs() {
        var arcs = List.of(
                new DirectedMinimumCut.Arc(0, 1, new BigDecimal("1.2")),
                new DirectedMinimumCut.Arc(0, 1, new BigDecimal("0.8")),
                new DirectedMinimumCut.Arc(1, 0, BigDecimal.valueOf(9)),
                new DirectedMinimumCut.Arc(1, 2, BigDecimal.valueOf(3)),
                new DirectedMinimumCut.Arc(0, 2, BigDecimal.valueOf(4)));

        var actual = DirectedMinimumCut.find(3, arcs, 0, 2);

        assertEquals(0, BigDecimal.valueOf(6).compareTo(actual.weight()));
        assertEquals(Set.of(0), actual.sourceVertices());
        assertEquals(List.of(Set.of(0)), actual.minimumSourceSides());
    }

    @Test
    void returnsALinearCanonicalSubsetOfAnExponentialTiedFamily() {
        var arcs = List.of(new DirectedMinimumCut.Arc(0, 3, BigDecimal.ONE));

        var actual = DirectedMinimumCut.find(4, arcs, 0, 3);

        assertEquals(Set.of(Set.of(0), Set.of(0, 1), Set.of(0, 2)),
                new HashSet<>(actual.minimumSourceSides()));
    }

    @Test
    void agreesWithExhaustiveEnumerationOnSmallDirectedGraphs() {
        var random = new Random(0xD1A1C7EDL);

        for (var size = 2; size <= 8; size++) {
            for (var trial = 0; trial < 25; trial++) {
                var arcs = randomArcs(size, random);
                var exhaustiveSides = exhaustiveMinimumSourceSides(size, arcs, 0, size - 1);
                var expected = cutWeight(arcs, exhaustiveSides.iterator().next());
                var actual = DirectedMinimumCut.find(size, arcs, 0, size - 1);

                assertTrue(actual.sourceVertices().contains(0));
                assertFalse(actual.sourceVertices().contains(size - 1));
                assertEquals(0, expected.compareTo(actual.weight()),
                        "minimum-cut weight for size=" + size + ", trial=" + trial);
                assertEquals(0, actual.weight().compareTo(cutWeight(arcs, actual.sourceVertices())));
                for (var sourceSide : actual.minimumSourceSides()) {
                    assertEquals(0, expected.compareTo(cutWeight(arcs, sourceSide)));
                }
                assertEquals(principalMinimumSourceSides(size, exhaustiveSides, 0, size - 1),
                        new HashSet<>(actual.minimumSourceSides()));
            }
        }
    }

    private static List<DirectedMinimumCut.Arc> randomArcs(int size, Random random) {
        var arcs = new ArrayList<DirectedMinimumCut.Arc>();
        for (var source = 0; source < size; source++) {
            for (var target = 0; target < size; target++) {
                if (source != target && random.nextBoolean()) {
                    arcs.add(new DirectedMinimumCut.Arc(source, target,
                            BigDecimal.valueOf(random.nextInt(21), 1)));
                }
            }
        }
        return arcs;
    }

    private static Set<Set<Integer>> exhaustiveMinimumSourceSides(int size,
                                                                  List<DirectedMinimumCut.Arc> arcs,
                                                                  int source,
                                                                  int sink) {
        BigDecimal best = null;
        var bestSides = new HashSet<Set<Integer>>();
        for (var mask = 0; mask < 1 << size; mask++) {
            if ((mask & 1 << source) == 0 || (mask & 1 << sink) != 0) {
                continue;
            }
            var sourceVertices = new HashSet<Integer>();
            for (var vertex = 0; vertex < size; vertex++) {
                if ((mask & 1 << vertex) != 0) {
                    sourceVertices.add(vertex);
                }
            }
            var weight = cutWeight(arcs, sourceVertices);
            if (best == null || weight.compareTo(best) < 0) {
                best = weight;
                bestSides.clear();
                bestSides.add(Set.copyOf(sourceVertices));
            } else if (weight.compareTo(best) == 0) {
                bestSides.add(Set.copyOf(sourceVertices));
            }
        }
        return Set.copyOf(bestSides);
    }

    private static Set<Set<Integer>> principalMinimumSourceSides(int size,
                                                                  Set<Set<Integer>> allMinimumSides,
                                                                  int source,
                                                                  int sink) {
        var canonical = new HashSet<>(allMinimumSides.iterator().next());
        for (var side : allMinimumSides) {
            canonical.retainAll(side);
        }

        var result = new HashSet<Set<Integer>>();
        result.add(Set.copyOf(canonical));
        for (var vertex = 0; vertex < size; vertex++) {
            if (vertex == source || vertex == sink || canonical.contains(vertex)) {
                continue;
            }
            var candidateVertex = vertex;
            var containing = allMinimumSides.stream().filter(side -> side.contains(candidateVertex)).toList();
            if (containing.isEmpty()) {
                continue;
            }
            var principal = new HashSet<>(containing.getFirst());
            for (var side : containing) {
                principal.retainAll(side);
            }
            result.add(Set.copyOf(principal));
        }
        return Set.copyOf(result);
    }

    private static BigDecimal cutWeight(List<DirectedMinimumCut.Arc> arcs, Set<Integer> sourceVertices) {
        var weight = ZERO;
        for (var arc : arcs) {
            if (sourceVertices.contains(arc.source()) && !sourceVertices.contains(arc.target())) {
                weight = weight.add(arc.capacity());
            }
        }
        return weight;
    }
}

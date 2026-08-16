package com.github.vrpjava.cvrp;

import java.math.BigDecimal;
import java.util.*;

import static java.math.BigDecimal.ZERO;

/**
 * Stoer-Wagner global minimum cut for an undirected graph with nonnegative edge weights.
 *
 * <p>This deliberately uses the simple {@code O(|V|^3)} implementation. CVRP relaxations are dense and the intended
 * instances have tens, rather than thousands, of vertices, so avoiding another dependency is more useful here than a
 * more elaborate priority-queue implementation.</p>
 */
final class StoerWagnerMinimumCut {
    private StoerWagnerMinimumCut() {
    }

    record Result(BigDecimal weight, Set<Integer> vertices) {
        Result {
            vertices = Set.copyOf(vertices);
        }
    }

    static Result find(BigDecimal[][] edgeWeights) {
        var graph = validatedCopy(edgeWeights);
        var size = graph.length;
        var active = new boolean[size];
        var groups = new ArrayList<Set<Integer>>(size);

        for (var vertex = 0; vertex < size; vertex++) {
            active[vertex] = true;
            groups.add(new HashSet<>(Set.of(vertex)));
        }

        BigDecimal bestWeight = null;
        Set<Integer> bestVertices = null;
        var activeCount = size;

        while (activeCount > 1) {
            var added = new boolean[size];
            var connectionWeights = new BigDecimal[size];
            Arrays.fill(connectionWeights, ZERO);

            var previous = -1;

            for (var step = 0; step < activeCount; step++) {
                var selected = mostTightlyConnected(active, added, connectionWeights);

                if (step == activeCount - 1) {
                    var cutWeight = connectionWeights[selected];
                    if (bestWeight == null || cutWeight.compareTo(bestWeight) < 0) {
                        bestWeight = cutWeight;
                        bestVertices = Set.copyOf(groups.get(selected));
                    }

                    merge(graph, active, groups, previous, selected);
                    activeCount--;
                } else {
                    added[selected] = true;
                    previous = selected;

                    for (var vertex = 0; vertex < size; vertex++) {
                        if (active[vertex] && !added[vertex]) {
                            connectionWeights[vertex] = connectionWeights[vertex].add(graph[selected][vertex]);
                        }
                    }
                }
            }
        }

        return new Result(Objects.requireNonNull(bestWeight), Objects.requireNonNull(bestVertices));
    }

    private static int mostTightlyConnected(boolean[] active,
                                             boolean[] added,
                                             BigDecimal[] connectionWeights) {
        var selected = -1;

        for (var vertex = 0; vertex < active.length; vertex++) {
            if (active[vertex] && !added[vertex] &&
                    (selected < 0 || connectionWeights[vertex].compareTo(connectionWeights[selected]) > 0)) {
                selected = vertex;
            }
        }

        if (selected < 0) {
            throw new IllegalStateException("No active vertex available for a minimum-cut phase");
        }
        return selected;
    }

    private static void merge(BigDecimal[][] graph,
                              boolean[] active,
                              List<Set<Integer>> groups,
                              int target,
                              int source) {
        groups.get(target).addAll(groups.get(source));

        for (var vertex = 0; vertex < graph.length; vertex++) {
            if (active[vertex] && vertex != target && vertex != source) {
                var mergedWeight = graph[target][vertex].add(graph[source][vertex]);
                graph[target][vertex] = mergedWeight;
                graph[vertex][target] = mergedWeight;
            }
        }
        active[source] = false;
    }

    private static BigDecimal[][] validatedCopy(BigDecimal[][] edgeWeights) {
        Objects.requireNonNull(edgeWeights, "edgeWeights");
        var size = edgeWeights.length;
        if (size < 2) {
            throw new IllegalArgumentException("A minimum cut requires at least two vertices");
        }

        var copy = new BigDecimal[size][size];

        for (var edgeWeight : edgeWeights) {
            if (edgeWeight == null || edgeWeight.length != size) {
                throw new IllegalArgumentException("edgeWeights must be a square matrix");
            }
        }

        for (var row = 0; row < size; row++) {
            for (var col = 0; col < size; col++) {
                var weight = Objects.requireNonNull(edgeWeights[row][col], "edge weight");
                if (weight.signum() < 0) {
                    throw new IllegalArgumentException("edge weights must be nonnegative");
                }
                copy[row][col] = weight;
            }
        }

        for (var row = 0; row < size; row++) {
            for (var col = 0; col < row; col++) {
                if (copy[row][col].compareTo(copy[col][row]) != 0) {
                    throw new IllegalArgumentException("edgeWeights must be symmetric");
                }
            }
        }
        return copy;
    }
}

package com.github.vrpjava.cvrp;

import java.math.BigDecimal;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

import static java.math.BigDecimal.ZERO;

/**
 * Dependency-free source-sink minimum cut for a directed graph with nonnegative {@link BigDecimal} capacities.
 *
 * <p>The implementation uses Dinic's maximum-flow algorithm. The intended separation graphs have at most a few
 * hundred vertices, so exact decimal arithmetic and a simple adjacency-list implementation are preferable to a new
 * graph dependency.</p>
 */
final class DirectedMinimumCut {
    private DirectedMinimumCut() {
    }

    record Arc(int source, int target, BigDecimal capacity) {
        Arc {
            Objects.requireNonNull(capacity, "capacity");
            if (source == target) {
                throw new IllegalArgumentException("An arc requires two distinct vertices.");
            }
            if (capacity.signum() < 0) {
                throw new IllegalArgumentException("Arc capacities must be nonnegative.");
            }
        }
    }

    record Result(BigDecimal weight,
                  Set<Integer> sourceVertices,
                  List<Set<Integer>> minimumSourceSides) {
        Result {
            Objects.requireNonNull(weight, "weight");
            sourceVertices = Set.copyOf(sourceVertices);
            var immutableSides = new ArrayList<Set<Integer>>();
            immutableSides.add(sourceVertices);
            for (var side : minimumSourceSides) {
                var immutableSide = Set.copyOf(side);
                if (!immutableSides.contains(immutableSide)) {
                    immutableSides.add(immutableSide);
                }
            }
            minimumSourceSides = List.copyOf(immutableSides);
        }
    }

    static Result find(int vertexCount, Collection<Arc> inputArcs, int source, int sink) {
        if (vertexCount < 2) {
            throw new IllegalArgumentException("A minimum cut requires at least two vertices.");
        }
        if (source < 0 || source >= vertexCount || sink < 0 || sink >= vertexCount || source == sink) {
            throw new IllegalArgumentException("The source and sink must be distinct graph vertices.");
        }

        var arcs = List.copyOf(inputArcs);
        var graph = new ArrayList<List<ResidualArc>>(vertexCount);
        for (var vertex = 0; vertex < vertexCount; vertex++) {
            graph.add(new ArrayList<>());
        }

        var sourceCapacity = ZERO;
        for (var arc : arcs) {
            validateVertex(arc.source(), vertexCount);
            validateVertex(arc.target(), vertexCount);
            addResidualArc(graph, arc.source(), arc.target(), arc.capacity());
            if (arc.source() == source) {
                sourceCapacity = sourceCapacity.add(arc.capacity());
            }
        }

        var levels = new int[vertexCount];
        while (buildLevels(graph, source, sink, levels)) {
            var nextArc = new int[vertexCount];
            BigDecimal pushed;
            while ((pushed = push(graph, levels, nextArc, source, sink, sourceCapacity)).signum() > 0) {
                // Residual capacities carry the flow; its value is not otherwise needed for the cut partition.
            }
        }

        var sourceVertices = reachableVertices(graph, source);
        var minimumSourceSides = principalMinimumSourceSides(graph, source, sink, sourceVertices);
        var weight = ZERO;
        for (var arc : arcs) {
            if (sourceVertices.contains(arc.source()) && !sourceVertices.contains(arc.target())) {
                weight = weight.add(arc.capacity());
            }
        }
        return new Result(weight, sourceVertices, minimumSourceSides);
    }

    private static void validateVertex(int vertex, int vertexCount) {
        if (vertex < 0 || vertex >= vertexCount) {
            throw new IllegalArgumentException("Arc endpoint is outside the graph: " + vertex);
        }
    }

    private static void addResidualArc(List<List<ResidualArc>> graph,
                                       int source,
                                       int target,
                                       BigDecimal capacity) {
        var forward = new ResidualArc(target, graph.get(target).size(), capacity);
        var reverse = new ResidualArc(source, graph.get(source).size(), ZERO);
        graph.get(source).add(forward);
        graph.get(target).add(reverse);
    }

    private static boolean buildLevels(List<List<ResidualArc>> graph, int source, int sink, int[] levels) {
        java.util.Arrays.fill(levels, -1);
        levels[source] = 0;
        var queue = new ArrayDeque<Integer>();
        queue.add(source);

        while (!queue.isEmpty()) {
            var vertex = queue.removeFirst();
            for (var arc : graph.get(vertex)) {
                if (arc.capacity.signum() > 0 && levels[arc.target] < 0) {
                    levels[arc.target] = levels[vertex] + 1;
                    queue.addLast(arc.target);
                }
            }
        }
        return levels[sink] >= 0;
    }

    private static BigDecimal push(List<List<ResidualArc>> graph,
                                   int[] levels,
                                   int[] nextArc,
                                   int vertex,
                                   int sink,
                                   BigDecimal available) {
        if (vertex == sink || available.signum() == 0) {
            return available;
        }

        var outgoing = graph.get(vertex);
        while (nextArc[vertex] < outgoing.size()) {
            var arc = outgoing.get(nextArc[vertex]);
            if (arc.capacity.signum() > 0 && levels[arc.target] == levels[vertex] + 1) {
                var pushed = push(graph, levels, nextArc, arc.target, sink, available.min(arc.capacity));
                if (pushed.signum() > 0) {
                    arc.capacity = arc.capacity.subtract(pushed);
                    var reverse = graph.get(arc.target).get(arc.reverseIndex);
                    reverse.capacity = reverse.capacity.add(pushed);
                    return pushed;
                }
            }
            nextArc[vertex]++;
        }
        return ZERO;
    }

    private static Set<Integer> reachableVertices(List<List<ResidualArc>> graph, int source) {
        var result = new HashSet<Integer>();
        var queue = new ArrayDeque<Integer>();
        result.add(source);
        queue.add(source);

        while (!queue.isEmpty()) {
            var vertex = queue.removeFirst();
            for (var arc : graph.get(vertex)) {
                if (arc.capacity.signum() > 0 && result.add(arc.target)) {
                    queue.addLast(arc.target);
                }
            }
        }
        return result;
    }

    /**
     * Return a linear-size canonical subset of the lattice of tied minimum cuts.
     *
     * <p>The source-reachable set is the smallest minimum source side. For each residual vertex not in that set,
     * its reachability closure can be united with the source side when it cannot reach the sink. The union has no
     * positive residual arc leaving it, so it is another minimum cut. This can expose useful ties without rerunning
     * maximum flow or attempting to enumerate the potentially exponential complete cut lattice.</p>
     */
    private static List<Set<Integer>> principalMinimumSourceSides(List<List<ResidualArc>> graph,
                                                                   int source,
                                                                   int sink,
                                                                   Set<Integer> sourceVertices) {
        var result = new ArrayList<Set<Integer>>();
        result.add(sourceVertices);
        for (var vertex = 0; vertex < graph.size(); vertex++) {
            if (vertex == source || vertex == sink || sourceVertices.contains(vertex)) {
                continue;
            }
            var closure = reachableVertices(graph, vertex);
            if (closure.contains(sink)) {
                continue;
            }
            var sourceSide = new HashSet<>(sourceVertices);
            sourceSide.addAll(closure);
            var immutableSide = Set.copyOf(sourceSide);
            if (!result.contains(immutableSide)) {
                result.add(immutableSide);
            }
        }
        return List.copyOf(result);
    }

    private static final class ResidualArc {
        private final int target;
        private final int reverseIndex;
        private BigDecimal capacity;

        private ResidualArc(int target, int reverseIndex, BigDecimal capacity) {
            this.target = target;
            this.reverseIndex = reverseIndex;
            this.capacity = capacity;
        }
    }
}

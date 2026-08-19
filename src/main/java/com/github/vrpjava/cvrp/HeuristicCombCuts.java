package com.github.vrpjava.cvrp;

import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static com.github.vrpjava.cvrp.CVRPSolver.minVehicles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.getVariable_noFlip;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;
import static org.ojalgo.function.constant.BigMath.HALF;

/**
 * Heuristic separation of strengthened comb inequalities with disjoint, depot-free teeth.
 * <p>
 * Fractional-support components and blocks provide candidate handles. Greedy crossing-edge matchings then provide
 * ordinary odd-tooth combs, and a small number of greedy paths enlarge their teeth while evaluating the strengthened
 * rounded-capacity right-hand side. This deliberately implements only the inexpensive core of the Lysgaard-Letchford-
 * Eglese heuristic: it does not perform graph shrinking or Padberg-Rao separation.
 */
final class HeuristicCombCuts {
    private static final int MAX_HANDLES = 128;
    private static final int MAX_FORCED_MATCHING_STARTS = 8;
    private static final int MAX_INITIAL_CUTS_PER_HANDLE = 2;
    private static final int MAX_ENLARGEMENT_PATHS = 4;
    private static final int MAX_CUTS_PER_PASS = 32;

    private HeuristicCombCuts() {
    }

    record Edge(int first, int second) implements Comparable<Edge> {
        Edge {
            if (first == second) {
                throw new IllegalArgumentException("An edge requires two distinct vertices.");
            }
            if (first > second) {
                var swap = first;
                first = second;
                second = swap;
            }
        }

        @Override
        public int compareTo(Edge other) {
            var byFirst = Integer.compare(first, other.first);
            return byFirst != 0 ? byFirst : Integer.compare(second, other.second);
        }
    }

    record Cut(Set<Integer> handle, Set<Set<Integer>> teeth, int rightHandSide) {
        Cut {
            handle = Set.copyOf(handle);
            var copiedTeeth = new HashSet<Set<Integer>>();
            teeth.forEach(tooth -> copiedTeeth.add(Set.copyOf(tooth)));
            teeth = Set.copyOf(copiedTeeth);
        }
    }

    record Separation(List<Cut> cuts, boolean timedOut) {
        Separation {
            cuts = List.copyOf(cuts);
        }
    }

    static Separation generate(BigDecimal vehicleCapacity,
                               BigDecimal[] demands,
                               Optimisation.Result parentResult,
                               long deadline) {
        if (demands.length < 7) {
            return new Separation(List.of(), false);
        }
        if (expired(deadline)) {
            return new Separation(List.of(), true);
        }

        var handleSearch = candidateHandles(parentResult, demands.length, deadline);
        if (handleSearch.timedOut()) {
            return new Separation(List.of(), true);
        }

        var handles = handleSearch.handles().stream()
                .sorted(Comparator
                        .comparing((Set<Integer> handle) -> boundary(handle, parentResult))
                        .thenComparingInt(Set::size)
                        .thenComparing(HeuristicCombCuts::sortedVertices, HeuristicCombCuts::compareLists))
                .limit(MAX_HANDLES)
                .toList();
        var candidates = new HashMap<Cut, BigDecimal>();

        for (var handle : handles) {
            if (expired(deadline)) {
                return finish(candidates, parentResult, true);
            }

            var initialCuts = ordinaryCandidates(handle, vehicleCapacity, demands, parentResult).stream()
                    .sorted(cutComparator(parentResult))
                    .limit(MAX_INITIAL_CUTS_PER_HANDLE)
                    .toList();

            for (var initial : initialCuts) {
                consider(initial, vehicleCapacity, demands, parentResult, candidates);
                var enlargement = enlarge(initial, vehicleCapacity, demands, parentResult, deadline);
                enlargement.cuts().forEach(cut ->
                        consider(cut, vehicleCapacity, demands, parentResult, candidates));
                if (enlargement.timedOut()) {
                    return finish(candidates, parentResult, true);
                }
            }
        }
        return finish(candidates, parentResult, false);
    }

    static boolean isStructurallyValid(Cut cut, BigDecimal vehicleCapacity, BigDecimal[] demands) {
        if (cut.handle().isEmpty() || cut.teeth().size() < 3 || cut.teeth().size() % 2 == 0) {
            return false;
        }

        for (var vertex : cut.handle()) {
            if (vertex <= 0 || vertex >= demands.length) {
                return false;
            }
        }

        var used = new HashSet<Integer>();
        for (var tooth : cut.teeth()) {
            if (tooth.size() < 2) {
                return false;
            }
            var intersectsHandle = false;
            var leavesHandle = false;
            for (var vertex : tooth) {
                if (vertex <= 0 || vertex >= demands.length || !used.add(vertex)) {
                    return false;
                }
                intersectsHandle |= cut.handle().contains(vertex);
                leavesHandle |= !cut.handle().contains(vertex);
            }
            if (!intersectsHandle || !leavesHandle) {
                return false;
            }
        }

        var sum = combSum(cut.handle(), cut.teeth(), vehicleCapacity, demands);
        return sum % 2 == 1 && cut.rightHandSide() == sum + 1;
    }

    /** Positive means that the current relaxation violates the cut. */
    static BigDecimal violation(Cut cut, Optimisation.Result result) {
        var leftHandSide = boundary(cut.handle(), result);
        for (var tooth : cut.teeth()) {
            leftHandSide = leftHandSide.add(boundary(tooth, result));
        }
        return BigDecimal.valueOf(cut.rightHandSide()).subtract(leftHandSide);
    }

    static void addTo(ExpressionsBasedModel model, Cut cut, int size) {
        var name = "heuristic-comb cut:H=" + sortedVertices(cut.handle()) + ",T=" + cut.teeth().stream()
                .map(HeuristicCombCuts::sortedVertices)
                .sorted(HeuristicCombCuts::compareLists)
                .toList();
        var expression = model.newExpression(name).lower(cut.rightHandSide());
        var coefficients = new HashMap<Edge, Integer>();

        addBoundaryCoefficients(cut.handle(), size, coefficients);
        cut.teeth().forEach(tooth -> addBoundaryCoefficients(tooth, size, coefficients));
        coefficients.forEach((edge, coefficient) -> expression.set(
                getVariable_noFlip(edge.second(), edge.first(), model), coefficient));
    }

    private static HandleSearch candidateHandles(Optimisation.Result result, int size, long deadline) {
        var fractionalEdges = new ArrayList<WeightedEdge>();
        for (var row = 2; row < size; row++) {
            for (var col = 1; col < row; col++) {
                var value = clampedEdgeValue(row, col, result);
                if (value.signum() > 0 && value.compareTo(ONE) < 0) {
                    fractionalEdges.add(new WeightedEdge(new Edge(row, col), value,
                            value.subtract(HALF).abs()));
                }
            }
        }
        fractionalEdges.sort(Comparator.comparing(WeightedEdge::distance).thenComparing(WeightedEdge::edge));

        var adjacency = new ArrayList<Set<Integer>>(size);
        for (var vertex = 0; vertex < size; vertex++) {
            adjacency.add(new HashSet<>());
        }
        var handles = new LinkedHashSet<Set<Integer>>();

        for (var index = 0; index < fractionalEdges.size(); ) {
            if (expired(deadline)) {
                return new HandleSearch(List.copyOf(handles), true);
            }
            var distance = fractionalEdges.get(index).distance();
            do {
                var edge = fractionalEdges.get(index++).edge();
                adjacency.get(edge.first()).add(edge.second());
                adjacency.get(edge.second()).add(edge.first());
            } while (index < fractionalEdges.size() &&
                    fractionalEdges.get(index).distance().compareTo(distance) == 0);

            connectedComponents(adjacency).forEach(handle -> addHandle(handles, handle, size));
            biconnectedComponents(adjacency).forEach(handle -> addHandle(handles, handle, size));
        }
        return new HandleSearch(List.copyOf(handles), false);
    }

    private static List<Cut> ordinaryCandidates(Set<Integer> handle,
                                                BigDecimal vehicleCapacity,
                                                BigDecimal[] demands,
                                                Optimisation.Result result) {
        var crossing = new ArrayList<WeightedEdge>();
        for (var row = 2; row < demands.length; row++) {
            for (var col = 1; col < row; col++) {
                if (handle.contains(row) == handle.contains(col) ||
                        demands[row].add(demands[col]).compareTo(vehicleCapacity) > 0) {
                    continue;
                }
                var value = clampedEdgeValue(row, col, result);
                if (value.signum() > 0) {
                    crossing.add(new WeightedEdge(new Edge(row, col), value, ZERO));
                }
            }
        }
        crossing.sort(Comparator.comparing(WeightedEdge::value).reversed().thenComparing(WeightedEdge::edge));

        var candidates = new LinkedHashSet<Cut>();
        var attempts = Math.min(MAX_FORCED_MATCHING_STARTS, crossing.size());
        for (var forced = -1; forced < attempts; forced++) {
            var matching = greedyMatching(crossing, forced);
            var teeth = new LinkedHashSet<Set<Integer>>();
            for (var index = 0; index < matching.size(); index++) {
                var edge = matching.get(index).edge();
                teeth.add(Set.of(edge.first(), edge.second()));
                var toothCount = index + 1;
                if (toothCount >= 3 && toothCount % 2 == 1) {
                    candidates.add(new Cut(handle, teeth, 3 * toothCount + 1));
                }
            }
        }
        return List.copyOf(candidates);
    }

    private static List<WeightedEdge> greedyMatching(List<WeightedEdge> edges, int forcedIndex) {
        var selected = new ArrayList<WeightedEdge>();
        var used = new HashSet<Integer>();
        if (forcedIndex >= 0) {
            select(edges.get(forcedIndex), selected, used);
        }
        for (var edge : edges) {
            if (!used.contains(edge.edge().first()) && !used.contains(edge.edge().second())) {
                select(edge, selected, used);
            }
        }
        selected.sort(Comparator.comparing(WeightedEdge::value).reversed().thenComparing(WeightedEdge::edge));
        return selected;
    }

    private static void select(WeightedEdge edge, List<WeightedEdge> selected, Set<Integer> used) {
        selected.add(edge);
        used.add(edge.edge().first());
        used.add(edge.edge().second());
    }

    private static Enlargement enlarge(Cut initial,
                                       BigDecimal vehicleCapacity,
                                       BigDecimal[] demands,
                                       Optimisation.Result result,
                                       long deadline) {
        var orderedTeeth = initial.teeth().stream()
                .map(HashSet::new)
                .sorted(Comparator.comparing(HeuristicCombCuts::sortedVertices,
                        HeuristicCombCuts::compareLists))
                .toList();
        var cuts = new LinkedHashSet<Cut>();
        var paths = Math.min(MAX_ENLARGEMENT_PATHS - 1, orderedTeeth.size());
        var handleBoundary = boundary(initial.handle(), result);
        var vertexDegrees = new BigDecimal[demands.length];
        for (var vertex = 1; vertex < demands.length; vertex++) {
            vertexDegrees[vertex] = vertexDegree(vertex, demands.length, result);
        }

        for (var focus = -1; focus < paths; focus++) {
            var teeth = new ArrayList<Set<Integer>>();
            orderedTeeth.forEach(tooth -> teeth.add(new HashSet<>(tooth)));
            var used = new HashSet<Integer>();
            teeth.forEach(used::addAll);
            var toothBoundaries = new BigDecimal[teeth.size()];
            var insideDemands = new BigDecimal[teeth.size()];
            var outsideDemands = new BigDecimal[teeth.size()];
            var toothSums = new int[teeth.size()];
            var leftHandSide = handleBoundary;
            var combSum = 0;

            for (var tooth = 0; tooth < teeth.size(); tooth++) {
                toothBoundaries[tooth] = boundary(teeth.get(tooth), result);
                insideDemands[tooth] = demand(teeth.get(tooth), initial.handle(), true, demands);
                outsideDemands[tooth] = demand(teeth.get(tooth), initial.handle(), false, demands);
                toothSums[tooth] = toothSum(vehicleCapacity, insideDemands[tooth], outsideDemands[tooth]);
                leftHandSide = leftHandSide.add(toothBoundaries[tooth]);
                combSum += toothSums[tooth];
            }

            while (used.size() < demands.length - 1) {
                if (expired(deadline)) {
                    return new Enlargement(List.copyOf(cuts), true);
                }
                Move best = null;
                for (var vertex = 1; vertex < demands.length; vertex++) {
                    if (used.contains(vertex)) {
                        continue;
                    }
                    for (var toothIndex = 0; toothIndex < teeth.size(); toothIndex++) {
                        if (focus >= 0 && toothIndex != focus) {
                            continue;
                        }
                        var newBoundary = toothBoundaries[toothIndex]
                                .add(vertexDegrees[vertex])
                                .subtract(connections(vertex, teeth.get(toothIndex), result)
                                        .multiply(BigDecimal.valueOf(2)));
                        var newInsideDemand = insideDemands[toothIndex];
                        var newOutsideDemand = outsideDemands[toothIndex];
                        if (initial.handle().contains(vertex)) {
                            newInsideDemand = newInsideDemand.add(demands[vertex]);
                        } else {
                            newOutsideDemand = newOutsideDemand.add(demands[vertex]);
                        }
                        var newToothSum = toothSum(vehicleCapacity, newInsideDemand, newOutsideDemand);
                        var newCombSum = combSum - toothSums[toothIndex] + newToothSum;
                        var newLeftHandSide = leftHandSide.subtract(toothBoundaries[toothIndex]).add(newBoundary);
                        var score = BigDecimal.valueOf(newCombSum + 1L).subtract(newLeftHandSide);
                        var move = new Move(vertex, toothIndex, score, newBoundary, newInsideDemand,
                                newOutsideDemand, newToothSum, newCombSum, newLeftHandSide);
                        if (best == null || move.compareTo(best) < 0) {
                            best = move;
                        }
                    }
                }
                if (best == null) {
                    break;
                }
                teeth.get(best.tooth()).add(best.vertex());
                used.add(best.vertex());
                toothBoundaries[best.tooth()] = best.boundary();
                insideDemands[best.tooth()] = best.insideDemand();
                outsideDemands[best.tooth()] = best.outsideDemand();
                toothSums[best.tooth()] = best.toothSum();
                combSum = best.combSum();
                leftHandSide = best.leftHandSide();

                if (combSum % 2 == 1 && best.score().signum() > 0) {
                    var candidate = cut(initial.handle(), teeth, vehicleCapacity, demands);
                    cuts.add(candidate);
                }
            }
        }
        return new Enlargement(List.copyOf(cuts), false);
    }

    private static Cut cut(Set<Integer> handle,
                           List<Set<Integer>> teeth,
                           BigDecimal vehicleCapacity,
                           BigDecimal[] demands) {
        var copied = new HashSet<Set<Integer>>();
        teeth.forEach(tooth -> copied.add(Set.copyOf(tooth)));
        return new Cut(handle, copied, combSum(handle, copied, vehicleCapacity, demands) + 1);
    }

    private static int combSum(Set<Integer> handle,
                               Set<Set<Integer>> teeth,
                               BigDecimal vehicleCapacity,
                               BigDecimal[] demands) {
        var sum = 0;
        for (var tooth : teeth) {
            var insideDemand = ZERO;
            var outsideDemand = ZERO;
            for (var vertex : tooth) {
                if (handle.contains(vertex)) {
                    insideDemand = insideDemand.add(demands[vertex]);
                } else {
                    outsideDemand = outsideDemand.add(demands[vertex]);
                }
            }
            sum += minVehicles(vehicleCapacity, insideDemand);
            sum += minVehicles(vehicleCapacity, outsideDemand);
            sum += minVehicles(vehicleCapacity, insideDemand.add(outsideDemand));
        }
        return sum;
    }

    private static void consider(Cut cut,
                                 BigDecimal vehicleCapacity,
                                 BigDecimal[] demands,
                                 Optimisation.Result result,
                                 Map<Cut, BigDecimal> candidates) {
        if (!isStructurallyValid(cut, vehicleCapacity, demands)) {
            return;
        }
        var violation = violation(cut, result);
        if (violation.signum() > 0) {
            candidates.put(cut, violation);
        }
    }

    private static Separation finish(Map<Cut, BigDecimal> candidates,
                                     Optimisation.Result result,
                                     boolean timedOut) {
        var cuts = candidates.keySet().stream()
                .sorted(cutComparator(result))
                .limit(MAX_CUTS_PER_PASS)
                .toList();
        return new Separation(cuts, timedOut);
    }

    private static Comparator<Cut> cutComparator(Optimisation.Result result) {
        return Comparator.<Cut, BigDecimal>comparing(cut -> violation(cut, result)).reversed()
                .thenComparingInt(cut -> cut.handle().size())
                .thenComparing(cut -> sortedVertices(cut.handle()), HeuristicCombCuts::compareLists)
                .thenComparing(HeuristicCombCuts::teethKey);
    }

    private static String teethKey(Cut cut) {
        return cut.teeth().stream().map(HeuristicCombCuts::sortedVertices)
                .sorted(HeuristicCombCuts::compareLists).toList().toString();
    }

    private static BigDecimal boundary(Set<Integer> subset, Optimisation.Result result) {
        var size = sizeFromVariableCount(result.count());
        var boundary = ZERO;
        for (var row = 1; row < size; row++) {
            for (var col = 0; col < row; col++) {
                if (subset.contains(row) != subset.contains(col)) {
                    boundary = boundary.add(getVariable_noFlip(row, col, result));
                }
            }
        }
        return boundary;
    }

    private static BigDecimal vertexDegree(int vertex, int size, Optimisation.Result result) {
        var degree = ZERO;
        for (var other = 0; other < size; other++) {
            if (other != vertex) {
                degree = degree.add(getVariable_noFlip(Math.max(vertex, other), Math.min(vertex, other), result));
            }
        }
        return degree;
    }

    private static BigDecimal connections(int vertex, Set<Integer> tooth, Optimisation.Result result) {
        var connections = ZERO;
        for (var other : tooth) {
            connections = connections.add(
                    getVariable_noFlip(Math.max(vertex, other), Math.min(vertex, other), result));
        }
        return connections;
    }

    private static BigDecimal demand(Set<Integer> tooth,
                                     Set<Integer> handle,
                                     boolean inside,
                                     BigDecimal[] demands) {
        var total = ZERO;
        for (var vertex : tooth) {
            if (handle.contains(vertex) == inside) {
                total = total.add(demands[vertex]);
            }
        }
        return total;
    }

    private static int toothSum(BigDecimal vehicleCapacity,
                                BigDecimal insideDemand,
                                BigDecimal outsideDemand) {
        return minVehicles(vehicleCapacity, insideDemand) +
                minVehicles(vehicleCapacity, outsideDemand) +
                minVehicles(vehicleCapacity, insideDemand.add(outsideDemand));
    }

    private static int sizeFromVariableCount(long count) {
        var size = (int) ((1.0 + Math.sqrt(1.0 + 8.0 * count)) / 2.0);
        if ((long) size * (size - 1L) / 2L != count) {
            throw new IllegalArgumentException("Result does not contain a packed lower triangle: " + count);
        }
        return size;
    }

    private static BigDecimal clampedEdgeValue(int row, int col, Optimisation.Result result) {
        var value = getVariable_noFlip(row, col, result);
        if (value.signum() < 0) {
            return ZERO;
        }
        return value.compareTo(ONE) > 0 ? ONE : value;
    }

    private static void addBoundaryCoefficients(Set<Integer> subset,
                                                int size,
                                                Map<Edge, Integer> coefficients) {
        for (var row = 1; row < size; row++) {
            for (var col = 0; col < row; col++) {
                if (subset.contains(row) != subset.contains(col)) {
                    coefficients.merge(new Edge(row, col), 1, Integer::sum);
                }
            }
        }
    }

    private static List<Set<Integer>> connectedComponents(List<Set<Integer>> adjacency) {
        var result = new ArrayList<Set<Integer>>();
        var visited = new boolean[adjacency.size()];
        for (var start = 1; start < adjacency.size(); start++) {
            if (visited[start] || adjacency.get(start).isEmpty()) {
                continue;
            }
            var component = new HashSet<Integer>();
            var queue = new ArrayDeque<Integer>();
            queue.add(start);
            visited[start] = true;
            while (!queue.isEmpty()) {
                var vertex = queue.remove();
                component.add(vertex);
                for (var neighbour : adjacency.get(vertex)) {
                    if (!visited[neighbour]) {
                        visited[neighbour] = true;
                        queue.add(neighbour);
                    }
                }
            }
            result.add(component);
        }
        return result;
    }

    private static List<Set<Integer>> biconnectedComponents(List<Set<Integer>> adjacency) {
        var discovery = new int[adjacency.size()];
        var low = new int[adjacency.size()];
        var parent = new int[adjacency.size()];
        Arrays.fill(parent, -1);
        var time = new int[]{0};
        var stack = new ArrayDeque<Edge>();
        var result = new ArrayList<Set<Integer>>();

        for (var vertex = 1; vertex < adjacency.size(); vertex++) {
            if (discovery[vertex] == 0 && !adjacency.get(vertex).isEmpty()) {
                biconnectedDfs(vertex, adjacency, discovery, low, parent, time, stack, result);
                if (!stack.isEmpty()) {
                    result.add(popBlock(stack, null));
                }
            }
        }
        return result;
    }

    private static void biconnectedDfs(int vertex,
                                       List<Set<Integer>> adjacency,
                                       int[] discovery,
                                       int[] low,
                                       int[] parent,
                                       int[] time,
                                       Deque<Edge> stack,
                                       List<Set<Integer>> result) {
        discovery[vertex] = low[vertex] = ++time[0];
        for (var neighbour : adjacency.get(vertex).stream().sorted().toList()) {
            var edge = new Edge(vertex, neighbour);
            if (discovery[neighbour] == 0) {
                parent[neighbour] = vertex;
                stack.push(edge);
                biconnectedDfs(neighbour, adjacency, discovery, low, parent, time, stack, result);
                low[vertex] = Math.min(low[vertex], low[neighbour]);
                if (low[neighbour] >= discovery[vertex]) {
                    result.add(popBlock(stack, edge));
                }
            } else if (neighbour != parent[vertex] && discovery[neighbour] < discovery[vertex]) {
                low[vertex] = Math.min(low[vertex], discovery[neighbour]);
                stack.push(edge);
            }
        }
    }

    private static Set<Integer> popBlock(Deque<Edge> stack, Edge last) {
        var block = new HashSet<Integer>();
        while (!stack.isEmpty()) {
            var edge = stack.pop();
            block.add(edge.first());
            block.add(edge.second());
            if (last != null && edge.equals(last)) {
                break;
            }
        }
        return block;
    }

    private static void addHandle(Set<Set<Integer>> handles, Set<Integer> handle, int size) {
        var customerCount = size - 1;
        if (handle.size() >= 3 && customerCount - handle.size() >= 3) {
            handles.add(Set.copyOf(handle));
        }
    }

    private static List<Integer> sortedVertices(Set<Integer> vertices) {
        return vertices.stream().sorted().toList();
    }

    private static int compareLists(List<Integer> first, List<Integer> second) {
        for (var index = 0; index < Math.min(first.size(), second.size()); index++) {
            var comparison = Integer.compare(first.get(index), second.get(index));
            if (comparison != 0) {
                return comparison;
            }
        }
        return Integer.compare(first.size(), second.size());
    }

    private static boolean expired(long deadline) {
        return System.currentTimeMillis() >= deadline;
    }

    private record WeightedEdge(Edge edge, BigDecimal value, BigDecimal distance) {
    }

    private record HandleSearch(List<Set<Integer>> handles, boolean timedOut) {
    }

    private record Enlargement(List<Cut> cuts, boolean timedOut) {
    }

    private record Move(int vertex,
                        int tooth,
                        BigDecimal score,
                        BigDecimal boundary,
                        BigDecimal insideDemand,
                        BigDecimal outsideDemand,
                        int toothSum,
                        int combSum,
                        BigDecimal leftHandSide) implements Comparable<Move> {
        @Override
        public int compareTo(Move other) {
            var byScore = other.score.compareTo(score);
            if (byScore != 0) {
                return byScore;
            }
            var byVertex = Integer.compare(vertex, other.vertex);
            return byVertex != 0 ? byVertex : Integer.compare(tooth, other.tooth);
        }
    }
}

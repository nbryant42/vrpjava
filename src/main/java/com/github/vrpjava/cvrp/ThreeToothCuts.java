package com.github.vrpjava.cvrp;

import com.github.vrpjava.Util;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.Variable;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;

import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.getVariable_noFlip;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;
import static org.ojalgo.function.constant.BigMath.HALF;
import static org.ojalgo.optimisation.Optimisation.State.INFEASIBLE;

/**
 * Exact separation of the restricted three-tooth 2-matching inequalities described in
 * {@code CombInequalitiesV2.md}. Teeth are disjoint two-customer edges, each crossing a customer-only handle.
 */
final class ThreeToothCuts {
    private static final BigDecimal THREE = BigDecimal.valueOf(3);

    private ThreeToothCuts() {
    }

    record Edge(int first, int second) implements Comparable<Edge> {
        Edge {
            if (first == second) {
                throw new IllegalArgumentException("A tooth edge requires two distinct vertices.");
            }
            if (first > second) {
                var swap = first;
                first = second;
                second = swap;
            }
        }

        boolean contains(int vertex) {
            return first == vertex || second == vertex;
        }

        @Override
        public int compareTo(Edge other) {
            var byFirst = Integer.compare(first, other.first);
            return byFirst != 0 ? byFirst : Integer.compare(second, other.second);
        }
    }

    record Cut(Set<Integer> handle, Set<Edge> teeth) {
        Cut {
            handle = Set.copyOf(handle);
            teeth = Set.copyOf(teeth);
        }
    }

    record Separation(Optional<Cut> cut, boolean complete) {
        Separation {
            Objects.requireNonNull(cut, "cut");
        }

        private static Separation exhausted() {
            return new Separation(Optional.empty(), true);
        }

        private static Separation incomplete() {
            return new Separation(Optional.empty(), false);
        }
    }

    static Separation generate(BigDecimal vehicleCapacity,
                               BigDecimal[] demands,
                               Optimisation.Result parentResult,
                               long deadline) {
        if (demands.length < 7) {
            return Separation.exhausted();
        }
        if (deadline <= System.currentTimeMillis()) {
            return Separation.incomplete();
        }

        var subproblem = buildSubproblem(vehicleCapacity, demands, parentResult, deadline);
        if (subproblem == null) {
            return Separation.incomplete();
        }
        if (subproblem.teeth().size() < 3) {
            return Separation.exhausted();
        }

        // newModel() captured the remaining duration before model construction. Refresh it so construction consumes
        // the same total budget rather than granting the optimiser a new copy of that time.
        Util.setTimeout(deadline, subproblem.model().options);
        var result = subproblem.model().maximise();
        if (result.getState() == INFEASIBLE) {
            return Separation.exhausted();
        }
        if (!result.getState().isFeasible()) {
            return Separation.incomplete();
        }

        var candidate = extract(subproblem, result);
        if (!isStructurallyValid(candidate, vehicleCapacity, demands)) {
            return Separation.incomplete();
        }

        var violation = violation(candidate, parentResult);
        var complete = result.getState().isOptimal();
        if (violation.signum() > 0) {
            return new Separation(Optional.of(candidate), complete);
        }

        // Negative values from the parent LP are omitted from the separation objective. That is conservative for
        // finding a candidate, but a numerically inflated optimum must not be mistaken for complete separation.
        if (complete && clampedViolation(candidate, parentResult).signum() <= 0) {
            return Separation.exhausted();
        }
        return Separation.incomplete();
    }

    static BigDecimal violation(Cut cut, Optimisation.Result result) {
        return score(cut, result, false).subtract(ONE);
    }

    static boolean isStructurallyValid(Cut cut, BigDecimal vehicleCapacity, BigDecimal[] demands) {
        if (cut.teeth().size() != 3 || cut.handle().isEmpty()) {
            return false;
        }

        var handleDemand = ZERO;
        for (var vertex : cut.handle()) {
            if (vertex <= 0 || vertex >= demands.length) {
                return false;
            }
            handleDemand = handleDemand.add(demands[vertex]);
        }
        if (handleDemand.compareTo(vehicleCapacity) > 0) {
            return false;
        }

        var endpoints = new HashSet<Integer>();
        for (var tooth : cut.teeth()) {
            if (tooth.first() <= 0 || tooth.second() >= demands.length ||
                    !endpoints.add(tooth.first()) || !endpoints.add(tooth.second()) ||
                    cut.handle().contains(tooth.first()) == cut.handle().contains(tooth.second()) ||
                    demands[tooth.first()].add(demands[tooth.second()]).compareTo(vehicleCapacity) > 0) {
                return false;
            }
        }
        return true;
    }

    static void addTo(ExpressionsBasedModel model, Cut cut) {
        var handle = cut.handle();
        var name = "three-tooth cut:H=" + handle.stream().sorted().toList() + ",F=" +
                cut.teeth().stream().sorted().toList();
        var expression = model.newExpression(name).upper(handle.size() + 1L);

        for (var row : handle) {
            for (var col : handle) {
                if (col < row) {
                    expression.set(getVariable_noFlip(row, col, model), ONE);
                }
            }
        }
        for (var tooth : cut.teeth()) {
            expression.set(getVariable_noFlip(tooth.second(), tooth.first(), model), ONE);
        }
    }

    private static Subproblem buildSubproblem(BigDecimal vehicleCapacity,
                                              BigDecimal[] demands,
                                              Optimisation.Result parentResult,
                                              long deadline) {
        var model = Util.newModel(deadline);
        var size = demands.length;
        var handles = new Variable[size];

        for (var vertex = 1; vertex < size; vertex++) {
            handles[vertex] = model.newVariable("h" + vertex).binary().weight(ONE.negate());
        }

        var handleCapacity = model.newExpression("handle capacity").upper(vehicleCapacity);
        for (var vertex = 1; vertex < size; vertex++) {
            handleCapacity.set(handles[vertex], demands[vertex]);
        }

        var teeth = new LinkedHashMap<Edge, Variable>();
        var incidentTeeth = new ArrayList<List<Variable>>(size);
        for (var vertex = 0; vertex < size; vertex++) {
            incidentTeeth.add(new ArrayList<>());
        }

        for (var row = 2; row < size; row++) {
            if (deadline <= System.currentTimeMillis()) {
                return null;
            }
            for (var col = 1; col < row; col++) {
                var x = getVariable_noFlip(row, col, parentResult);
                if (x.signum() <= 0) {
                    // For a violated cut, x(F) - x(delta(H))/2 - 1 > 0. Since F is contained in delta(H),
                    // x(F) > 2; all three selected tooth edges (each bounded by one) therefore have positive
                    // support. Zero-support f variables, as well as zero-weight z variables, can be omitted exactly.
                    continue;
                }

                setUpInternalEdge(model, handles[row], handles[col], row, col, x);

                if (demands[row].add(demands[col]).compareTo(vehicleCapacity) <= 0) {
                    var edge = new Edge(row, col);
                    var tooth = model.newVariable("f" + row + "_" + col).binary().weight(x);
                    teeth.put(edge, tooth);
                    incidentTeeth.get(row).add(tooth);
                    incidentTeeth.get(col).add(tooth);

                    model.newExpression("tooth enters handle " + row + "_" + col).upper(ZERO)
                            .set(tooth, ONE).set(handles[row], ONE.negate()).set(handles[col], ONE.negate());
                    model.newExpression("tooth leaves handle " + row + "_" + col).upper(BigDecimal.valueOf(2))
                            .set(tooth, ONE).set(handles[row], ONE).set(handles[col], ONE);
                }
            }
        }

        var threeTeeth = model.newExpression("three teeth").level(THREE);
        teeth.values().forEach(tooth -> threeTeeth.set(tooth, ONE));

        for (var vertex = 1; vertex < size; vertex++) {
            var matching = model.newExpression("tooth matching " + vertex).upper(ONE);
            incidentTeeth.get(vertex).forEach(tooth -> matching.set(tooth, ONE));
        }

        return new Subproblem(model, handles, teeth);
    }

    private static void setUpInternalEdge(ExpressionsBasedModel model,
                                          Variable firstHandle,
                                          Variable secondHandle,
                                          int first,
                                          int second,
                                          BigDecimal weight) {
        var internal = model.newVariable("z" + first + "_" + second).lower(ZERO).upper(ONE).weight(weight);
        model.newExpression("internal first " + first + "_" + second).upper(ZERO)
                .set(internal, ONE).set(firstHandle, ONE.negate());
        model.newExpression("internal second " + first + "_" + second).upper(ZERO)
                .set(internal, ONE).set(secondHandle, ONE.negate());
        model.newExpression("internal lower " + first + "_" + second).lower(ONE.negate())
                .set(internal, ONE).set(firstHandle, ONE.negate()).set(secondHandle, ONE.negate());
    }

    private static Cut extract(Subproblem subproblem, Optimisation.Result result) {
        var handle = new HashSet<Integer>();
        for (var vertex = 1; vertex < subproblem.handles().length; vertex++) {
            if (selected(subproblem.model(), subproblem.handles()[vertex], result)) {
                handle.add(vertex);
            }
        }

        var teeth = new HashSet<Edge>();
        subproblem.teeth().forEach((edge, variable) -> {
            if (selected(subproblem.model(), variable, result)) {
                teeth.add(edge);
            }
        });
        return new Cut(handle, teeth);
    }

    private static boolean selected(ExpressionsBasedModel model, Variable variable, Optimisation.Result result) {
        return result.get(model.indexOf(variable)).compareTo(HALF) > 0;
    }

    private static BigDecimal clampedViolation(Cut cut, Optimisation.Result result) {
        return score(cut, result, true).subtract(ONE);
    }

    private static BigDecimal score(Cut cut, Optimisation.Result result, boolean clampNegative) {
        var score = BigDecimal.valueOf(-cut.handle().size());
        for (var row : cut.handle()) {
            for (var col : cut.handle()) {
                if (col < row) {
                    score = score.add(edgeValue(row, col, result, clampNegative));
                }
            }
        }
        for (var tooth : cut.teeth()) {
            score = score.add(edgeValue(tooth.second(), tooth.first(), result, clampNegative));
        }
        return score;
    }

    private static BigDecimal edgeValue(int row,
                                        int col,
                                        Optimisation.Result result,
                                        boolean clampNegative) {
        var value = getVariable_noFlip(row, col, result);
        return clampNegative && value.signum() < 0 ? ZERO : value;
    }

    private record Subproblem(ExpressionsBasedModel model,
                              Variable[] handles,
                              Map<Edge, Variable> teeth) {
    }
}

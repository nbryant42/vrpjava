package com.github.vrpjava.cvrp;

import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.getVariable_noFlip;
import static java.math.BigDecimal.ZERO;

/**
 * Polynomial separation of generalized large multistar (GLM) inequalities.
 *
 * <p>For a nonempty customer nucleus {@code N}, the installed inequality is:</p>
 *
 * <pre>
 * Q x(E(N)) + sum(j outside N, q[j] x(E(N:{j}))) &lt;= Q |N| - q(N).
 * </pre>
 *
 * <p>Using the customer degree equations, finding the most violated member reduces to one source-sink minimum cut.
 * The residual graph also yields a linear-size canonical family of tied minimum cuts, which are returned together so
 * that the worker can install equally strong inequalities before re-solving. The reduction requires customer edges
 * with {@code q[i] + q[j] > Q} to be fixed to zero; the base model installs those valid bounds when it creates the
 * edge variables. Every returned nucleus is nevertheless checked against the displayed inequality directly before it
 * is installed.</p>
 */
final class GeneralizedLargeMultistarCuts {
    // GLM slack scales with capacity; a normalized threshold avoids turning solver residuals into negligible cuts.
    private static final BigDecimal MIN_RELATIVE_VIOLATION = new BigDecimal("0.000001");

    private GeneralizedLargeMultistarCuts() {
    }

    record Cut(Set<Integer> nucleus) {
        Cut {
            nucleus = Set.copyOf(nucleus);
        }
    }

    record Separation(List<Cut> cuts, boolean timedOut) {
        Separation {
            cuts = cuts.stream().map(candidate -> new Cut(candidate.nucleus())).distinct().toList();
        }
    }

    static Separation generate(BigDecimal vehicleCapacity,
                               BigDecimal[] demands,
                               Optimisation.Result parentResult,
                               long deadline) {
        validateInputs(vehicleCapacity, demands, parentResult);
        if (expired(deadline)) {
            return new Separation(List.of(), true);
        }

        var customerCount = demands.length - 1;
        var source = customerCount;
        var sink = source + 1;
        var arcs = new ArrayList<DirectedMinimumCut.Arc>();
        var unary = new BigDecimal[customerCount];

        for (var customer = 1; customer < demands.length; customer++) {
            unary[customer - 1] = vehicleCapacity.subtract(demands[customer])
                    .multiply(edgeValue(customer, 0, parentResult));
        }

        for (var row = 2; row < demands.length; row++) {
            for (var col = 1; col < row; col++) {
                var value = edgeValue(row, col, parentResult);
                unary[row - 1] = unary[row - 1].subtract(demands[col].multiply(value));
                unary[col - 1] = unary[col - 1].subtract(demands[row].multiply(value));

                var capacityFactor = vehicleCapacity.subtract(demands[row]).subtract(demands[col]);
                if (capacityFactor.signum() < 0) {
                    if (value.signum() > 0) {
                        // The minimum-cut reduction has a negative capacity unless this valid base-model bound holds.
                        return new Separation(List.of(), false);
                    }
                    continue;
                }

                var capacity = capacityFactor.multiply(value);
                if (capacity.signum() > 0) {
                    arcs.add(new DirectedMinimumCut.Arc(row - 1, col - 1, capacity));
                    arcs.add(new DirectedMinimumCut.Arc(col - 1, row - 1, capacity));
                }
            }
        }

        for (var vertex = 0; vertex < customerCount; vertex++) {
            if (unary[vertex].signum() > 0) {
                arcs.add(new DirectedMinimumCut.Arc(vertex, sink, unary[vertex]));
            } else if (unary[vertex].signum() < 0) {
                arcs.add(new DirectedMinimumCut.Arc(source, vertex, unary[vertex].negate()));
            }
        }

        var minimumCut = DirectedMinimumCut.find(customerCount + 2, arcs, source, sink);
        var cuts = new ArrayList<Cut>();
        var minimumViolation = vehicleCapacity.multiply(MIN_RELATIVE_VIOLATION);
        for (var sourceSide : minimumCut.minimumSourceSides()) {
            var nucleus = new HashSet<Integer>();
            for (var vertex : sourceSide) {
                if (vertex < customerCount) {
                    nucleus.add(vertex + 1);
                }
            }

            var candidate = new Cut(nucleus);
            if (isStructurallyValid(candidate, demands) &&
                    violation(candidate, vehicleCapacity, demands, parentResult).compareTo(minimumViolation) > 0 &&
                    !cuts.contains(candidate)) {
                cuts.add(candidate);
            }
        }
        return new Separation(cuts, expired(deadline));
    }

    static boolean isStructurallyValid(Cut cut, BigDecimal[] demands) {
        if (cut.nucleus().isEmpty()) {
            return false;
        }
        for (var vertex : cut.nucleus()) {
            if (vertex <= 0 || vertex >= demands.length) {
                return false;
            }
        }
        return true;
    }

    /** Positive means that the current relaxation violates the cut. */
    static BigDecimal violation(Cut cut,
                                BigDecimal vehicleCapacity,
                                BigDecimal[] demands,
                                Optimisation.Result result) {
        var leftHandSide = ZERO;
        var nucleusDemand = ZERO;

        for (var vertex : cut.nucleus()) {
            nucleusDemand = nucleusDemand.add(demands[vertex]);
        }

        for (var row = 2; row < demands.length; row++) {
            for (var col = 1; col < row; col++) {
                var rowInside = cut.nucleus().contains(row);
                var colInside = cut.nucleus().contains(col);
                if (rowInside && colInside) {
                    leftHandSide = leftHandSide.add(
                            vehicleCapacity.multiply(getVariable_noFlip(row, col, result)));
                } else if (rowInside != colInside) {
                    var outside = rowInside ? col : row;
                    leftHandSide = leftHandSide.add(
                            demands[outside].multiply(getVariable_noFlip(row, col, result)));
                }
            }
        }

        var rightHandSide = vehicleCapacity.multiply(BigDecimal.valueOf(cut.nucleus().size()))
                .subtract(nucleusDemand);
        return leftHandSide.subtract(rightHandSide);
    }

    static void addTo(ExpressionsBasedModel model,
                      Cut cut,
                      BigDecimal vehicleCapacity,
                      BigDecimal[] demands) {
        if (!isStructurallyValid(cut, demands)) {
            throw new IllegalArgumentException("Invalid generalized-large-multistar nucleus: " + cut.nucleus());
        }

        var sortedNucleus = cut.nucleus().stream().sorted().toList();
        var nucleusDemand = sortedNucleus.stream().map(vertex -> demands[vertex]).reduce(ZERO, BigDecimal::add);
        var rightHandSide = vehicleCapacity.multiply(BigDecimal.valueOf(sortedNucleus.size()))
                .subtract(nucleusDemand);
        var expression = model.newExpression("GLM cut:N=" + sortedNucleus).upper(rightHandSide);

        for (var row = 2; row < demands.length; row++) {
            for (var col = 1; col < row; col++) {
                var rowInside = cut.nucleus().contains(row);
                var colInside = cut.nucleus().contains(col);
                if (rowInside && colInside) {
                    expression.set(getVariable_noFlip(row, col, model), vehicleCapacity);
                } else if (rowInside != colInside) {
                    var outside = rowInside ? col : row;
                    expression.set(getVariable_noFlip(row, col, model), demands[outside]);
                }
            }
        }
    }

    private static BigDecimal edgeValue(int first, int second, Optimisation.Result result) {
        var value = getVariable_noFlip(Math.max(first, second), Math.min(first, second), result);
        return value.signum() < 0 ? ZERO : value;
    }

    private static void validateInputs(BigDecimal vehicleCapacity,
                                       BigDecimal[] demands,
                                       Optimisation.Result parentResult) {
        if (vehicleCapacity == null || vehicleCapacity.signum() <= 0) {
            throw new IllegalArgumentException("vehicleCapacity must be positive.");
        }
        if (demands == null || demands.length < 2) {
            throw new IllegalArgumentException("At least one customer is required.");
        }
        var expectedVariables = (long) demands.length * (demands.length - 1L) / 2L;
        if (parentResult == null || parentResult.count() != expectedVariables) {
            throw new IllegalArgumentException("Result does not contain the expected packed lower triangle.");
        }
    }

    private static boolean expired(long deadline) {
        return System.currentTimeMillis() >= deadline;
    }
}

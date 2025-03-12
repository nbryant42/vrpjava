package org.example;

import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.Variable;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.AbstractMap.SimpleImmutableEntry;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.LongStream;

import static com.google.common.collect.MoreCollectors.onlyElement;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
import static java.util.Arrays.*;
import static java.util.stream.Collectors.toSet;
import static org.example.CutCandidates.round;
import static org.example.SubtourCuts.formatCut;
import static org.example.Util.isLowerTriangular;
import static org.example.Util.newModel;
import static org.example.Util.setTimeout;
import static org.ojalgo.optimisation.Optimisation.State.OPTIMAL;

/**
 * Solver for the Capacitated Vehicle Routing Problem (CVRP).
 * <p>
 * This is a depth-first, lazily bounded, branch-and-bound-and-cut exact algorithm based on:
 *
 * <ul>
 *     <li>The ojAlgo solver (which has available interfaces to commercial solvers)</li>
 *     <li>The Two-Index Vehicle Flow Formulation<li>
 *     <li>The Pavlikov-Petersen-Sørensen "RCC-Sep" algorithm (for both lower bounds and row generation.)</li>
 * </ul>
 * <p>
 * Currently only supports symmetric problems, and therefore requires that the cost matrix is lower-triangular.
 * In the future, if support for the asymmetric problem is implemented, the plan is to detect whether the cost
 * matrix is lower-triangular, and adapt accordingly.
 *
 * @see CVRPSolver for more usage details.
 * @see RccSepCVRPCuts for the RCC-Sep subroutine.
 * @see <a href="https://repub.eur.nl/pub/135594/EI2021-01.pdf">This PDF</a> for proofs on the LP bound.
 * @see <a href="https://onlinelibrary.wiley.com/doi/10.1002/net.22183">The RCC-Sep paper.</a>
 */
public class OjAlgoCVRPSolver extends CVRPSolver {
    public record Cut(int minVehicles, Set<Integer> subset) {
        Cut(int size, Optimisation.Result result) {
            this(alpha(result), IntStream.range(1, size)
                    .filter(i -> result.get(i).signum() > 0)
                    .boxed()
                    .collect(Collectors.toSet()));
        }

        private static int alpha(Optimisation.Result result) {
            var alpha = result.get(0).add(ONE);
            try {
                return round(alpha).intValueExact();
            } catch (ArithmeticException e) {
                throw new ArithmeticException(e.getMessage() + ": " + alpha);
            }
        }
    }

    record ModelResult(ExpressionsBasedModel model, Optimisation.Result result) {
    }

    ModelResult initBounds(int minVehicles,
                           int maxVehicles,
                           BigDecimal vehicleCapacity,
                           BigDecimal[] demands,
                           BigDecimal[][] costMatrix,
                           long timeout) {
        var deadline = System.currentTimeMillis() + timeout;
        var model = newModel(deadline);
        var vars = buildVars(costMatrix, model);

        buildConstraints(model, minVehicles, maxVehicles, vars);
        model.relax();

        return new ModelResult(model, updateBounds(vehicleCapacity, demands, model, demands.length, deadline));
    }

    private static Optimisation.Result updateBounds(BigDecimal vehicleCapacity,
                                                    BigDecimal[] demands,
                                                    ExpressionsBasedModel model,
                                                    int size,
                                                    long deadline) {
        //var iter = 1;
        setTimeout(deadline, model.options);
        var result = minimize(model);
        var cuts = new HashSet<Set<Integer>>();

        while (result.getState() == OPTIMAL && System.currentTimeMillis() < deadline) {
            setTimeout(deadline, model.options);
            var rccCuts = RccSepCVRPCuts.generate(vehicleCapacity, demands, result, deadline);

            if (addCuts(rccCuts, cuts, model, result, "cut: ", size)) {
                result = minimize(model);
                continue;
            }
            var subtourCuts = SubtourCuts.generate(vehicleCapacity, demands, result);

            if (addCuts(subtourCuts, cuts, model, result, "subtour cut: ", size)) {
                result = minimize(model);
                continue;
            }

            // no more cuts to add. done.
            break;
        }

        // done or timed out.
        //System.out.println(iter + " iterations, " + cuts.size() + " cuts");
        return result;
    }

    private static Optimisation.Result weakUpdateBounds(BigDecimal vehicleCapacity,
                                                        BigDecimal[] demands,
                                                        ExpressionsBasedModel model,
                                                        int size,
                                                        long deadline) {
        //var iter = 1;
        setTimeout(deadline, model.options);
        var result = minimize(model);
        var cuts = new HashSet<Set<Integer>>();

        while (result.getState() == OPTIMAL && System.currentTimeMillis() < deadline) {
            setTimeout(deadline, model.options);

            var subtourCuts = SubtourCuts.generate(vehicleCapacity, demands, result);

            if (addCuts(subtourCuts, cuts, model, result, "subtour cut: ", size)) {
                result = minimize(model);
                continue;
            }

            // no more cuts to add. done.
            break;
        }

        //System.out.println(iter + " iterations, " + cuts.size() + " cuts");

        // done or timed out. ( TODO: review/unit test for timeout handling here and throughout.)
        // if there are no fractional variables, this is a candidate solution, but we don't know for sure until we've
        // validated that it satisfies the full set of constraints, so iterate on additional cuts.
        // This needs to be done on a fast path, so don't run the RCC-Sep ILP model unless confirmed invalid.
        return isInvalidIntegerSolution(vehicleCapacity, demands, result) ?
                updateBounds(vehicleCapacity, demands, model, size, deadline) :
                result;
    }

    private record Node(ExpressionsBasedModel model, Optimisation.Result result) {
        private Node(ModelResult mr) {
            this(mr.model(), mr.result());
        }
    }

    @Override
    protected Result doSolve(int minVehicles,
                             int maxVehicles,
                             BigDecimal vehicleCapacity,
                             BigDecimal[] demands,
                             BigDecimal[][] costMatrix,
                             long timeout) {
        var size = costMatrix.length;
        var start = System.currentTimeMillis();
        var deadline = start + timeout;
        var globalBounds = initBounds(minVehicles, maxVehicles, vehicleCapacity, demands, costMatrix, timeout);

        System.out.println("Bounds init complete after " + (System.currentTimeMillis() - start) + "ms");

        var stack = new ArrayList<Node>();
        var nodes = 0;
        Optimisation.Result incumbent = null;

        stack.add(new Node(globalBounds));

        for (; !stack.isEmpty(); nodes++) {
            var node = stack.removeLast();

            // decide which variable to branch on: find the variable with the smallest rounding
            // gap to the nearest integer.
            var frac = IntStream.range(0, (int) node.result().count())
                    .mapToObj(i -> {
                        var v = node.result().get(i);
                        return new SimpleImmutableEntry<>(i, v.setScale(0, RoundingMode.HALF_EVEN).subtract(v).abs());
                    })
                    .filter(entry -> entry.getValue().signum() > 0)
                    .min(Entry.comparingByValue());

            if (frac.isPresent()) {
                var k = frac.get().getKey();
                var v = node.result().get(k);
                var closest = v.setScale(0, RoundingMode.HALF_EVEN);
                var other = closest.compareTo(v) > 0 ? closest.subtract(ONE) : closest.add(ONE);

                // push two child nodes with variable fixed to `closest` and `other`.
                var model1 = node.model().copy(true, false);
                model1.getVariable(k).level(closest);
                var result1 = weakUpdateBounds(vehicleCapacity, demands, model1, size, deadline);

                var model2 = node.model().copy(true, false);
                model2.getVariable(k).level(other);
                var result2 = weakUpdateBounds(vehicleCapacity, demands, model2, size, deadline);

                var node1 = new Node(model1, result1);
                var node2 = new Node(model2, result2);

                // the greater bound is the worst result. push the best result last, so it's on
                // top of stack.
                pushOrFathom(stack, incumbent,
                        result1.getValue() > result2.getValue() ? new Node[]{node1, node2} : new Node[]{node2, node1});

            } else {
                double lb = globalBounds.result().getValue();
                double ub = node.result().getValue();

                if (incumbent == null || ub < incumbent.getValue()) {
                    var seconds = BigDecimal.valueOf(System.currentTimeMillis() - start)
                            .divide(BigDecimal.valueOf(1000L), 3, RoundingMode.HALF_EVEN);

                    System.out.println("[" + seconds + "s]: New incumbent. Bounds now " + (float) lb + '/' + (float) ub
                            + " (" + BigDecimal.valueOf(lb / ub * 100.0).setScale(2, RoundingMode.HALF_EVEN) + "%)");

                    incumbent = node.result();

                    if (ub <= lb) {
                        break;
                    }
                }
            }
        }

        if (incumbent == null) {
            System.out.println(nodes + " nodes, no solution.");
            return null;
        }

        var cycles = findCycles(size, incumbent);

        System.out.println(nodes + " nodes, " + cycles.size() + " cycles: " + cycles);
        var cycleDemands = cycles.stream()
                .map(cycle -> cycle.stream().map(i -> demands[i]).reduce(ZERO, BigDecimal::add))
                .toList();
        System.out.println("Cycle demands: " + cycleDemands);

        return new Result(incumbent.getValue(), cycles);
    }

    // Warning -- this does not check for sub-tours -- that's assumed to be handled elsewhere.
    private static boolean isInvalidIntegerSolution(BigDecimal vehicleCapacity,
                                                    BigDecimal[] demands,
                                                    Optimisation.Result result) {
        return isIntegerSolution(result) && findCycles(demands.length, result).stream().anyMatch(cycle ->
                cycle.stream().map(i -> demands[i])
                        .reduce(ZERO, BigDecimal::add)
                        .compareTo(vehicleCapacity) > 0);
    }

    private static boolean isIntegerSolution(Optimisation.Result result) {
        return LongStream.range(0, result.count()).allMatch(i -> {
            var v = result.get(i);
            return v.setScale(0, RoundingMode.HALF_EVEN).compareTo(v) == 0;
        });
    }

    private static void pushOrFathom(List<Node> stack, Optimisation.Result incumbent, Node... nodes) {
        stream(nodes).filter(n -> incumbent == null || n.result().getValue() <= incumbent.getValue())
                .forEachOrdered(stack::add);
    }

    // TODO get rid of this. Keeping it around for now because it's still a bit faster than B&B sometimes.
    Result iterativeDeepeningSolver(int minVehicles,
                                    int maxVehicles,
                                    BigDecimal vehicleCapacity,
                                    BigDecimal[] demands,
                                    BigDecimal[][] costMatrix,
                                    long timeout) {
        if (!isLowerTriangular(costMatrix)) {
            throw new UnsupportedOperationException("costMatrix must be lower-triangular.");
        }
        var deadline = System.currentTimeMillis() + timeout;
        var model = newModel(deadline);
        var vars = buildVars(costMatrix, model);
        var cuts = new HashSet<Set<Integer>>();

        buildConstraints(model, minVehicles, maxVehicles, vars);

        // solve

        var result = minimize(model.snapshot());

        for (var iter = 1; result.getState() == OPTIMAL && System.currentTimeMillis() < deadline; iter++) {
            setTimeout(deadline, model.options);
            var size = costMatrix.length;
            var rccCuts = RccSepCVRPCuts.generate(vehicleCapacity, demands, result, deadline);

            if (addCuts(rccCuts, cuts, model, result, "cut: ", vars.length)) {
                result = minimize(model.snapshot());
                continue;
            }

            var subtourCuts = SubtourCuts.generate(vehicleCapacity, demands, result);

            if (addCuts(subtourCuts, cuts, model, result, "subtour cut: ", vars.length)) {
                result = minimize(model.snapshot());
                continue;
            }

            // no more cuts to add. done.

            List<List<Integer>> cycles;
            try {
                cycles = findCycles(size, result);
            } catch (IllegalArgumentException e) {
                System.out.println("Solution is not integer. Re-solving.");
                result = minimize(model);
                continue;
            }

            System.out.println(iter + " iterations, " + cuts.size() + " cuts, " + cycles.size() + " cycles: " + cycles);
            var cycleDemands = cycles.stream()
                    .map(cycle -> cycle.stream().map(i -> demands[i]).reduce(ZERO, BigDecimal::add))
                    .toList();
            System.out.println("Cycle demands: " + cycleDemands);

            return new Result(result.getValue(), cycles);
        }

        return null;
    }

    private static Boolean addCuts(Collection<Cut> candidates,
                                   Set<Set<Integer>> cuts,
                                   ExpressionsBasedModel model,
                                   Optimisation.Result result,
                                   String label,
                                   int size) {
        return candidates.stream().map(cut -> addCut(cuts, model, result, cut, label, size))
                .reduce(false, (a, b) -> a || b);
    }

    private static boolean addCut(Set<Set<Integer>> cuts,
                                  ExpressionsBasedModel model,
                                  Optimisation.Result result,
                                  Cut cut,
                                  String label,
                                  int size) {
        var subset = cut.subset();
        subset.remove(0);

        if (isViolated(result, size, subset, cut.minVehicles()) && cuts.add(subset)) {
            addCut(size, model, subset, cut.minVehicles(), label + formatCut(subset));
            return true;
        }
        return false;
    }

    private static void addCut(int size,
                               ExpressionsBasedModel model,
                               Set<Integer> subset,
                               int minVehicles,
                               String name) {
        var min = minVehicles * 2L;

        // TODO I know we're adding redundant cuts during B&B search.
        // suppressing this logging for now.
        //System.out.println("Adding " + name + " >= " + min);

        var cut = model.newExpression(name).lower(min);

        for (var target : subset) {
            for (var row = 1; row < size; row++) {
                for (var col = 0; col < row; col++) {
                    if (target == row && !subset.contains(col) ||
                            target == col && !subset.contains(row)) {
                        cut.set(getVariable(row, col, model), ONE);
                    }
                }
            }
        }
    }

    static boolean isViolated(Optimisation.Result result,
                              int size,
                              Set<Integer> subset,
                              int minVehicles) {
        var min = minVehicles * 2L;
        var total = ZERO;

        for (var target : subset) {
            for (var row = 1; row < size; row++) {
                for (var col = 0; col < row; col++) {
                    if (target == row && !subset.contains(col) ||
                            target == col && !subset.contains(row)) {
                        total = total.add(getVariable(row, col, result));
                    }
                }
            }
        }

        return total.compareTo(BigDecimal.valueOf(min)) < 0;
    }

    static Optimisation.Result minimize(ExpressionsBasedModel model) {
        // var start = System.currentTimeMillis();
        return model.minimise();

        //System.out.println("Elapsed: " + (System.currentTimeMillis() - start) + "ms; " + result);
        //return result;
    }

    private record Target(int node, BigDecimal count) {
        @Override
        public String toString() {
            return count.compareTo(ONE) == 0 ? Integer.toString(node) : (node + "x" + count);
        }
    }

    /**
     * Identify the arcs that run to/from a specific node
     */
    private static List<Target> arcsFrom(int node, int size, Optimisation.Result result) {
        var arcs = new ArrayList<Target>();

        IntStream.range(0, size).filter(i -> i != node).forEachOrdered(i -> {
            var count = getVariable(node, i, result);
            if (count.signum() > 0) {
                arcs.add(new Target(i, count));
            }
        });
        return arcs;
    }

    /**
     * Get a variable from the result. This will convert the row/column address to a linear address,
     * and flip row/col if necessary to ensure symmetry for a packed lower-triangular matrix.
     */
    static BigDecimal getVariable(int row, int col, Optimisation.Result result) {
        if (row == col || row < 0) {
            throw new IllegalArgumentException(Integer.toString(row));
        }
        if (col < 0) {
            throw new IllegalArgumentException(Integer.toString(col));
        }
        if (row < col) {
            var tmp = col;
            col = row;
            row = tmp;
        }
        return result.get(base(row) + col);
    }

    /**
     * Get a {@link Variable} from the model. This will convert the row/column address to a linear address,
     * and flip row/col if necessary to ensure symmetry for a packed lower-triangular matrix.
     */
    static Variable getVariable(int row, int col, ExpressionsBasedModel model) {
        if (row == col || row < 0) {
            throw new IllegalArgumentException(Integer.toString(row));
        }
        if (col < 0) {
            throw new IllegalArgumentException(Integer.toString(col));
        }
        if (row < col) {
            var tmp = col;
            col = row;
            row = tmp;
        }
        return model.getVariable(base(row) + col);
    }

    /**
     * @return The base offset for the row, for the purpose of mapping a row/col address in the square cost matrix
     * to a linear variable offset in the flat array of vars.
     * <p>
     * Undefined for rows < 1. Assumes a lower-triangle matrix for symmetrical problems.
     */
    static int base(int row) {
        var side = row + 1;
        return (side * side - side) / 2 - row;
    }

    static List<List<Integer>> findCycles(int size, Optimisation.Result result) {
        var cycles = new ArrayList<List<Integer>>();
        var depotArcs = arcsFrom(0, size, result);
        var remaining = IntStream.range(1, size).boxed().collect(toSet());
        var seen = new HashSet<Integer>();

        for (var depotArc : depotArcs) {
            var node = depotArc.node();

            if (seen.contains(node)) {
                continue;
            }

            var cycle = newCycle(node);
            remaining.remove(node);

            if (depotArc.count().compareTo(ONE) == 0) {
                // nontrivial cycle.
                var previous = 0;

                while (true) {
                    int nextNode;
                    try {
                        nextNode = getNextNode(node, previous, size, result).node();
                    } catch (IllegalArgumentException e) {
                        throw new IllegalArgumentException("Current cycle: " + cycle + ". " + e.getMessage());
                    }

                    if (nextNode == 0) {
                        break;
                    }

                    seen.add(nextNode);
                    remaining.remove(nextNode);
                    cycle.add(nextNode);
                    previous = node;
                    node = nextNode;
                }
            }

            cycles.add(cycle);
        }

        findRemainingCycles(size, result, cycles, remaining);

        return cycles;
    }

    /**
     * Like {@link #findCycles(int, Optimisation.Result)},
     * but also detects subsets connected by fractional variables.
     */
    static List<Set<Integer>> findFractionalCycles(int size, Optimisation.Result result) {
        var cycles = new ArrayList<Set<Integer>>();
        var depotArcs = arcsFrom(0, size, result);
        var remaining = IntStream.range(1, size).boxed().collect(toSet());
        var seen = new HashSet<Integer>();

        for (var depotArc : depotArcs) {
            if (seen.contains(depotArc.node())) {
                continue;
            }

            var stack = new ArrayList<Integer>();
            stack.add(depotArc.node());

            var cycle = new HashSet<Integer>();
            cycle.add(0);
            cycle.add(depotArc.node());

            remaining.remove(depotArc.node());

            if (depotArc.count().compareTo(TWO) < 0) {
                // nontrivial cycle.
                do {
                    var node = stack.removeLast();

                    if (node != 0) {
                        seen.add(node);
                        cycle.add(node);
                        stack.addAll(getNextNodes(cycle, node, size, result));
                        remaining.remove(node);
                    }
                } while (!stack.isEmpty());
            }

            cycles.add(cycle);
        }

        findRemainingFractionalCycles(size, result, cycles, remaining);

        return cycles;
    }

    private static void findRemainingCycles(int size,
                                            Optimisation.Result result,
                                            List<List<Integer>> cycles,
                                            Set<Integer> remaining) {
        int src;

        while ((src = pop(remaining)) >= 0) {
            var cycleNodes = new ArrayList<Integer>();
            findCycleNodes(size, result, remaining, cycleNodes, src);

            cycles.add(cycleNodes);
        }
    }

    // TODO this will sometimes crash due to conflicts
    private static void findRemainingFractionalCycles(int size,
                                                      Optimisation.Result result,
                                                      List<Set<Integer>> cycles,
                                                      Set<Integer> remaining) {
        int src;

        while ((src = pop(remaining)) >= 0) {
            var cycleNodes = new HashSet<Integer>();
            findCycleNodes(size, result, remaining, cycleNodes, src);

            cycles.add(cycleNodes);
        }
    }

    private static void findCycleNodes(int size,
                                       Optimisation.Result result,
                                       Set<Integer> remaining,
                                       Collection<Integer> cycleNodes,
                                       int src) {
        var visited = new HashSet<Integer>();

        cycleNodes.add(src);
        visited.add(src);

        var dest = pickNode(src, size, result).node();
        var previous = src;

        src = dest;
        cycleNodes.add(dest);
        visited.add(dest);
        remaining.remove(dest);

        while (true) {
            dest = getNextNode(src, previous, size, result).node();
            remaining.remove(dest);
            if (visited.add(dest)) {
                cycleNodes.add(dest);
            } else {
                break;
            }
            previous = src;
            src = dest;
        }
    }

    private static List<Integer> newCycle(int node) {
        var cycle = new ArrayList<Integer>();
        cycle.add(0);
        cycle.add(node);
        return cycle;
    }

    private static Target getNextNode(int node, int previous, int size, Optimisation.Result result) {
        try {
            return arcsFrom(node, size, result).stream().filter(t -> t.node != previous).collect(onlyElement());
        } catch (IllegalArgumentException e) {
            throw new IllegalArgumentException("arcs from " + node + " excluding " + previous + ": " + e.getMessage());
        }
    }

    private static List<Integer> getNextNodes(Set<Integer> cycle, int node, int size, Optimisation.Result result) {
        return arcsFrom(node, size, result).stream().map(Target::node).filter(n -> !cycle.contains(n)).toList();
    }

    private static Target pickNode(int node, int size, Optimisation.Result result) {
        return arcsFrom(node, size, result).getFirst();
    }

    // assumes that c's iterator supports remove()
    // performs best on collections that support a constant-time remove op.
    private static int pop(Collection<Integer> c) {
        var it = c.iterator();
        if (it.hasNext()) {
            var next = it.next();
            it.remove();
            return next;
        }
        return -1;
    }

    private static void buildConstraints(ExpressionsBasedModel model,
                                         int minVehicles,
                                         int maxVehicles,
                                         Variable[][] vars) {
        var size = vars.length;

        var vcConstr = model.newExpression("vehicleCount").lower(2L * minVehicles).upper(2L * maxVehicles);
        for (var i = 1; i < size; i++) {
            vcConstr.set(vars[i][0], ONE);
        }

        // In contrast to https://repub.eur.nl/pub/135594/EI2021-01.pdf, all our matrices
        // are lower-triangular, so the subscripts are reversed vs. constraint (2) on page 3.
        for (var k = 1; k < size; k++) {
            var inout = model.newExpression("in_out_" + k).level(TWO);
            for (var i = 0; i < k; i++) {
                inout.set(vars[k][i], ONE);
            }
            for (var j = k + 1; j < size; j++) {
                inout.set(vars[j][k], ONE);
            }
        }
    }

    private static Variable[][] buildVars(BigDecimal[][] costMatrix, ExpressionsBasedModel model) {
        var size = costMatrix.length;
        var vars = new Variable[size][size];

        for (var row = 1; row < size; row++) {
            var varsRow = vars[row];
            var costRow = costMatrix[row];

            for (var col = 0; col < row; col++) {
                varsRow[col] = model.newVariable("x" + row + "_" + col)
                        .weight(costRow[col])
                        .lower(ZERO)
                        .upper(col == 0 ? TWO : ONE)
                        .integer();
            }
        }
        return vars;
    }
}

package com.github.vrpjava.cvrp;

import org.ojalgo.netio.BasicLogger;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.Variable;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static com.github.vrpjava.Util.setTimeout;
import static com.github.vrpjava.cvrp.CutCandidates.round;
import static com.github.vrpjava.cvrp.SubtourCuts.formatCut;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
import static java.util.stream.Collectors.toSet;

/**
 * Solver for the Capacitated Vehicle Routing Problem (CVRP).
 * <p>
 * This is a lazily bounded, branch-and-bound-and-cut exact algorithm based on:
 *
 * <ul>
 *     <li>The ojAlgo solver (which has available interfaces to commercial solvers)</li>
 *     <li>The Two-Index Vehicle Flow Formulation</li>
 *     <li>The Pavlikov-Petersen-Sørensen "RCC-Sep" algorithm (for both lower bounds and row generation.)</li>
 * </ul>
 * <p>
 * Currently only supports symmetric problems, and therefore requires that the cost matrix is lower-triangular.
 * In the future, if support for the asymmetric problem is implemented, the plan is to detect whether the cost
 * matrix is lower-triangular, and adapt accordingly.
 *
 * @see CVRPSolver for more usage details
 * @see RccSepCVRPCuts for the RCC-Sep subroutine
 * @see <a href="https://repub.eur.nl/pub/135594/EI2021-01.pdf">This PDF</a> for proofs on the LP bound
 * @see <a href="https://onlinelibrary.wiley.com/doi/10.1002/net.22183">The RCC-Sep paper</a>
 */
public class OjAlgoCVRPSolver extends CVRPSolver {
    private boolean debug;

    /**
     * Default constructor.
     */
    public OjAlgoCVRPSolver() {
    }

    private double bestFirstRatio = 0.95;
    private long bestFirstMillis = 30_000L;
    private CVRPSolver heuristic = new ClarkeWrightCVRPSolver();

    record Cut(int minVehicles, Set<Integer> subset) {
        Cut(int size, Optimisation.Result result) {
            this(alpha(result), IntStream.range(1, size)
                    .filter(i -> result.get(i).signum() > 0)
                    .boxed()
                    .collect(Collectors.toSet()));
        }

        private static int alpha(Optimisation.Result result) {
            return round(result.get(0).add(ONE)).intValueExact();
        }
    }

    @Override
    protected Result doSolve(int minVehicles,
                             int maxVehicles,
                             BigDecimal vehicleCapacity,
                             BigDecimal[] demands,
                             BigDecimal[][] costMatrix,
                             long timeout) {
        return new Job(this, minVehicles, maxVehicles, vehicleCapacity, demands, costMatrix, timeout).run();
    }

    void debug(String s) {
        if (debug) {
            BasicLogger.debug(s);
        }
    }

    static BigDecimal toSeconds(long val) {
        return BigDecimal.valueOf(val).divide(BigDecimal.valueOf(1000L), 3, RoundingMode.HALF_EVEN);
    }

    static boolean addCuts(Collection<Cut> candidates,
                           Set<Set<Integer>> cuts,
                           ExpressionsBasedModel model,
                           Optimisation.Result result,
                           Job job,
                           int size) {
        return candidates.stream().map(cut -> addCut(cuts, model, result, job, cut, size))
                .reduce(false, (a, b) -> a || b);
    }

    private static boolean addCut(Set<Set<Integer>> cuts,
                                  ExpressionsBasedModel model,
                                  Optimisation.Result result,
                                  Job job,
                                  Cut cut,
                                  int size) {
        var subset = cut.subset();
        subset.remove(0);

        if (isViolated(result, size, subset, cut.minVehicles()) && cuts.add(subset)) {
            addCut(size, model, job, subset, cut.minVehicles());
            return true;
        }
        return false;
    }

    // add a cut to the node-local model, and also propagate it to the global model
    // if `globalBounds` is not null.
    private static void addCut(int size,
                               ExpressionsBasedModel model,
                               Job job,
                               Set<Integer> subset,
                               int minVehicles) {
        var name = formatCut(subset);
        var min = minVehicles * 2L;

        //debug("Adding " + name + " >= " + min);

        addCut(size, model, subset, name, min);
        if (job != null) {
            job.addCut(subset, name, min);
        }
    }

    static void addCut(int size, ExpressionsBasedModel model, Set<Integer> subset, String name, long min) {
        var cut = model.newExpression(name).lower(min);

        for (var target : subset) {
            for (var row = 1; row < size; row++) {
                for (var col = 0; col < row; col++) {
                    if (target == row && !subset.contains(col) ||
                            target == col && !subset.contains(row)) {
                        cut.set(getVariable_noFlip(row, col, model), ONE);
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
                        total = total.add(getVariable_noFlip(row, col, result));
                    }
                }
            }
        }

        return total.compareTo(BigDecimal.valueOf(min)) < 0;
    }

    static Optimisation.Result minimize(ExpressionsBasedModel model, long deadline) {
        setTimeout(deadline, model.options);
        return model.minimise();
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
        return row < col ? result.get(base(col) + row) : result.get(base(row) + col);
    }

    /**
     * Get a variable from the result. This will convert the row/column address to a linear address,
     * but will NOT flip row/col; use this variant only when you are certain that `row > col`
     */
    static BigDecimal getVariable_noFlip(int row, int col, Optimisation.Result result) {
        return result.get(base(row) + col);
    }

    /**
     * Get a {@link Variable} from the model. This will convert the row/column address to a linear address,
     * but will NOT flip row/col; use this variant only when you are certain that `row > col`
     */
    private static Variable getVariable_noFlip(int row, int col, ExpressionsBasedModel model) {
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

    static Set<List<Integer>> findCycles(int size, Optimisation.Result result) {
        var cycles = new HashSet<List<Integer>>();
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

                for (int nextNode; (nextNode = getNextNode(node, previous, size, result).node()) != 0; ) {
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
                    seen.add(node);
                    cycle.add(node);
                    arcsFrom(node, size, result).stream().map(Target::node).filter(n -> !cycle.contains(n))
                            .forEach(stack::add);
                    remaining.remove(node);
                } while (!stack.isEmpty());
            }

            cycles.add(cycle);
        }

        findRemainingFractionalCycles(size, result, cycles, remaining);

        return cycles;
    }

    private static void findRemainingCycles(int size,
                                            Optimisation.Result result,
                                            Set<List<Integer>> cycles,
                                            Set<Integer> remaining) {
        int src;

        while ((src = pop(remaining)) >= 0) {
            var cycleNodes = new ArrayList<Integer>();
            findCycleNodes(size, result, remaining, cycleNodes, src);

            cycles.add(cycleNodes);
        }
    }

    private static void findRemainingFractionalCycles(int size,
                                                      Optimisation.Result result,
                                                      List<Set<Integer>> cycles,
                                                      Set<Integer> remaining) {
        for (int src; (src = pop(remaining)) > 0; ) {
            var cycle = new HashSet<Integer>();
            var toAdd = new HashSet<Integer>();

            do {
                arcsFrom(src, size, result).stream().map(Target::node).filter(n -> !cycle.contains(n))
                        .forEach(toAdd::add);
                cycle.add(src);
                remaining.remove(src);
                src = pop(toAdd);
            } while (src > 0);

            cycles.add(cycle);
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
        var nextNodes = arcsFrom(node, size, result).stream().filter(t -> t.node != previous).toList();

        return switch (nextNodes.size()) {
            case 0 -> throw new NoSuchElementException();
            case 1 -> nextNodes.getFirst();
            default -> throw new IllegalArgumentException("more than 1 arcs from " + node + " excluding " + previous +
                    ": " + nextNodes);
        };
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

    static void buildConstraints(ExpressionsBasedModel model,
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

    static Variable[][] buildVars(BigDecimal[][] costMatrix, ExpressionsBasedModel model) {
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

    /**
     * Get the currently configured ratio threshold for "hard" problems.
     *
     * @return value as a ratio from lower to upper bound
     * @see #setBestFirstRatio(double)
     */
    @SuppressWarnings("unused")
    public double getBestFirstRatio() {
        return bestFirstRatio;
    }

    /**
     * Get the currently configured threshold time for "hard" problems.
     *
     * @return value in milliseconds
     * @see #setBestFirstMillis(long)
     */
    @SuppressWarnings("unused")
    public long getBestFirstMillis() {
        return bestFirstMillis;
    }

    /**
     * Set the minimum bounds ratio before switching to best-first search. The consequences of getting this tunable
     * wrong are asymmetrical: best-first search will likely ruin any possibility of getting even an approximate result
     * for the hard problems, but depth-first search will greatly slow down the easy problems.
     * <p>
     * There is no single ratio threshold that works best for both cases. Therefore, this works in conjunction with
     * {@link #setBestFirstMillis(long)}, and the switch does not occur unless both the ratio and a maximum time limit
     * are satisfied. (If it's already taken too long, we know we're working on a hard problem instance.)
     *
     * @param bestFirstRatio the minimum ratio.
     */
    @SuppressWarnings("unused")
    public void setBestFirstRatio(double bestFirstRatio) {
        this.bestFirstRatio = bestFirstRatio;
    }

    /**
     * Used in conjunction with {@link #setBestFirstRatio(double)}
     *
     * @param bestFirstMillis milliseconds after which the problem is considered "hard", and we no longer consider
     *                        switching to best-first search.
     */
    @SuppressWarnings("unused")
    public void setBestFirstMillis(long bestFirstMillis) {
        this.bestFirstMillis = bestFirstMillis;
    }

    /**
     * Get the currently configured heuristic implementation.
     *
     * @return {@link CVRPSolver}
     * @see #setHeuristic(CVRPSolver)
     */
    @SuppressWarnings("unused")
    public CVRPSolver getHeuristic() {
        return heuristic;
    }

    /**
     * Set the implementation of {@link CVRPSolver} that will be used to determine the heuristic starting point.
     * <p>
     * A starting point helps improve the upper bound and eliminate nodes searched by the branch-and-bound process,
     * so this is required. The default is {@link NearestNeighborCVRPSolver}.
     * If you have a better heuristic, set it here.
     *
     * @param heuristic any subclass of {@link CVRPSolver} (but not this instance!)
     */
    @SuppressWarnings("unused")
    public void setHeuristic(CVRPSolver heuristic) {
        this.heuristic = heuristic;
    }

    /**
     * Get the debug property
     *
     * @return true if debug logging is enabled
     */
    @SuppressWarnings("unused")
    public boolean isDebug() {
        return debug;
    }

    /**
     * Set the debug property. If enabled, logging works via ojAlgo's {@link BasicLogger} mechanism.
     * You can supply a thin wrapper implementation to redirect it to the logging library of your choice.
     *
     * @param debug true if debug logging is enabled
     */
    public void setDebug(boolean debug) {
        this.debug = debug;
    }
}

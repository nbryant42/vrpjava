package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.CVRPSolver.Result;
import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import org.ojalgo.optimisation.Expression;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayDeque;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.stream.Stream;

import static com.github.vrpjava.Util.newModel;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.addCut;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildConstraints;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildVars;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.findCycles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.toSeconds;
import static com.github.vrpjava.cvrp.Worker.updateBounds;
import static java.lang.Double.POSITIVE_INFINITY;
import static java.lang.Math.max;
import static java.math.BigDecimal.ZERO;
import static org.ojalgo.optimisation.Optimisation.State.INFEASIBLE;

/**
 * Multithreading is not implemented yet, but when it is, this class will represent a single processing job that we
 * register with a thread scheduler, and function as the coordination point for worker threads.
 */
class Job {
    private static final int PRUNE_INTERVAL = 50;

    private final OjAlgoCVRPSolver solver;
    private final int minVehicles;
    private final BigDecimal vehicleCapacity;
    private final BigDecimal[] demands;
    private final BigDecimal[][] costMatrix;
    private final long start;
    private final long deadline;
    private final Result kickstarter;
    private final GlobalBounds globalBounds;
    private final int maxScale;
    private boolean done;
    private double bestKnown;
    private Queue<Node> queue;
    private Result.State state;
    private Optimisation.Result incumbent;
    private int nodes;

    public Job(OjAlgoCVRPSolver solver, int minVehicles, BigDecimal vehicleCapacity,
               BigDecimal[] demands, BigDecimal[][] costMatrix, long timeout) {
        this.start = System.currentTimeMillis();
        this.solver = solver;
        this.minVehicles = minVehicles;
        this.vehicleCapacity = vehicleCapacity;
        this.demands = demands;
        this.costMatrix = costMatrix;
        this.deadline = start + timeout;
        this.kickstarter = solver.getHeuristic().doSolve(minVehicles, vehicleCapacity, demands, costMatrix,
                timeout);
        this.globalBounds = initBounds(minVehicles, vehicleCapacity, demands, costMatrix, deadline);
        this.maxScale = maxScale(costMatrix);
    }

    private static int maxScale(BigDecimal[][] costMatrix) {
        var size = costMatrix.length;
        var max = Integer.MIN_VALUE;

        for (var row = 1; row < size; row++) {
            var costRow = costMatrix[row];

            for (var col = 0; col < row; col++) {
                max = max(max, costRow[col].stripTrailingZeros().scale());
            }
        }
        return max;
    }

    static GlobalBounds initBounds(int minVehicles, BigDecimal vehicleCapacity, BigDecimal[] demands,
                                   BigDecimal[][] costMatrix, long deadline) {
        var model = newModel(deadline);
        var vars = buildVars(costMatrix, model);

        buildConstraints(model, minVehicles, vars);
        model.relax();

        var cuts = new HashSet<Cut>();
        var result = updateBounds(vehicleCapacity, demands, model, null, deadline, cuts);
        var prunedModel = pruneCuts(minVehicles, costMatrix, cuts, model, result, deadline);

        return new GlobalBounds(prunedModel, cuts, result);
    }

    // Find all cuts that have positive slack and prune them from the LP model, but keep them cached in the pool.
    private static ExpressionsBasedModel pruneCuts(int minVehicles, BigDecimal[][] costMatrix, Set<Cut> cuts,
                                                   ExpressionsBasedModel model, Optimisation.Result result,
                                                   long deadline) {
        var size = costMatrix.length;
        var prunedModel = newModel(deadline);
        var prunedVars = buildVars(costMatrix, prunedModel);

        buildConstraints(prunedModel, minVehicles, prunedVars);
        prunedModel.relax();

        cuts.stream().filter(cut -> isBinding(model, result, cut)).forEach(cut -> addCut(size, prunedModel, cut));
        return prunedModel;
    }

    // ojAlgo seems to get stuck in a loop when we format cuts with slack variables, so just calculate the slack
    static boolean isBinding(ExpressionsBasedModel model, Optimisation.Result result, Cut cut) {
        var expr = model.getExpression(cut.name());
        if (expr == null) {
            return false;
        }
        var sum = ZERO;

        for (int i = model.countVariables() - 1; i >= 0; i--) {
            var v = model.getVariable(i);
            sum = sum.add(expr.get(v).multiply(result.get(i)));
        }
        return sum.compareTo(expr.getUpperLimit()) >= 0;
    }

    Result run() {
        if (kickstarter.state() != Result.State.HEURISTIC) {
            // This theoretically can't happen now that I've removed the ill-considered `maxVehicles` parameter, but
            // let's check the status just to be thorough:
            bestKnown = POSITIVE_INFINITY;

            if (!globalBounds.getResult(deadline).getState().isOptimal()) {
                var myState = globalBounds.getResult(deadline).getState() == INFEASIBLE ? Result.State.INFEASIBLE :
                        Result.State.UNEXPLORED;

                return buildResult(myState, null, 0);
            }
        } else {
            bestKnown = kickstarter.objective();

            if (!globalBounds.getResult(deadline).getState().isOptimal()) {
                return buildResult(Result.State.HEURISTIC, null, 0);
            }
        }

        // queue of pending nodes for branch-and-bound search. Each node is represented as a Map
        // of variable IDs and values to be fixed in the model.
        // The queue can be either a LIFO queue (stack) for depth-first search or a priority queue for best-first
        // search. Depth-first search finds many valid solutions more quickly, whereas best-first search is more likely
        // to proceed directly to the better solutions, but will explore more internal nodes before it gets there. We
        // switch strategies based on the problem at hand, so don't assume this will always be a LIFO queue.
        queue = evaluateStrategy(globalBounds.getResult(deadline).getValue(), bestKnown, start,
                Collections.asLifoQueue(new ArrayDeque<>()), "Initial");
        var worker = new Worker(this);
        state = kickstarter.state();

        // the root node has no variables fixed
        queue.add(new Node(0, globalBounds.getResult(deadline).getValue(), Map.of(), globalBounds.getCuts()));

        while (!done && deadline > System.currentTimeMillis()) {
            var node = queue.poll();

            if (node == null) {
                if (state == Result.State.FEASIBLE) {
                    state = Result.State.OPTIMAL;
                }
                done = true;
            } else {
                if (++nodes % PRUNE_INTERVAL == 0) {
                    globalBounds.pruneCuts(this);
                }
                worker.process(node);
            }
        }

        return buildResult(state, incumbent, nodes);
    }

    void reportSolution(Optimisation.Result globalBoundsResult, Optimisation.Result nodeResult) {
        var lb = globalBoundsResult.getValue();
        var ub = nodeResult.getValue();
        queue = evaluateStrategy(lb, ub, start, queue, "New");

        incumbent = nodeResult;
        bestKnown = ub;
        state = Result.State.FEASIBLE;

        if (ub <= lb) {
            state = Result.State.OPTIMAL;
            done = true;
        } else if (!globalBoundsResult.getState().isOptimal()) {
            done = true;
        }
    }

    private Result buildResult(Result.State myState, Optimisation.Result incumbent, int nodes) {
        var cycles = incumbent == null ? kickstarter.cycles() : findCycles(demands.length, incumbent);

        solver.debug(nodes + " nodes, " + cycles.size() + " cycles: " + cycles);
        var cycleDemands = cycles.stream()
                .map(cycle -> cycle.stream().map(i -> demands[i]).reduce(ZERO, BigDecimal::add))
                .toList();
        solver.debug("Cycle demands: " + cycleDemands);
        solver.debug("Currently " + countCuts() + " cuts.");

        return new Result(myState, bestKnown, cycles);
    }

    private long countCuts() {
        return cuts(globalBounds.getModel()).count();
    }

    private static Stream<Expression> cuts(ExpressionsBasedModel model) {
        return model.getExpressions().stream().filter(e -> e.getName().startsWith("cut:"));
    }

    void queueNode(Node parent, double nodeBound, Integer k, BigDecimal v, Set<Cut> bindingCuts) {
        var childVars = new HashMap<>(parent.vars());
        childVars.put(k, v);
        queue.add(new Node(parent.depth() + 1, nodeBound, childVars, bindingCuts));
    }

    /**
     * Possibly update the search strategy from depth-first to best-first.
     *
     * @return either an updated queue (if changing the strategy) or the existing queue unmodified
     */
    private Queue<Node> evaluateStrategy(double lb, double ub, long start, Queue<Node> queue, String descriptor) {
        var ratio = lb / ub;
        var elapsed = System.currentTimeMillis() - start;
        var suffix = lb + "/" + ub + " (" + BigDecimal.valueOf(ratio * 100.0).setScale(2, RoundingMode.HALF_EVEN) +
                "%); " + countCuts() + "/" + globalBounds.cutPoolSize() + " active/pooled cuts, " + nodes + " nodes.";

        if (!(queue instanceof PriorityQueue<Node>) && ratio > solver.getBestFirstRatio() &&
                elapsed < solver.getBestFirstMillis()) {
            // Found a feasible solution, and bounds are tight enough that best-first search may help.
            // Switch to best-first search.
            // Also, if it's taken us more than X amount of time to get here, then we're working a
            // hard problem, and it's not a good idea to switch.
            queue = new PriorityQueue<>(queue);
            solver.debug("[" + toSeconds(elapsed) + "s]: Switching to best-first search. Bounds now " + suffix);
        } else {
            solver.debug("[" + toSeconds(elapsed) + "s]: " + descriptor + " solution. Bounds now " + suffix);
        }
        return queue;
    }

    double getBestKnown() {
        return bestKnown;
    }

    ExpressionsBasedModel copyGlobalBoundsModel() {
        return globalBounds.getModel().copy();
    }

    BigDecimal getVehicleCapacity() {
        return vehicleCapacity;
    }

    BigDecimal[] getDemands() {
        return demands;
    }

    long getDeadline() {
        return deadline;
    }

    int maxScale() {
        return maxScale;
    }

    GlobalBounds globalBounds() {
        return globalBounds;
    }

    Optimisation.Result getGlobalBoundsResult() {
        return globalBounds.getResult(deadline);
    }

    boolean isBestFirst() {
        return queue instanceof PriorityQueue<Node>;
    }

    BigDecimal[][] getCostMatrix() {
        return costMatrix;
    }

    int getMinVehicles() {
        return minVehicles;
    }

    boolean isBindingAnywhere(Cut cut) {
        return queue.stream().anyMatch(n -> n.isBinding(cut));
    }
}

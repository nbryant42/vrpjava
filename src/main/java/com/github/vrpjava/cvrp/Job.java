package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.CVRPSolver.Result;
import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import com.google.errorprone.annotations.concurrent.GuardedBy;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Collections;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

import static com.github.vrpjava.Util.newModel;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildConstraints;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildVars;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.findCycles;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.getVariable_noFlip;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.toSeconds;
import static com.github.vrpjava.cvrp.SubtourCuts.formatCut;
import static com.github.vrpjava.cvrp.Worker.updateBounds;
import static java.lang.Double.POSITIVE_INFINITY;
import static java.lang.Math.max;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;
import static org.ojalgo.function.constant.BigMath.HALF;
import static org.ojalgo.optimisation.Optimisation.State.INFEASIBLE;

/**
 * Represents a single processing job that we register with a thread scheduler,
 * and functions as the coordination point for worker threads.
 */
class Job {
    private static final BigDecimal MINUS_HALF = HALF.negate();

    private final OjAlgoCVRPSolver solver;
    private final BigDecimal vehicleCapacity;
    private final BigDecimal[] demands;
    private final long start;
    private final long deadline; // TODO replace usages with nanoTime
    private final Result kickstarter;
    @GuardedBy("globalBounds")
    private final GlobalBounds globalBounds;
    private final int maxScale;
    @GuardedBy("this")
    private boolean done;
    private volatile double bestKnown;

    @GuardedBy("this")
    private volatile Queue<Node> queue;
    private Result.State state;
    private Optimisation.Result incumbent;
    @GuardedBy("this")
    private int nodes;
    private final AtomicLong totalTime = new AtomicLong();
    private final AtomicInteger nodesInFlight = new AtomicInteger();

    Job(OjAlgoCVRPSolver solver, int minVehicles, BigDecimal vehicleCapacity,
        BigDecimal[] demands, BigDecimal[][] costMatrix, long timeout) {
        this.start = System.currentTimeMillis();
        this.solver = solver;
        this.vehicleCapacity = vehicleCapacity;
        this.demands = demands;
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

        return new GlobalBounds(model, updateBounds(vehicleCapacity, demands, model, null, deadline));
    }

    private static int doAddCuts(Collection<Cut> candidates,
                                 Set<Set<Integer>> cuts,
                                 ExpressionsBasedModel model,
                                 Optimisation.Result result,
                                 Job job,
                                 int size) {
        return candidates.stream().map(cut -> {
                    var subset = cut.subset();
                    subset.remove(0);

                    if (isViolated(result, size, subset, cut.minVehicles()) && cuts.add(subset)) {
                        int minVehicles = cut.minVehicles();
                        var name = formatCut(subset);

                        addCut(size, model, subset, name, minVehicles);
                        if (job != null) {
                            addCut(size, job.globalBounds.getModel(), subset, name, minVehicles);
                            // don't calculate the result here, only lazily when needed
                            job.globalBounds.clearResult();
                        }
                        return 1;
                    }
                    return 0;
                })
                .reduce(0, Integer::sum);
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

    private static void addCut(int size, ExpressionsBasedModel model, Set<Integer> subset, String name, long minVehicles) {
        if (subset.size() <= size * (size + 1) / (2 * size - 2)) {
            // Use constraint form (1.3)

            var cut = model.newExpression(name).upper(subset.size() - minVehicles);

            for (var row : subset) {
                for (var col : subset) {
                    if (col < row) {
                        cut.set(getVariable_noFlip(row, col, model), ONE);
                    }
                }
            }
        } else {
            // Use constraint form (4.1)

            var cut = model.newExpression(name).upper(size - 1 - subset.size() - minVehicles);

            for (var row = 1; row < size; row++) {
                if (subset.contains(row)) {
                    cut.set(getVariable_noFlip(row, 0, model), MINUS_HALF);
                } else {
                    cut.set(getVariable_noFlip(row, 0, model), HALF);

                    for (var col = 1; col < row; col++) {
                        if (!subset.contains(col)) {
                            cut.set(getVariable_noFlip(row, col, model), ONE);
                        }
                    }
                }
            }
        }
    }

    Result run() {
        var globalBoundsResult = globalBounds.getResult(deadline);

        if (kickstarter.state() != Result.State.HEURISTIC) {
            // This theoretically can't happen now that I've removed the ill-considered `maxVehicles` parameter, but
            // let's check the status just to be thorough:
            bestKnown = POSITIVE_INFINITY;

            if (!globalBoundsResult.getState().isOptimal()) {
                var myState = globalBoundsResult.getState() == INFEASIBLE ? Result.State.INFEASIBLE :
                        Result.State.UNEXPLORED;

                return buildResult(myState, null, 0);
            }
        } else {
            bestKnown = kickstarter.objective();

            if (!globalBoundsResult.getState().isOptimal()) {
                return buildResult(Result.State.HEURISTIC, null, 0);
            }
        }

        // queue of pending nodes for branch-and-bound search. Each node is represented as a Map
        // of variable IDs and values to be fixed in the model.
        // The queue can be either a LIFO queue (stack) for depth-first search or a priority queue for best-first
        // search. Depth-first search finds many valid solutions more quickly, whereas best-first search is more likely
        // to proceed directly to the better solutions, but will explore more internal nodes before it gets there. We
        // switch strategies based on the problem at hand, so don't assume this will always be a LIFO queue.
        queue = Collections.asLifoQueue(new ArrayDeque<>());
        solver.debug(evaluateStrategy(globalBoundsResult.getValue(), bestKnown, start, "Initial"));
        state = kickstarter.state();

        // the root node has no variables fixed.
        queue.add(new Node(0, globalBoundsResult.getValue(), Map.of()));
        solver.register(this);

        synchronized (this) {
            while (!done && deadline > System.currentTimeMillis()) {
                var remaining = deadline - System.currentTimeMillis();
                if (remaining <= 0) {
                    break;
                }
                try {
                    wait(remaining);
                } catch (InterruptedException ignored) {
                }
            }
        }

        solver.deregister(this);
        return buildResult(state, incumbent, nodes);
    }

    void reportSolution(Optimisation.Result nodeResult) {
        var globalBoundsResult = globalBounds.getResult(deadline);
        var lb = globalBoundsResult.getValue();
        var ub = nodeResult.getValue();
        String msg = null;

        synchronized (this) {
            if (ub < bestKnown) {
                msg = evaluateStrategy(lb, ub, start, "New");
                incumbent = nodeResult;
                bestKnown = ub;
                state = Result.State.FEASIBLE;

                if (ub <= lb) {
                    state = Result.State.OPTIMAL;
                    setDone();
                } else if (!globalBoundsResult.getState().isOptimal()) {
                    setDone();
                }
            }
        }

        if (msg != null) {
            solver.debug(msg);
        }
    }

    private void setDone() {
        done = true;
        notifyAll();
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
        synchronized (globalBounds) {
            return globalBounds.getModel().getExpressions().stream().filter(e -> e.getName().contains("cut:")).count();
        }
    }

    synchronized void queueNode(Node node) {
        queue.add(node);
    }

    /**
     * Possibly update the search strategy from depth-first to best-first,
     * by changing the queue to a {@link PriorityQueue}
     */
    private String evaluateStrategy(double lb, double ub, long start, String descriptor) {
        var ratio = lb / ub;
        var elapsed = System.currentTimeMillis() - start;
        var peek = queue.peek();
        var suffix = lb + "/" + ub + " (" + BigDecimal.valueOf(ratio * 100.0).setScale(2, RoundingMode.HALF_EVEN) +
                "%); " + countCuts() + " cuts, " + nodes + " nodes. Next node has bound " +
                (peek == null ? null : peek.bound());

        synchronized (this) {
            if (queue instanceof PriorityQueue<Node> || ratio <= solver.getBestFirstRatio() ||
                    elapsed >= solver.getBestFirstMillis()) {
                return "[" + toSeconds(elapsed) + "s]: " + descriptor + " solution. Bounds now " + suffix;
            }
            // Found a feasible solution, and bounds are tight enough that best-first search may help.
            // Switch to best-first search.
            // Also, if it's taken us more than X amount of time to get here, then we're working a
            // hard problem, and it's not a good idea to switch.
            queue = new PriorityQueue<>(queue);
            return "[" + toSeconds(elapsed) + "s]: Switching to best-first search. Bounds now " + suffix;
        }
    }

    double getBestKnown() {
        return bestKnown;
    }

    ExpressionsBasedModel copyGlobalBoundsModel() {
        synchronized (globalBounds) {
            return globalBounds.getModel().copy();
        }
    }

    BigDecimal vehicleCapacity() {
        return vehicleCapacity;
    }

    BigDecimal[] demands() {
        return demands;
    }

    long deadline() {
        return deadline;
    }

    int maxScale() {
        return maxScale;
    }

    /**
     * If we're also propagating cuts to the global model (<code>job</code> is not null), then we need to acquire the
     * <code>globalBounds</code> lock first. This method decides whether that's necessary.
     */
    static int addCuts(Collection<Cut> rccCuts, Set<Set<Integer>> cuts, ExpressionsBasedModel model,
                       Optimisation.Result result, Job job, int size) {
        return job == null ? doAddCuts(rccCuts, cuts, model, result, null, size) :
                job.addCuts(rccCuts, cuts, model, result);
    }

    int addCuts(Collection<Cut> rccCuts, Set<Set<Integer>> cuts, ExpressionsBasedModel model,
                Optimisation.Result result) {
        synchronized (globalBounds) {
            return doAddCuts(rccCuts, cuts, model, result, this, demands.length);
        }
    }

    boolean isBestFirst() {
        return queue instanceof PriorityQueue<Node>;
    }

    synchronized boolean hasWork() {
        return !queue.isEmpty();
    }

    synchronized Node nextNode() {
        nodesInFlight.getAndIncrement();
        nodes++;
        return queue.remove();
    }

    long totalTime() {
        return totalTime.get();
    }

    void resetTime() {
        this.totalTime.set(0);
    }

    /**
     * Called after each node is processed to record the runtime, for scheduling fairness purposes.
     * <p>
     * Also decrements an internal counter of nodes in flight, and if the queue is empty, handles the completion logic.
     */
    void nodeComplete(long elapsedTime) {
        totalTime.getAndAdd(elapsedTime);
        if (nodesInFlight.decrementAndGet() <= 0) {
            synchronized (this) {
                if (queue.isEmpty()) {
                    if (state == Result.State.FEASIBLE) {
                        state = Result.State.OPTIMAL;
                    }
                    setDone();
                }
            }
        }
    }
}

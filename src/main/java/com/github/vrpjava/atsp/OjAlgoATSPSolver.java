package com.github.vrpjava.atsp;

import org.ojalgo.netio.BasicLogger;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.Variable;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.stream.IntStream;

import static java.math.BigDecimal.ONE;
import static java.util.stream.Collectors.toSet;
import static com.github.vrpjava.Util.buildAsymmetricVars;
import static com.github.vrpjava.Util.newModel;

/**
 * Solver for the Asymmetric Travelling Salesman Problem.
 */
public class OjAlgoATSPSolver extends ATSPSolver {
    private boolean debug;

    /**
     * Default constructor
     */
    public OjAlgoATSPSolver() {
    }

    @Override
    protected List<Edge> doSolve(BigDecimal[][] costMatrix, long timeout) {
        // TODO check for the trivial 2x2 problem.

        var model = newModel(System.currentTimeMillis() + timeout);
        var vars = buildAsymmetricVars(costMatrix, model);

        buildConstraints(model, vars);

        // solve

        while (true) {
            var start = System.currentTimeMillis();
            var result = model.minimise();
            var elapsed = System.currentTimeMillis() - start;

            debug("Elapsed: " + elapsed + "ms; " + result.getState() + " " + result.getValue());

            var cycles = findCycles(costMatrix.length, result);
            var srcOnly = cycles.stream().map(cycle -> cycle.stream().map(Edge::src).toList()).toList();

            debug(srcOnly.size() + " cycles: " + srcOnly);

            if (cycles.size() == 1) {
                return cycles.getFirst();
            }

            addCuts(cycles, model, vars);
        }
    }

    private void debug(String s) {
        if (debug) {
            BasicLogger.debug(s);
        }
    }

    private static void addCuts(List<List<Edge>> cycles, ExpressionsBasedModel model, Variable[][] vars) {
        for (var cycle : cycles) {
            var cut = model.newExpression("cut: " + cycle).level(cycle.size() - 1);

            for (var edge : cycle) {
                cut.set(vars[edge.src()][edge.dest()], ONE);
            }
        }
    }

    private static List<List<Edge>> findCycles(int size, Optimisation.Result result) {
        var cycles = new ArrayList<List<Edge>>();
        var remaining = IntStream.range(0, size).boxed().collect(toSet());
        int src;

        while ((src = pop(remaining)) >= 0) {
            var visited = new HashSet<Integer>();
            var cycleEdges = new ArrayList<Edge>();
            int dest;

            visited.add(src);
            do {
                dest = getDest(src, size, result);
                cycleEdges.add(new Edge(src, dest));
                remaining.remove(dest);
                src = dest;
            } while (visited.add(dest));

            cycles.add(cycleEdges);
        }
        return cycles;
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

    private static int getDest(int vtx, int size, Optimisation.Result result) {
        var start = vtx * (size - 1);
        for (var cur = 0; cur < vtx; cur++) {
            if (result.get(start + cur).equals(ONE)) {
                return cur;
            }
        }
        for (var cur = vtx + 1; cur < size; cur++) {
            if (result.get(start + cur - 1).equals(ONE)) {
                return cur;
            }
        }
        throw new RuntimeException("Can't find destination! vtx=" + vtx + ", size=" + size + ", result=" + result);
    }

    private static void buildConstraints(ExpressionsBasedModel model, Variable[][] vars) {
        var size = vars.length;

        for (var i = 0; i < size; i++) {
            var outgoing = model.newExpression("outgoing_" + i).level(ONE);
            for (var j = 0; j < size; j++) {
                if (i != j) {
                    outgoing.set(vars[i][j], ONE);
                }
            }

            var incoming = model.newExpression("incoming_" + i).level(ONE);
            for (var j = 0; j < size; j++) {
                if (i != j) {
                    incoming.set(vars[j][i], ONE);
                }
            }
        }
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
    @SuppressWarnings("unused")
    public void setDebug(boolean debug) {
        this.debug = debug;
    }
}

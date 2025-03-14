package com.github.vrpjava.atsp;

import com.github.vrpjava.Util;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

/**
 * Abstract superclass for implementations of an Asymmetric Travelling Salesman Problem solver.
 */
public abstract class ATSPSolver {
    /**
     * Default constructor.
     */
    protected ATSPSolver() {
    }

    /**
     * <p>
     * Return an approximate solution via the nearest neighbor heuristic.
     * </p><p>
     * This may be useful as a starting point or lower bound for other algorithms.
     * </p>
     *
     * @param costMatrix square matrix of distances between locations (should be zero on the diagonal)
     * @return the list of edges defining the solution
     */
    public static List<Edge> nearestNeighbor(BigDecimal[][] costMatrix) {
        var size = costMatrix.length;
        var result = new ArrayList<Edge>();
        var src = 0;
        var visited = new HashSet<Integer>();
        visited.add(0);

        while (true) {
            var row = costMatrix[src];
            var dest = 0;
            BigDecimal min = null;

            for (var tmp = 0; tmp < size; tmp++) {
                if (tmp != src) {
                    var cost = row[tmp];

                    if ((min == null || cost.compareTo(min) < 0) && !visited.contains(tmp)) {
                        min = cost;
                        dest = tmp;
                        visited.add(dest);
                    }
                }
            }

            result.add(new Edge(src, dest));
            if (dest == 0) {
                break;
            }
            src = dest;
        }
        return result;
    }

    /**
     * Equivalent to <code>solve(costMatrix, 1000L * 60L * 60L)</code>
     *
     * @param costMatrix square matrix of distances between locations (should be zero on the diagonal)
     * @return the list of edges defining the solution
     */
    public final List<Edge> solve(BigDecimal[][] costMatrix) {
        return solve(costMatrix, 1000L * 60L * 60L);
    }

    /**
     * Solve the Asymmetric Travelling Salesman Problem (ATSP)
     *
     * @param costMatrix    square matrix of distances between locations (should be zero on the diagonal)
     * @param timeoutMillis the maximum wall-clock time in milliseconds
     * @return the list of edges defining the solution
     */
    public final List<Edge> solve(BigDecimal[][] costMatrix, long timeoutMillis) {
        Util.validate(costMatrix);

        return doSolve(costMatrix, timeoutMillis);
    }

    /**
     * To be implemented by subclasses.
     *
     * @param costMatrix square matrix of distances between locations (should be zero on the diagonal)
     * @param timeout    the maximum wall-clock time in milliseconds
     * @return the list of edges defining the solution
     */
    protected abstract List<Edge> doSolve(BigDecimal[][] costMatrix, long timeout);
}

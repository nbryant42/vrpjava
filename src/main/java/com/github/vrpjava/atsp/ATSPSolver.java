package com.github.vrpjava.atsp;

import com.github.vrpjava.Util;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

public abstract class ATSPSolver {
    /**
     * Return an approximate solution via the nearest neighbor heuristic.
     * <p>
     * This may be useful as a starting point or lower bound for other algorithms.
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

    public final List<Edge> solve(BigDecimal[][] costMatrix) {
        return solve(costMatrix, 1000L * 60L * 60L);
    }

    public final List<Edge> solve(BigDecimal[][] costMatrix, long timeoutMillis) {
        Util.validate(costMatrix);

        return doSolve(costMatrix, timeoutMillis);
    }

    protected abstract List<Edge> doSolve(BigDecimal[][] costMatrix, long timeout);
}

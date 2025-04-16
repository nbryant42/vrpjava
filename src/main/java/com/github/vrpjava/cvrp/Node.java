package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;

import java.math.BigDecimal;
import java.util.Map;
import java.util.Set;

record Node(int depth, double bound, Map<Integer, BigDecimal> vars, Set<Cut> bindingCuts) implements Comparable<Node> {
    @Override
    public int compareTo(Node o) {
        var v = Double.compare(bound, o.bound); // sort smallest bound first (minimization problem)
        return v != 0 ? v : o.depth - depth; // tiebreak on the deeper node
    }

    boolean isBinding(Cut cut) {
        return bindingCuts.contains(cut);
    }
}

package com.github.vrpjava.cvrp;

import java.math.BigDecimal;
import java.util.HashMap;
import java.util.Map;

record Node(int depth, double bound, Map<Integer, BigDecimal> vars) implements Comparable<Node> {
    Node(Node parent, double nodeBound, Integer varToFix, BigDecimal val) {
        this(parent.depth() + 1, nodeBound, buildVars(parent, varToFix, val));
    }

    private static Map<Integer, BigDecimal> buildVars(Node parent, Integer k, BigDecimal v) {
        var childVars = new HashMap<>(parent.vars());
        childVars.put(k, v);
        return childVars;
    }

    @Override
    public int compareTo(Node o) {
        var v = Double.compare(bound, o.bound); // sort smallest bound first (minimization problem)
        return v != 0 ? v : o.depth - depth; // tiebreak on the deeper node
    }
}

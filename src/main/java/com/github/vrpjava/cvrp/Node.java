package com.github.vrpjava.cvrp;

import java.math.BigDecimal;
import java.util.Map;

record Node(int depth, double bound, Map<Integer, BigDecimal> vars) implements Comparable<Node> {
    @Override
    public int compareTo(Node o) {
        return Double.compare(bound, o.bound); // sort smallest bound first (minimization problem)
    }
}

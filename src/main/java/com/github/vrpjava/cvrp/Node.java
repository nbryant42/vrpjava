package com.github.vrpjava.cvrp;

import java.math.BigDecimal;
import java.util.Map;

record Node(double bound, BigDecimal gap, Map<Integer, BigDecimal> vars) implements Comparable<Node> {
    @Override
    public int compareTo(Node o) {
        var b = Double.compare(bound, o.bound);
        return b != 0 ? b : gap.compareTo(o.gap); // tie-break on the gap
    }
}

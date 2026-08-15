package com.github.vrpjava.atsp;

import org.junit.jupiter.api.Test;

import java.math.BigDecimal;
import java.util.List;

import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
import static java.math.BigDecimal.valueOf;
import static org.junit.jupiter.api.Assertions.assertEquals;

class ATSPSolverTest {
    @Test
    void nearestNeighbor() {
        var edges = ATSPSolver.nearestNeighbor(new BigDecimal[][]{
                {ZERO, ONE, TWO, TWO},
                {ONE, ZERO, TWO, TWO},
                {TWO, TWO, ZERO, ONE},
                {TWO, TWO, ONE, ZERO}});

        assertEquals("[0-->1, 1-->2, 2-->3, 3-->0]", edges.toString());
    }

    @Test
    void nearestNeighborDoesNotMarkRejectedCandidatesVisited() {
        var edges = ATSPSolver.nearestNeighbor(new BigDecimal[][]{
                {ZERO, valueOf(3), TWO, ONE},
                {ONE, ZERO, ONE, ONE},
                {ONE, ONE, ZERO, ONE},
                {ONE, TWO, ONE, ZERO}});

        assertEquals(List.of(
                new Edge(0, 3),
                new Edge(3, 2),
                new Edge(2, 1),
                new Edge(1, 0)), edges);
    }
}

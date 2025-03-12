package org.example;

import org.junit.jupiter.api.Test;

import java.math.BigDecimal;

import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
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
}
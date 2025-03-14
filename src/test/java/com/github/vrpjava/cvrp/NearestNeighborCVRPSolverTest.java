package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.CVRPSolver.Result;
import org.junit.jupiter.api.Test;

import java.math.BigDecimal;
import java.util.List;

import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

class NearestNeighborCVRPSolverTest {
    @Test
    void worksCorrectly() {
        BigDecimal[] demands = {ZERO, ONE, ONE};
        BigDecimal[][] costs = {
                {ZERO, ZERO, ZERO},
                {ONE, ZERO, ZERO},
                {ONE, ONE, ZERO}};

        var solver = new NearestNeighborCVRPSolver();

        // capacity constraint
        assertEquals(new Result(4.0, List.of(List.of(0, 1), List.of(0, 2))),
                solver.doSolve(1, 2, ONE, demands, costs, 0L));

        // constraints non-binding
        assertEquals(new Result(3.0, List.of(List.of(0, 1, 2))),
                solver.doSolve(1, 2, TWO, demands, costs, 0L));

        // min vehicles constraint
        assertEquals(new Result(4.0, List.of(List.of(0, 1), List.of(0, 2))),
                solver.doSolve(2, 2, TWO, demands, costs, 0L));

        // max vehicles constraint (no solution)
        // (the problem is not well-defined here -- maybe we should just drop the maxVehicles parameter)
        assertNull(solver.doSolve(1, 1, ONE, demands, costs, 0L));

    }
}
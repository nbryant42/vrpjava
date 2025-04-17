package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.Test;

import java.math.BigDecimal;
import java.util.List;
import java.util.Set;

import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.HEURISTIC;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public abstract class AbstractCVRPSolverTest {
    @Test
    void worksCorrectly() {
        BigDecimal[] demands = {ZERO, ONE, ONE};
        BigDecimal[][] costs = {
                {ZERO, ZERO, ZERO},
                {ONE, ZERO, ZERO},
                {ONE, ONE, ZERO}};

        try (var solver = newSolver()) {
            // capacity constraint
            assertEquals(new CVRPSolver.Result(expectedStatus(), 4.0, Set.of(List.of(0, 1), List.of(0, 2))),
                    solver.doSolve(1, ONE, demands, costs, 0L));

            // constraints non-binding. There are two equivalent solutions here; allow either.
            var actual = solver.doSolve(1, TWO, demands, costs, 0L);
            assertTrue(new CVRPSolver.Result(expectedStatus(), 3.0, Set.of(List.of(0, 1, 2))).equals(actual) ||
                    new CVRPSolver.Result(expectedStatus(), 3.0, Set.of(List.of(0, 2, 1))).equals(actual));

            // min vehicles constraint
            assertEquals(new CVRPSolver.Result(expectedStatus(), 4.0, Set.of(List.of(0, 1), List.of(0, 2))),
                    solver.doSolve(2, TWO, demands, costs, 0L));
        }
    }

    protected CVRPSolver.Result.State expectedStatus() {
        return HEURISTIC;
    }

    protected abstract CVRPSolver newSolver();
}

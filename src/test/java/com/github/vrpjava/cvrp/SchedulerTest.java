package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.Test;

import java.math.BigDecimal;
import java.util.concurrent.atomic.AtomicBoolean;

import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.OPTIMAL;
import static java.math.BigDecimal.ZERO;
import static java.math.BigDecimal.valueOf;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

class SchedulerTest {
    @Test
    void workerFailureIsSurfacedWithoutKillingTheScheduler() {
        var injected = new IllegalArgumentException("injected worker failure");
        var failNextNode = new AtomicBoolean(true);

        try (var solver = new OjAlgoCVRPSolver(delegate -> (job, node) -> {
            if (failNextNode.getAndSet(false)) {
                throw injected;
            }
            return delegate.process(job, node);
        })) {
            var thrown = assertThrows(IllegalStateException.class, () -> solveTinyProblem(solver));

            assertSame(injected, thrown.getCause());
            assertEquals(OPTIMAL, solveTinyProblem(solver).state());
        }
    }

    private static CVRPSolver.Result solveTinyProblem(OjAlgoCVRPSolver solver) {
        var demands = new BigDecimal[]{ZERO, valueOf(1), valueOf(1)};
        var costs = new BigDecimal[][]{
                {ZERO, ZERO, ZERO},
                {valueOf(1), ZERO, ZERO},
                {valueOf(1), valueOf(1), ZERO}};

        return solver.solve(2, valueOf(2), demands, costs, 10_000L);
    }
}

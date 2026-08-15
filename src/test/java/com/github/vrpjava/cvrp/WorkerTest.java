package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import org.junit.jupiter.api.Test;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.util.List;
import java.util.Set;

import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.ojalgo.optimisation.Optimisation.State.OPTIMAL;

class WorkerTest {
    @Test
    void branchingCoversTheCompleteIntegerDomain() {
        assertEquals(List.of(ZERO, ONE, TWO), Worker.branchValues(ZERO, TWO));
    }

    @Test
    void overloadedRouteProducesADirectCapacityCut() {
        var demands = new BigDecimal[]{ZERO, TWO, ONE, ONE};
        var cycles = List.of(List.of(0, 1, 2), List.of(0, 3));
        var cuts = Worker.capacityCuts(TWO, demands, cycles);
        var cut = cuts.iterator().next();
        var candidate = Optimisation.Result.of(OPTIMAL, 1, 1, 1, 2, 0, 0);

        assertEquals(Set.of(new Cut(2, Set.of(1, 2))), cuts);
        assertTrue(Job.isViolated(candidate, demands.length, cut.subset(), cut.minVehicles()));
        assertFalse(Worker.isValidIntegerSolution(TWO, demands, cycles));
    }

    @Test
    void candidateValidationChecksRouteStructureCapacityAndCoverage() {
        var demands = new BigDecimal[]{ZERO, TWO, ONE, ONE};

        var validCycles = List.of(List.of(0, 1), List.of(0, 2, 3));

        assertTrue(Worker.isValidIntegerSolution(TWO, demands, validCycles));
        assertTrue(Worker.capacityCuts(TWO, demands, validCycles).isEmpty());
        assertFalse(Worker.isValidIntegerSolution(TWO, demands,
                List.of(List.of(0, 3), List.of(1, 2))));
        assertFalse(Worker.isValidIntegerSolution(TWO, demands,
                List.of(List.of(0, 1), List.of(0, 2))));
    }
}

package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.ExperimentalParameters;
import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.SearchStrategy;
import org.junit.jupiter.api.Test;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.ojalgo.optimisation.Optimisation.State.OPTIMAL;
import static org.ojalgo.optimisation.Optimisation.State.INFEASIBLE;

class WorkerTest {
    @Test
    void provenNodeBoundCanFathomBeforeFurtherSeparation() {
        assertTrue(Worker.boundFathoms(593.0, 593.0, 0));
        assertTrue(Worker.boundFathoms(594.0, 593.0, 0));
        assertFalse(Worker.boundFathoms(592.0, 593.0, 0));
    }

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

    @Test
    void optionalThreeToothSeparationTightensTheRootUpdate() {
        var demands = new BigDecimal[]{ZERO, ONE, ONE, ONE, ONE, ONE, ONE};
        var capacity = BigDecimal.valueOf(6);
        var deadline = Long.MAX_VALUE;
        var disabled = new ExperimentalParameters(SearchStrategy.BEST_FIRST, 1, 0,
                0L, 0L, 0.85, 30_000L);
        var enabled = new ExperimentalParameters(SearchStrategy.BEST_FIRST, 1, 0,
                0L, 10_000L, 0.85, 30_000L);
        var progress = new ArrayList<String>();

        var withoutComb = Worker.updateBounds(capacity, demands, violatedThreeToothModel(), null, deadline, disabled);
        var withComb = Worker.updateBounds(capacity, demands, violatedThreeToothModel(), null, deadline, enabled,
                progress::add);

        assertTrue(withoutComb.getState().isOptimal());
        assertEquals(INFEASIBLE, withComb.getState());
        assertEquals(List.of("Starting 3-tooth separation", "3-tooth separation found 1 cuts"), progress);
    }

    @Test
    void optionalHeuristicCombSeparationTightensTheRootUpdate() {
        var demands = new BigDecimal[]{ZERO, ONE, ONE, ONE, ONE, ONE, ONE};
        var capacity = BigDecimal.valueOf(6);
        var deadline = Long.MAX_VALUE;
        var disabled = new ExperimentalParameters(SearchStrategy.BEST_FIRST, 1, 0,
                0L, 0L, 0L, 0.85, 30_000L);
        var enabled = new ExperimentalParameters(SearchStrategy.BEST_FIRST, 1, 0,
                0L, 10_000L, 0L, 0.85, 30_000L);
        var progress = new ArrayList<String>();

        var withoutComb = Worker.updateBounds(capacity, demands, violatedThreeToothModel(), null, deadline, disabled);
        var withComb = Worker.updateBounds(capacity, demands, violatedThreeToothModel(), null, deadline, enabled,
                progress::add);

        assertTrue(withoutComb.getState().isOptimal());
        assertEquals(INFEASIBLE, withComb.getState());
        assertEquals(List.of("Starting heuristic comb separation", "Heuristic comb separation found 1 cuts"),
                progress);
    }

    @Test
    void optionalGlmSeparationTightensTheRootUpdate() {
        var demands = new BigDecimal[]{ZERO, BigDecimal.valueOf(4), BigDecimal.valueOf(4),
                BigDecimal.valueOf(4), BigDecimal.valueOf(4)};
        var capacity = BigDecimal.TEN;
        var deadline = Long.MAX_VALUE;
        var disabled = new ExperimentalParameters(SearchStrategy.BEST_FIRST, 1, 0,
                0L, 0L, 0L, 0L, 0.85, 30_000L);
        var enabled = new ExperimentalParameters(SearchStrategy.BEST_FIRST, 1, 0,
                0L, 0L, 10_000L, 0L, 0.85, 30_000L);
        var progress = new ArrayList<String>();

        var withoutMultistar = Worker.updateBounds(capacity, demands, violatedMultistarModel(), null, deadline,
                disabled);
        var withMultistar = Worker.updateBounds(capacity, demands, violatedMultistarModel(), null, deadline, enabled,
                progress::add);

        assertTrue(withoutMultistar.getState().isOptimal());
        assertEquals(INFEASIBLE, withMultistar.getState());
        assertEquals(List.of("Starting GLM separation", "GLM separation found 3 cuts"), progress);
    }

    private static ExpressionsBasedModel violatedThreeToothModel() {
        var edges = new double[7][7];
        edge(edges, 1, 2, 0.5);
        edge(edges, 1, 3, 0.5);
        edge(edges, 2, 3, 0.5);
        edge(edges, 1, 4, 1.0);
        edge(edges, 2, 5, 1.0);
        edge(edges, 3, 6, 1.0);
        edge(edges, 0, 4, 1.0);
        edge(edges, 0, 5, 1.0);
        edge(edges, 0, 6, 1.0);

        var model = new ExpressionsBasedModel();
        for (var row = 1; row < edges.length; row++) {
            for (var col = 0; col < row; col++) {
                model.newVariable("x" + row + "_" + col).level(edges[row][col]);
            }
        }
        return model;
    }

    private static ExpressionsBasedModel violatedMultistarModel() {
        var edges = new double[5][5];
        edge(edges, 0, 3, 1.0);
        edge(edges, 3, 1, 1.0);
        edge(edges, 1, 2, 1.0);
        edge(edges, 2, 4, 1.0);
        edge(edges, 4, 0, 1.0);

        var model = new ExpressionsBasedModel();
        for (var row = 1; row < edges.length; row++) {
            for (var col = 0; col < row; col++) {
                model.newVariable("x" + row + "_" + col).level(edges[row][col]);
            }
        }
        return model;
    }

    private static void edge(double[][] edges, int first, int second, double value) {
        edges[first][second] = value;
        edges[second][first] = value;
    }
}

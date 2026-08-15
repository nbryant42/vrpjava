package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.Test;

import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.FEASIBLE;
import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.HEURISTIC;
import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.OPTIMAL;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SearchProgressTest {
    @Test
    void resolvedExhaustionProvesTheIncumbentOptimal() {
        var progress = new SearchProgress();

        progress.nodeStarted();
        var completion = progress.nodeFinished(NodeOutcome.RESOLVED, false);

        assertAll(
                () -> assertTrue(completion.exhausted()),
                () -> assertTrue(completion.proofComplete()),
                () -> assertEquals(OPTIMAL, completion.resultState(HEURISTIC)),
                () -> assertEquals(OPTIMAL, completion.resultState(FEASIBLE)));
    }

    @Test
    void incompleteNodePreventsAnOptimalityClaim() {
        var progress = new SearchProgress();

        progress.nodeStarted();
        var completion = progress.nodeFinished(NodeOutcome.INCOMPLETE, false);

        assertAll(
                () -> assertTrue(completion.exhausted()),
                () -> assertFalse(completion.proofComplete()),
                () -> assertEquals(HEURISTIC, completion.resultState(HEURISTIC)),
                () -> assertEquals(FEASIBLE, completion.resultState(FEASIBLE)));
    }

    @Test
    void queuedWorkPreventsExhaustion() {
        var progress = new SearchProgress();

        progress.nodeStarted();
        var completion = progress.nodeFinished(NodeOutcome.RESOLVED, true);

        assertAll(
                () -> assertFalse(completion.exhausted()),
                () -> assertTrue(completion.proofComplete()),
                () -> assertEquals(HEURISTIC, completion.resultState(HEURISTIC)));
    }

    @Test
    void incompleteOutcomeRemainsStickyAcrossConcurrentNodes() {
        var progress = new SearchProgress();

        progress.nodeStarted();
        progress.nodeStarted();
        var first = progress.nodeFinished(NodeOutcome.INCOMPLETE, false);
        var completion = progress.nodeFinished(NodeOutcome.RESOLVED, false);

        assertAll(
                () -> assertFalse(first.exhausted()),
                () -> assertFalse(first.proofComplete()),
                () -> assertTrue(completion.exhausted()),
                () -> assertFalse(completion.proofComplete()),
                () -> assertEquals(HEURISTIC, completion.resultState(HEURISTIC)));
    }
}

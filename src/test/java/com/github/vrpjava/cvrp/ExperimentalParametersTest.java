package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.Test;

import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.ExperimentalParameters;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.SearchStrategy.AUTO;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.SearchStrategy.BEST_FIRST;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.SearchStrategy.DEPTH_FIRST;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ExperimentalParametersTest {
    @Test
    void defaultsPreserveRootOnlyRccAndAutomaticSearch() {
        var parameters = ExperimentalParameters.defaults();

        assertEquals(AUTO, parameters.searchStrategy());
        assertEquals(Long.MAX_VALUE, parameters.rccMillis());
        assertFalse(parameters.useRccAtDepth(1));
        assertEquals(1234L, parameters.rccDeadline(1234L));
        assertTrue(parameters.useBestFirst(0.90, 1_000L));
        assertFalse(parameters.useBestFirst(0.80, 1_000L));
        assertFalse(parameters.useBestFirst(0.90, 30_000L));
    }

    @Test
    void explicitStrategiesIgnoreAutomaticThresholds() {
        var depthFirst = new ExperimentalParameters(DEPTH_FIRST, 1, 2, 1_000L, 0.85, 30_000L);
        var bestFirst = new ExperimentalParameters(BEST_FIRST, 1, 2, 1_000L, 0.85, 30_000L);

        assertFalse(depthFirst.useBestFirst(1.0, 0L));
        assertTrue(bestFirst.useBestFirst(0.01, Long.MAX_VALUE));
        assertTrue(depthFirst.useRccAtDepth(1));
        assertTrue(depthFirst.useRccAtDepth(2));
        assertFalse(depthFirst.useRccAtDepth(0));
        assertFalse(depthFirst.useRccAtDepth(3));
    }

    @Test
    void nodeRccDepthRangeIsInclusive() {
        var parameters = new ExperimentalParameters(AUTO, 4, 5, 1_000L, 0.85, 30_000L);

        assertFalse(parameters.useRccAtDepth(3));
        assertTrue(parameters.useRccAtDepth(4));
        assertTrue(parameters.useRccAtDepth(5));
        assertFalse(parameters.useRccAtDepth(6));
    }

    @Test
    void invalidParametersAreRejected() {
        assertAll(
                () -> assertThrows(NullPointerException.class,
                        () -> new ExperimentalParameters(null, 1, 0, 0L, 0.85, 30_000L)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> new ExperimentalParameters(AUTO, 0, 0, 0L, 0.85, 30_000L)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> new ExperimentalParameters(AUTO, 1, -1, 0L, 0.85, 30_000L)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> new ExperimentalParameters(AUTO, 1, 0, -1L, 0.85, 30_000L)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> new ExperimentalParameters(AUTO, 1, 0, 0L, 0.0, 30_000L)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> new ExperimentalParameters(AUTO, 1, 0, 0L, 1.01, 30_000L)),
                () -> assertThrows(IllegalArgumentException.class,
                        () -> new ExperimentalParameters(AUTO, 1, 0, 0L, 0.85, -1L)));
    }
}

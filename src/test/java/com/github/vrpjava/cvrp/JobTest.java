package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.ojalgo.optimisation.Optimisation.State.FEASIBLE;
import static org.ojalgo.optimisation.Optimisation.State.OPTIMAL;
import static org.ojalgo.optimisation.Optimisation.State.UNEXPLORED;

class JobTest {
    @Test
    void onlyAProvenLowerBoundCanCloseTheSearch() {
        assertTrue(Job.canProveOptimal(OPTIMAL, 12.0, 12.0));
        assertFalse(Job.canProveOptimal(FEASIBLE, 12.0, 12.0));
        assertFalse(Job.canProveOptimal(UNEXPLORED, 12.0, 12.0));
    }
}

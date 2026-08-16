package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import org.junit.jupiter.api.Test;
import org.ojalgo.optimisation.Optimisation;

import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.ojalgo.optimisation.Optimisation.State.OPTIMAL;

class CutCandidatesTest {

    @Test
    void returnsStableSnapshotsWhileCallbacksContinue() {
        var parentResult = Optimisation.Result.of(0.0, OPTIMAL, 0.0, 0.0, 0.0);
        var candidates = new CutCandidates(3, parentResult);
        var first = new Cut(1, Set.of(1));
        var stronger = new Cut(1, Set.of(2));

        candidates.take(Optimisation.Result.of(1.0, OPTIMAL), first);
        var snapshot = candidates.getCuts();
        candidates.take(Optimisation.Result.of(2.0, OPTIMAL), stronger);

        assertEquals(Set.of(first), snapshot);
        assertEquals(Set.of(stronger), candidates.getCuts());
    }
}

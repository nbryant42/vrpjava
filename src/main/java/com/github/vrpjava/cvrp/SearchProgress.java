package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.CVRPSolver.Result;

import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.INFEASIBLE;
import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.OPTIMAL;

enum NodeOutcome {
    RESOLVED,
    INCOMPLETE
}

/**
 * Proof bookkeeping for the nodes removed from a job's queue.
 *
 * This class is used while holding the owning {@link Job} lock.
 */
final class SearchProgress {
    record Completion(boolean exhausted, boolean proofComplete) {
        Result.State resultState(Result.State current) {
            if (!exhausted || !proofComplete) {
                return current;
            }
            return current.isFeasible() ? OPTIMAL : INFEASIBLE;
        }
    }

    private int inFlight;
    private boolean proofComplete = true;

    void nodeStarted() {
        inFlight++;
    }

    Completion nodeFinished(NodeOutcome outcome, boolean hasQueuedNodes) {
        if (inFlight <= 0) {
            throw new IllegalStateException("No node is in flight.");
        }
        if (outcome == NodeOutcome.INCOMPLETE) {
            proofComplete = false;
        }
        inFlight--;
        return new Completion(inFlight == 0 && !hasQueuedNodes, proofComplete);
    }
}

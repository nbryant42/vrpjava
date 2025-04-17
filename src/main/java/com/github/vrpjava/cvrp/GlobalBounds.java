package com.github.vrpjava.cvrp;

import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.minimize;

final class GlobalBounds {
    private final ExpressionsBasedModel model;
    private Optimisation.Result result;

    GlobalBounds(ExpressionsBasedModel model, Optimisation.Result result) {
        this.model = model;
        this.result = result;
    }

    ExpressionsBasedModel getModel() {
        return model;
    }

    synchronized Optimisation.Result getResult(long deadline) {
        if (result == null) {
            // TODO although the global lower bound CAN change during a run, it seems rare. Maybe drop this
            // re-solve and just hold it constant?
            result = minimize(model, deadline);
        }
        return result;
    }

    void clearResult() {
        result = null;
    }
}

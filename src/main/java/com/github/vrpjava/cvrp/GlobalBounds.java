package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;

import java.util.HashSet;
import java.util.Set;
import java.util.stream.Stream;

import static com.github.vrpjava.Util.newModel;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildConstraints;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.buildVars;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.minimize;

final class GlobalBounds {
    private ExpressionsBasedModel model;
    private final Set<Cut> cuts;
    private Optimisation.Result result;

    GlobalBounds(ExpressionsBasedModel model, Set<Cut> cuts, Optimisation.Result result) {
        this.model = model;
        this.cuts = new HashSet<>(cuts);
        this.result = result;
    }

    ExpressionsBasedModel getModel() {
        return model;
    }

    Optimisation.Result getResult(long deadline) {
        if (result == null) {
            // TODO although the global lower bound CAN change during a run, it seems rare. Maybe drop this
            // re-solve and just hold it constant?
            result = minimize(model, deadline);
        }
        return result;
    }

    void addCut(int size, Cut cut) {
        OjAlgoCVRPSolver.addCut(size, model, cut);
        cuts.add(cut);
        // don't calculate the result here, only lazily when needed, otherwise we'll duplicate effort.
        result = null;
    }

    Stream<Cut> streamCuts() {
        return cuts.stream();
    }

    int cutPoolSize() {
        return cuts.size();
    }

    void pruneCuts(Job job) {
        var size = job.getDemands().length;
        model = newModel(job.getDeadline());
        var vars = buildVars(job.getCostMatrix(), model);

        buildConstraints(model, job.getMinVehicles(), vars);
        model.relax();

        for (var cut : cuts) {
            if (job.isBindingAnywhere(cut)) {
                OjAlgoCVRPSolver.addCut(size, model, cut);
            }
        }
    }

    Set<Cut> getCuts() {
        return cuts;
    }
}

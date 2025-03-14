package com.github.vrpjava.cvrp;

import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.HashSet;
import java.util.Set;

import static java.lang.Math.min;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.isViolated;

class CutCandidates {
    private final int size;
    private BigDecimal objective;
    private final Set<Cut> cuts = new HashSet<>();
    private final Optimisation.Result parentResult;

    void take(Optimisation.Result result) {
        take(result, new Cut(size, result));
    }

    void take(Optimisation.Result result, Cut cut) {
        if (!isViolated(parentResult, size, cut.subset(), cut.minVehicles())) {
            return;
        }
        var objective = round(BigDecimal.valueOf(result.getValue()));

        if (this.objective == null || this.objective.compareTo(objective) < 0) {
            this.objective = objective;
            cuts.clear();
        }
        if (this.objective.compareTo(objective) == 0) {
            cuts.add(cut);
        }
    }

    static BigDecimal round(BigDecimal objective) {
        return objective.setScale(min(8, objective.scale()), RoundingMode.HALF_EVEN);
    }

    CutCandidates(int size, Optimisation.Result parentResult) {
        this.size = size;
        this.parentResult = parentResult;
    }

    Set<Cut> getCuts() {
        return cuts;
    }
}

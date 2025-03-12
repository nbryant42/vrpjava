package org.example;

import org.example.OjAlgoCVRPSolver.Cut;
import org.ojalgo.optimisation.Optimisation;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.HashSet;
import java.util.Set;

import static java.lang.Math.min;
import static org.example.OjAlgoCVRPSolver.isViolated;

public class CutCandidates {
    private final int size;
    private BigDecimal objective;
    private final Set<Cut> cuts = new HashSet<>();
    private final Optimisation.Result parentResult;

    public void take(Optimisation.Result result) {
        take(result, new Cut(size, result));
    }

    public void take(Optimisation.Result result, Cut cut) {
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

    public CutCandidates(int size, Optimisation.Result parentResult) {
        this.size = size;
        this.parentResult = parentResult;
    }

    public int getSize() {
        return size;
    }

    public Set<Cut> getCuts() {
        return cuts;
    }
}

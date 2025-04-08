package com.github.vrpjava.cvrp;

import io.github.lmores.tsplib.TsplibArchive;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.math.BigDecimal;
import java.math.MathContext;
import java.math.RoundingMode;
import java.util.List;
import java.util.Set;
import java.util.stream.IntStream;

import static com.github.vrpjava.Util.setUpHardware_raptorLake;
import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.OPTIMAL;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolverTest.getEdgeWeightNonRounded;
import static java.lang.Math.min;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.*;

@SuppressWarnings("deprecation")
class IterativeRowGenCVRPSolverTest {
    @BeforeAll
    static void setUp() {
        setUpHardware_raptorLake();
    }

    @Test
    @Disabled
    void eil33_iterativeRowGen() throws IOException {
        _eil33_iterative(8000, 30_000L);
    }

    @Test
    @Disabled
    void eil33_iterativeRowGen_insane() throws IOException {
        _eil33_iterative(4000, 300_000L);
    }

    private static void _eil33_iterative(int val, long timeout) throws IOException {
        var vrp = TsplibArchive.loadVrpInstance("eil33.vrp");
        var dim = min(vrp.dimension(), Integer.MAX_VALUE);
        var costs = new BigDecimal[dim][dim];
        var nodeCoords = vrp.nodeCoords();
        // round to 9 to more closely match the way this is handled in eil33-2 from MIPLIB.
        var mc = new MathContext(9, RoundingMode.HALF_EVEN);

        System.out.println(vrp);

        // total demand is 29370
        var demands = IntStream.of(0, 700, 400, 400, 1200, 40, 80, 2000, 900,
                        600, 750, 1500, 150, 250, 1600, 450, 700, 550, 650, 200, 400, 300,
                        1300, 700, 750, 1400, 4000, 600, 1000, 500, 2500, 1700, 1100)
                .limit(dim)
                .mapToObj(BigDecimal::valueOf).toArray(BigDecimal[]::new);

        for (int i = 0; i < dim; i++) {
            var row = costs[i];

            for (int j = 0; j < dim; j++) {
                row[j] = i <= j ? ZERO : BigDecimal.valueOf(getEdgeWeightNonRounded(i, j, nodeCoords)).round(mc);
            }
        }

        var solver = new IterativeRowGenCVRPSolver();
        var capacity = BigDecimal.valueOf(val);
        var minVehicles = 1;
        var start = System.currentTimeMillis();
        var result = solver.doSolve(minVehicles, capacity, demands, costs, timeout);

        System.out.println("Total elapsed: " + (System.currentTimeMillis() - start) + " ms");

        assertEquals(new CVRPSolver.Result(OPTIMAL, 837.67155201, Set.of(
                        List.of(0, 1, 15, 26, 27, 16, 28, 29),
                        List.of(0, 2, 12, 11, 32, 8, 9, 7, 4),
                        List.of(0, 3, 5, 6, 10, 18, 19, 21, 20, 22, 23, 24, 25, 17, 13),
                        List.of(0, 30, 14, 31))),
                result);
    }
}
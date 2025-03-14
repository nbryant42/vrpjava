package org.example;

import io.github.lmores.tsplib.TsplibArchive;
import org.example.OjAlgoCVRPSolver.GlobalBounds;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.math.BigDecimal;
import java.math.MathContext;
import java.math.RoundingMode;
import java.util.List;
import java.util.stream.IntStream;

import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.math.BigDecimal.ZERO;
import static org.example.CVRPSolver.Result;
import static org.example.OjAlgoCVRPSolver.base;
import static org.example.Util.setUpHardware;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

class OjAlgoCVRPSolverTest {
    @BeforeAll
    static void setUp() {
        setUpHardware();
    }

    /**
     * Test the EIL33 CVRP example from TSPLIB.
     * <p>
     * Infeasible in this codebase prior to implementation of the RCC-Sep algorithm, but now takes about 10-25 seconds.
     * (Runtime is a bit unpredictable because ojAlgo doesn't always come up with the same solutions each time.)
     * <p>
     * This exact solution greatly improves on the solution with cost ~934 published
     * <a href="https://github.com/IBMDecisionOptimization/Decision-Optimization-with-CPLEX-samples/blob/master/Vehicle-routing.pdf">here</a>.
     */
    @Test
    @Disabled
    void eil33() throws IOException {
        assertEquals(new Result(837.67155201, List.of(
                        List.of(0, 1, 15, 26, 27, 16, 28, 29),
                        List.of(0, 2, 12, 11, 32, 8, 9, 7, 4),
                        List.of(0, 3, 5, 6, 10, 18, 19, 21, 20, 22, 23, 24, 25, 17, 13),
                        List.of(0, 30, 14, 31))),
                doTestEil33(Integer.MAX_VALUE, 120_000L));
    }

    @Test
    @Disabled
    void eil33_iterativeDeepening() throws IOException {
        _eil33_iterative(8000, 30_000L);
    }

    @Test
    @Disabled
    void eil33_iterativeDeepening_insane() throws IOException {
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

        var solver = new OjAlgoCVRPSolver();
        var capacity = BigDecimal.valueOf(val);
        var minVehicles = 1;
        var start = System.currentTimeMillis();
        var result = solver.iterativeDeepeningSolver(minVehicles, dim - 1, capacity, demands, costs, timeout);

        System.out.println("Total elapsed: " + (System.currentTimeMillis() - start) + " ms");

        assertEquals(new Result(837.67155201, List.of(
                        List.of(0, 1, 15, 26, 27, 16, 28, 29),
                        List.of(0, 2, 12, 11, 32, 8, 9, 7, 4),
                        List.of(0, 3, 5, 6, 10, 18, 19, 21, 20, 22, 23, 24, 25, 17, 13),
                        List.of(0, 30, 14, 31))),
                result);
    }

    // takes 3-4 seconds
    @Test
    @Disabled
    void eil33_boundsOnly() throws IOException {
        var mr = (GlobalBounds) doTestEil33(Integer.MAX_VALUE, true, 1000L * 60L * 60L);
        assertEquals(835.227782375, mr.getResult().getValue());
    }

    @Test
    @Disabled
    void eil33_moreVehicles() throws IOException {
        var r = (Result) doTestEil33(Integer.MAX_VALUE, false, 300_000L, 4000);
        // best I've seen from a <= 5 minute run is 1539.9554.
        // I also saw some better results, but that took a lot of time after hardcoding the upper bound.
        // Best known (to me) solution is as follows, obtained on a Ryzen 9 3900X:
        //
        // [22.176s]: Bounds init complete. Bounds from heuristic: 1430.6775/1539.9556 (92.90%)
        // Currently 200 cuts.
        // [362.291s]: New incumbent. Bounds now 1430.6775/1539.1586 (92.95%)
        // [530.284s]: New incumbent. Bounds now 1430.6775/1534.6448 (93.23%)
        // 2851 nodes, 8 cycles: [[0, 1, 15, 17, 28, 16, 27], [0, 2, 10, 9, 8, 6, 5], [0, 3, 31, 14], [0, 4, 32, 11, 12], [0, 7, 22, 18], [0, 13, 19, 21, 20, 23, 24, 25], [0, 26], [0, 29, 30]]
        // Cycle demands: [4000, 2770, 3700, 3950, 3950, 4000, 4000, 3000]
        // Currently 1107 cuts.
        // Total elapsed: 900130 ms
        double v = 1674.9719;
        if (r.objective() > v) {
            fail("Objective > " + v);
        }
    }

    // slow, and doesn't always find the same bound.
    @Test
    @Disabled
    void eil33_moreVehicles_boundsOnly() throws IOException {
        var mr = (GlobalBounds) doTestEil33(Integer.MAX_VALUE, true, 1000L * 60L * 60L, 4000);
        assertEquals(1430.6774874709083, mr.getResult().getValue());
    }

    @Test
    void timeout() throws IOException {
        assertEquals(new Result(2875.8908523, List.of(
                        List.of(0, 3, 4, 2, 12, 30, 31, 5, 11, 6),
                        List.of(0, 7, 32, 1, 13, 8, 9, 10, 14),
                        List.of(0, 15, 17, 29, 26, 18, 19, 25),
                        List.of(0, 28, 27, 21, 22, 20, 24, 16, 23))),
                doTestEil33(Integer.MAX_VALUE, 0L));
    }


    // 240-254ms w/ naive code; now ~550-800 ms
    @Test
    void eil33_reduced() throws IOException {
        assertEquals(new Result(428.7145713, List.of(
                        List.of(0, 2, 12, 11, 10, 9, 8, 7, 6, 5, 4),
                        List.of(0, 3, 13, 1, 14, 15, 16))),
                doTestEil33(17));
    }

    @Test
    void eil33_reduced_boundsOnly() throws IOException {
        var mr = (GlobalBounds) doTestEil33(17, true, 1000L * 60L * 60L);
        assertEquals(422.810195025, mr.getResult().getValue());
    }

    // takes about 2.5s w/ naive code; 1.5s now
    // trivial -- solves at the root node -- so, also 1.5s with B&B
    @Test
    void eil33_reduced2() throws IOException {
        assertEquals(new Result(499.25424343, List.of(
                        List.of(0, 1, 14, 15, 17, 16, 23, 22, 20, 21, 19, 18, 13, 12),
                        List.of(0, 3, 2, 11, 10, 9, 8, 7, 6, 5, 4))),
                doTestEil33(24));
    }

    // more than 30s to run w/ naive code; 1.3s now. Trivially solvable at root node in B&B.
    @Test
    void eil33_reduced3() throws IOException {
        assertEquals(new Result(588.46611851, List.of(
                        List.of(0, 1, 13, 11, 10, 9, 8, 7, 6, 5, 12, 2, 3),
                        List.of(0, 4),
                        List.of(0, 14, 15, 17, 19, 18, 21, 20, 22, 23, 24, 16))),
                doTestEil33(25));
    }

    private static Result doTestEil33(int limit) throws IOException {
        return doTestEil33(limit, 1000L * 60L * 60L);
    }

    private static Result doTestEil33(int limit, long timeoutMillis) throws IOException {
        return (Result) doTestEil33(limit, false, timeoutMillis);
    }

    private static Object doTestEil33(int limit, boolean boundsOnly, long timeoutMillis) throws IOException {
        return doTestEil33(limit, boundsOnly, timeoutMillis, 8000);
    }

    private static Object doTestEil33(int limit, boolean boundsOnly, long timeoutMillis, int vehicleCapacity) throws IOException {
        var vrp = TsplibArchive.loadVrpInstance("eil33.vrp");
        var dim = min(vrp.dimension(), limit);
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

        var solver = new OjAlgoCVRPSolver();
        var capacity = BigDecimal.valueOf(vehicleCapacity);
        var minVehicles = 1;
        var start = System.currentTimeMillis();
        var result = boundsOnly ? solver.initBounds(minVehicles, dim - 1, capacity, demands, costs, timeoutMillis) :
                solver.solve(minVehicles, dim - 1, capacity, demands, costs, timeoutMillis);

        System.out.println("Total elapsed: " + (System.currentTimeMillis() - start) + " ms");

        return result;
    }

    // get non-rounded weights, to more closely match the way this is handled in eil33-2 from MIPLIB.
    private static double getEdgeWeightNonRounded(int i, int j, double[][] nodeCoords) {
        final double[] x = nodeCoords[i];
        final double[] y = nodeCoords[j];
        return nonRoundedEuclideanDistance(x[0], x[1], y[0], y[1]);
    }

    private static double nonRoundedEuclideanDistance(
            final double x1, final double y1, final double x2, final double y2) {
        return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    }

    @Test
    void testBase() {
        assertEquals(0, base(1));
        assertEquals(1, base(2));
        assertEquals(3, base(3));
        assertEquals(6, base(4));
    }
}
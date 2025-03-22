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
import java.util.stream.IntStream;

import static com.github.vrpjava.Util.setUpHardware;
import static com.github.vrpjava.cvrp.CVRPSolver.Result;
import static com.github.vrpjava.cvrp.Job.initBounds;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.base;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static org.ojalgo.optimisation.Optimisation.State.UNEXPLORED;

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
    void eil33() throws IOException {
        var solver = newSolver();
        var timeout = 300_000L;
        solver.setBestFirstMillis(timeout); // overriding here because JaCoCo slows tests down

        assertEquals(new Result(Result.State.OPTIMAL, 837.67155201, List.of(
                        List.of(0, 1, 15, 26, 27, 16, 28, 29),
                        List.of(0, 2, 12, 11, 32, 8, 9, 7, 4),
                        List.of(0, 3, 5, 6, 10, 18, 19, 21, 20, 22, 23, 24, 25, 17, 13),
                        List.of(0, 30, 14, 31))),
                doTestEil33(Integer.MAX_VALUE, false, timeout, 8000, solver));
    }

    private static OjAlgoCVRPSolver newSolver() {
        var solver = new OjAlgoCVRPSolver();
        solver.setDebug(true);
        return solver;
    }

    // takes 3-4 seconds
    @Test
    @Disabled
    void eil33_boundsOnly() throws IOException {
        var timeout = 1000L * 60L * 60L;
        var deadline = System.currentTimeMillis() + timeout;
        var mr = (GlobalBounds) doTestEil33(Integer.MAX_VALUE, true, timeout);
        assertEquals(835.227782375, mr.getResult(deadline).getValue());
    }

    @Test
    void eil33_boundsOnly_timeout() throws IOException {
        var mr = (GlobalBounds) doTestEil33(Integer.MAX_VALUE, true, 0L);
        assertEquals(UNEXPLORED, mr.getResult(System.currentTimeMillis()).getState());
    }

    @Test
    @Disabled
    void eil33_moreVehicles() throws IOException {
        var r = (Result) doTestEil33(Integer.MAX_VALUE, false, 300_000L, 4000);
        // Best known (to me) solution is as follows, obtained on a Ryzen 9 3900X:
        //
        // Currently 201 cuts.
        // [23.783s]: Initial solution. Bounds now 1430.6775/1996.0273 (71.68%)
        // [95.870s]: New solution. Bounds now 1430.6775/1490.0372 (96.02%)
        // [138.927s]: New solution. Bounds now 1430.6775/1489.5574 (96.05%)
        // [156.401s]: New solution. Bounds now 1430.6775/1488.7949 (96.10%)
        // [211.726s]: New solution. Bounds now 1430.6775/1486.3844 (96.25%)
        // 321 nodes, 8 cycles: [[0, 1, 14, 31], [0, 2, 12, 11, 4], [0, 3, 5, 6, 9, 10, 23, 22], [0, 7, 8, 32], [0, 13, 25, 24, 20, 21, 19, 18], [0, 15, 17, 27, 16, 28, 29], [0, 26], [0, 30]]
        // Cycle demands: [4000, 3250, 3870, 4000, 3950, 3800, 4000, 2500]
        // Currently 694 cuts.
        // Total elapsed: 300008 ms
        double v = 1674.9719;
        if (r.objective() > v) {
            fail("Objective > " + v);
        }
    }

    // slow, and doesn't always find the same bound.
    @Test
    @Disabled
    void eil33_moreVehicles_boundsOnly() throws IOException {
        var timeout = 1000L * 60L * 60L;
        var deadline = System.currentTimeMillis() + timeout;
        var mr = (GlobalBounds) doTestEil33(Integer.MAX_VALUE, true, timeout, 4000);
        assertEquals(1430.6774874709083, mr.getResult(deadline).getValue());
    }

    @Test
    void timeoutImmediate() throws IOException {
        assertEquals(new Result(Result.State.HEURISTIC, 1043.17356774, List.of(
                        List.of(0, 3, 5, 32, 13, 15, 17, 26, 28),
                        List.of(0, 4, 7, 8, 1, 14, 25),
                        List.of(0, 2, 11, 9, 18, 21, 20, 23, 16, 30),
                        List.of(0, 12, 6, 10, 19, 22, 24, 27, 29, 31))),
                doTestEil33(Integer.MAX_VALUE, 0L));
    }

    @Test
    void timeoutInRccSep() throws IOException {
        assertEquals(new Result(Result.State.HEURISTIC, 1043.17356774, List.of(
                        List.of(0, 3, 5, 32, 13, 15, 17, 26, 28),
                        List.of(0, 4, 7, 8, 1, 14, 25),
                        List.of(0, 2, 11, 9, 18, 21, 20, 23, 16, 30),
                        List.of(0, 12, 6, 10, 19, 22, 24, 27, 29, 31))),
                doTestEil33(Integer.MAX_VALUE, 100L));
    }

    @Test
    void timeoutInNode() throws IOException {
        var solver = newSolver();
        solver.setBestFirstMillis(0L); // deliberately slow this down
        Result result = (Result) doTestEil33(Integer.MAX_VALUE, false, 25_000L, 8000, solver);
        assertTrue(result.state().isFeasible());
    }

    // 240-254ms w/ naive code; now ~550-800 ms
    @Test
    void eil33_reduced() throws IOException {
        assertEquals(new Result(Result.State.OPTIMAL, 428.7145713, List.of(
                        List.of(0, 2, 12, 11, 10, 9, 8, 7, 6, 5, 4),
                        List.of(0, 3, 13, 1, 14, 15, 16))),
                doTestEil33(17));
    }

    @Test
    void eil33_reduced_boundsOnly() throws IOException {
        var timeout = 1000L * 60L * 60L;
        var deadline = System.currentTimeMillis() + timeout;
        var mr = (GlobalBounds) doTestEil33(17, true, timeout);
        assertEquals(422.810195025, mr.getResult(deadline).getValue());
    }

    // takes about 2.5s w/ naive code; 1.5s now
    // trivial -- solves at the root node -- so, also 1.5s with B&B
    @Test
    void eil33_reduced2() throws IOException {
        assertEquals(new Result(Result.State.OPTIMAL, 499.25424343, List.of(
                        List.of(0, 1, 14, 15, 17, 16, 23, 22, 20, 21, 19, 18, 13, 12),
                        List.of(0, 3, 2, 11, 10, 9, 8, 7, 6, 5, 4))),
                doTestEil33(24));
    }

    // more than 30s to run w/ naive code; 1.3s now. Trivially solvable at root node in B&B.
    @Test
    void eil33_reduced3() throws IOException {
        assertEquals(new Result(Result.State.OPTIMAL, 588.46611851, List.of(
                        List.of(0, 1, 13, 11, 10, 9, 8, 7, 6, 5, 12, 2, 3),
                        List.of(0, 4),
                        List.of(0, 14, 15, 17, 19, 18, 21, 20, 22, 23, 24, 16))),
                doTestEil33(25));
    }

    @Test
    void infeasible() {
        var FORTYFIVE = BigDecimal.valueOf(45);

        var result = newSolver().solve(1, 2, BigDecimal.valueOf(70),
                new BigDecimal[]{ZERO, FORTYFIVE, FORTYFIVE, BigDecimal.valueOf(50)},
                new BigDecimal[][]{
                        {ZERO, ZERO, ZERO, ZERO},
                        {ONE, ZERO, ZERO, ZERO},
                        {ONE, ONE, ZERO, ZERO},
                        {ONE, ONE, ONE, ZERO},
                });

        assertEquals(new Result(Result.State.INFEASIBLE, Double.POSITIVE_INFINITY, List.of()), result);
    }

    @Test
    void infeasible_timeout() {
        var FORTYFIVE = BigDecimal.valueOf(45);

        var result = newSolver().solve(1, 2, BigDecimal.valueOf(70),
                new BigDecimal[]{ZERO, FORTYFIVE, FORTYFIVE, BigDecimal.valueOf(50)},
                new BigDecimal[][]{
                        {ZERO, ZERO, ZERO, ZERO},
                        {ONE, ZERO, ZERO, ZERO},
                        {ONE, ONE, ZERO, ZERO},
                        {ONE, ONE, ONE, ZERO},
                }, 0L);

        assertEquals(new Result(Result.State.UNEXPLORED, Double.POSITIVE_INFINITY, List.of()), result);
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
        return doTestEil33(limit, boundsOnly, timeoutMillis, vehicleCapacity, newSolver());
    }

    private static Object doTestEil33(int limit, boolean boundsOnly, long timeoutMillis, int vehicleCapacity, OjAlgoCVRPSolver solver) throws IOException {
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

        var capacity = BigDecimal.valueOf(vehicleCapacity);
        var minVehicles = 1;
        var start = System.currentTimeMillis();
        var result = boundsOnly ?
                initBounds(minVehicles, dim - 1, capacity, demands, costs, start + timeoutMillis) :
                solver.solve(minVehicles, dim - 1, capacity, demands, costs, timeoutMillis);

        System.out.println("Total elapsed: " + (System.currentTimeMillis() - start) + " ms");

        return result;
    }

    // get non-rounded weights, to more closely match the way this is handled in eil33-2 from MIPLIB.
    static double getEdgeWeightNonRounded(int i, int j, double[][] nodeCoords) {
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
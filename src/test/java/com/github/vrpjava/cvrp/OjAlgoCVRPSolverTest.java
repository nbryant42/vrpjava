package com.github.vrpjava.cvrp;

import io.github.lmores.tsplib.TsplibArchive;
import io.github.lmores.tsplib.vrp.VrpInstance;
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
import static com.github.vrpjava.cvrp.CVRPSolver.Result;
import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.HEURISTIC;
import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.OPTIMAL;
import static com.github.vrpjava.cvrp.Job.initBounds;
import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.base;
import static java.lang.Math.min;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static java.math.BigDecimal.ZERO;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static org.ojalgo.optimisation.Optimisation.State.UNEXPLORED;

class OjAlgoCVRPSolverTest extends AbstractCVRPSolverTest {
    @BeforeAll
    static void setUp() {
        setUpHardware_raptorLake();
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
        // old heuristic 835.2278/1043.1736 (80.07%)
        // new heuristic 835.2278/843.0978 (99.07%)
        try (var solver = newSolver()) {
            var timeout = 300_000L;
            solver.setBestFirstMillis(timeout); // overriding here because JaCoCo slows tests down

            assertEquals(new Result(OPTIMAL, 837.67155201, Set.of(
                            List.of(0, 1, 15, 26, 27, 16, 28, 29),
                            List.of(0, 2, 12, 11, 32, 8, 9, 7, 4),
                            List.of(0, 3, 5, 6, 10, 18, 19, 21, 20, 22, 23, 24, 25, 17, 13),
                            List.of(0, 30, 14, 31))),
                    doTestEil33(Integer.MAX_VALUE, false, timeout, 8000, solver));
        }
    }

    /**
     * Like {@link #eil33()}, but rounds the distances to integers for a
     * direct comparison with most of the academic literature, such as
     * <a href="https://www.lancaster.ac.uk/staff/letchfoa/articles/2004-cvrp-exact.pdf">Lysgaard et al.</a>
     * <p>
     * TODO: Notice that we search more nodes than they do, and our bounds are slightly weaker at the root node
     * (Lysgaard et al., Table 7). From their paper, it looks like our bounds could be tightened by implementing
     * multistar inequalities (Table 5) and/or hypotour inequalities (Table 6). The larger number of nodes may also be
     * due to their odd-edges cut-set branching strategy.
     */
    @Test
    void eil33_rounded() throws IOException {
        try (var solver = newSolver()) {
            var timeout = 300_000L;
            solver.setBestFirstMillis(timeout); // overriding here because JaCoCo slows tests down

            var actual = (Result) doTestEil33(Integer.MAX_VALUE, false, timeout, 8000, solver, true);

            assertEquals(OPTIMAL, actual.state());
            assertEquals(835.0, actual.objective());
            assertEquals(4, actual.cycles().size());
        }
    }

    // variant with 5 vehicles instead of 4. still solves quite quickly.
    @Test
    @Disabled
    void eil33_k5() throws IOException {
        try (var solver = newSolver()) {
            var timeout = 300_000L;
            solver.setBestFirstMillis(timeout); // overriding here because JaCoCo slows tests down

            var actual = (Result) doTestEil33(Integer.MAX_VALUE, false, timeout, 6000, solver, true);

            assertEquals(OPTIMAL, actual.state());
            assertEquals(1019.0, actual.objective());
            assertEquals(5, actual.cycles().size());
        }
    }

    // variant with 6 vehicles instead of 4. no solution in sight after 5 minutes
    @Test
    @Disabled
    void eil33_k6() throws IOException {
        try (var solver = newSolver()) {
            var timeout = 10_000L;
            solver.setBestFirstMillis(timeout); // overriding here because JaCoCo slows tests down

            var actual = (Result) doTestEil33(Integer.MAX_VALUE, false, timeout, 5384, solver, true);

            assertEquals(HEURISTIC, actual.state());
            assertEquals(1222.0, actual.objective());
            assertEquals(6, actual.cycles().size());
        }
    }

    protected OjAlgoCVRPSolver newSolver() {
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
        // old heuristic 1430.6775/1996.0273 (71.68%)
        // new heuristic 1430.6775/1535.6805 (93.16%)
        //
        // Logs for the best known (to me) solution:
        //
        // Currently 228 cuts.
        // [19.558s]: Initial solution. Bounds now 1430.6775/1535.6805 (93.16%)
        // [51.750s]: New solution. Bounds now 1430.6775/1462.7662 (97.81%)
        // [88.375s]: New solution. Bounds now 1430.6775/1456.1049 (98.25%)
        // 388 nodes, 8 cycles: [[0, 4, 7], [0, 15, 17, 27, 16, 28, 29], [0, 2, 12, 11, 22, 18], [0, 13, 19, 21, 20, 23, 24, 25], [0, 3, 5, 6, 8, 9, 10, 32], [0, 26], [0, 1, 14, 31], [0, 30]]
        // Cycle demands: [3200, 3800, 4000, 4000, 3870, 4000, 4000, 2500]
        // Currently 649 cuts.
        // Total elapsed: 300015 ms
        double v = 1535.6806;
        if (r.objective() > v) {
            fail("Objective > " + v);
        }
    }

    @Test
    @Disabled
    void eil33_moreVehicles_rounded() throws IOException {
        try (var solver = newSolver()) {
            var r = (Result) doTestEil33(Integer.MAX_VALUE, false, 3_600_000L, 4000, solver, true);
            // Best so far:
            // [9.239s]: Switching to best-first search. Bounds now 1427.166666634/1531.0 (93.22%); 203 cuts, 0 nodes.
            // [219.190s]: New solution. Bounds now 1427.166666634/1455.0 (98.09%); 652 cuts, 7320 nodes.
            // [356.903s]: New solution. Bounds now 1427.166666624/1454.0 (98.15%); 709 cuts, 10502 nodes.
            // [1310.753s]: New solution. Bounds now 1427.166666634/1451.0 (98.36%); 1029 cuts, 19445 nodes.
            // [1314.813s]: New solution. Bounds now 1427.166666634/1450.0 (98.43%); 1032 cuts, 19463 nodes.
            // 29280 nodes, 8 cycles: [[0, 12, 21, 20, 23, 24, 25], [0, 2, 11, 32, 8, 6], [0, 15, 17, 27, 16, 28, 29], [0, 5, 9, 10, 18, 19, 22, 13], [0, 26], [0, 1, 14, 31], [0, 3, 7, 4], [0, 30]]
            // Cycle demands: [3700, 3980, 3800, 3790, 4000, 4000, 3600, 2500]
            // Currently 1197 cuts.
            // Total elapsed: 3600026 ms

            double v = 1531.0;
            if (r.objective() > v) {
                fail("Objective > " + v);
            }
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
        assertEquals(new Result(Result.State.HEURISTIC, 843.09779333, Set.of(
                        List.of(0, 3, 1, 14, 31, 30),
                        List.of(0, 13, 15, 17, 25, 24, 23, 20, 22, 21, 19, 18, 10, 6, 5),
                        List.of(0, 29, 28, 16, 27, 26),
                        List.of(0, 2, 12, 11, 32, 8, 9, 7, 4))),
                doTestEil33(Integer.MAX_VALUE, 0L));
    }

    @Test
    void timeoutInRccSep() throws IOException {
        assertEquals(new Result(Result.State.HEURISTIC, 843.09779333, Set.of(
                        List.of(0, 3, 1, 14, 31, 30),
                        List.of(0, 13, 15, 17, 25, 24, 23, 20, 22, 21, 19, 18, 10, 6, 5),
                        List.of(0, 29, 28, 16, 27, 26),
                        List.of(0, 2, 12, 11, 32, 8, 9, 7, 4))),
                doTestEil33(Integer.MAX_VALUE, 100L));
    }

    @Test
    void timeoutInNode() throws IOException {
        try (var solver = newSolver()) {
            solver.setBestFirstMillis(0L); // deliberately slow this down
            var result = (Result) doTestEil33(Integer.MAX_VALUE, false, 25_000L, 8000, solver);
            assertTrue(result.state().isFeasible());
        }
    }

    // 240-254ms w/ naive code; now ~550-800 ms
    @Test
    void eil33_reduced() throws IOException {
        // old heuristic 422.81018/477.28214 (88.59%)
        // new heuristic 422.81018/432.9653 (97.65%)
        assertEquals(new Result(OPTIMAL, 428.7145713, Set.of(
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
        // old heuristic 499.25424/721.02826 (69.24%)
        // new heuristic 499.25424/595.4756 (83.84%)
        assertEquals(new Result(OPTIMAL, 499.25424343, Set.of(
                        List.of(0, 1, 14, 15, 17, 16, 23, 22, 20, 21, 19, 18, 13, 12),
                        List.of(0, 3, 2, 11, 10, 9, 8, 7, 6, 5, 4))),
                doTestEil33(24));
    }

    // more than 30s to run w/ naive code; 1.3s now. Trivially solvable at root node in B&B.
    @Test
    void eil33_reduced3() throws IOException {
        // old heuristic 588.4661/832.13934 (70.72%)
        // new heuristic 588.4661/593.60974 (99.13%)
        assertEquals(new Result(OPTIMAL, 588.46611851, Set.of(
                        List.of(0, 1, 13, 11, 10, 9, 8, 7, 6, 5, 12, 2, 3),
                        List.of(0, 4),
                        List.of(0, 14, 15, 17, 19, 18, 21, 20, 22, 23, 24, 16))),
                doTestEil33(25));
    }

    private Result doTestEil33(int limit) throws IOException {
        return doTestEil33(limit, 1000L * 60L * 60L);
    }

    private Result doTestEil33(int limit, long timeoutMillis) throws IOException {
        return (Result) doTestEil33(limit, false, timeoutMillis);
    }

    private Object doTestEil33(int limit, boolean boundsOnly, long timeoutMillis) throws IOException {
        return doTestEil33(limit, boundsOnly, timeoutMillis, 8000);
    }

    private Object doTestEil33(int limit, boolean boundsOnly, long timeoutMillis, int vehicleCapacity)
            throws IOException {
        try (var solver = newSolver()) {
            return doTestEil33(limit, boundsOnly, timeoutMillis, vehicleCapacity, solver);
        }
    }

    private static Object doTestEil33(int limit, boolean boundsOnly, long timeoutMillis, int vehicleCapacity,
                                      OjAlgoCVRPSolver solver) throws IOException {
        return doTestEil33(limit, boundsOnly, timeoutMillis, vehicleCapacity, solver, false);
    }

    private static Object doTestEil33(int limit, boolean boundsOnly, long timeoutMillis, int vehicleCapacity,
                                      OjAlgoCVRPSolver solver, boolean round) throws IOException {
        var vrp = TsplibArchive.loadVrpInstance("eil33.vrp");
        var dim = min(vrp.dimension(), limit);

        System.out.println(vrp);

        // total demand is 29370
        var demands = IntStream.of(0, 700, 400, 400, 1200, 40, 80, 2000, 900,
                        600, 750, 1500, 150, 250, 1600, 450, 700, 550, 650, 200, 400, 300,
                        1300, 700, 750, 1400, 4000, 600, 1000, 500, 2500, 1700, 1100)
                .limit(dim)
                .mapToObj(BigDecimal::valueOf).toArray(BigDecimal[]::new);

        var costs = buildCosts(round, dim, vrp);
        var capacity = BigDecimal.valueOf(vehicleCapacity);
        var minVehicles = 1;
        var start = System.currentTimeMillis();
        var result = boundsOnly ?
                initBounds(minVehicles, capacity, demands, costs, start + timeoutMillis) :
                solver.solve(minVehicles, capacity, demands, costs, timeoutMillis);

        System.out.println("Total elapsed: " + (System.currentTimeMillis() - start) + " ms");

        return result;
    }

    private static BigDecimal[][] buildCosts(boolean round, int dim, VrpInstance vrp) {
        // round to 9 to more closely match the way this is handled in eil33-2 from MIPLIB.
        var mc = new MathContext(9, RoundingMode.HALF_EVEN);
        var nodeCoords = vrp.nodeCoords();
        var costs = new BigDecimal[dim][dim];

        for (int i = 0; i < dim; i++) {
            var row = costs[i];

            for (int j = 0; j < dim; j++) {
                if (i <= j) {
                    row[j] = ZERO;
                } else {
                    row[j] = round ? BigDecimal.valueOf(vrp.getEdgeWeight(i, j)) :
                            BigDecimal.valueOf(getEdgeWeightNonRounded(i, j, nodeCoords)).round(mc);
                }
            }
        }
        return costs;
    }

    @Test
    @Disabled
    void eil51() throws IOException {
        var timeout = 300_000L;

        try (var solver = newSolver()) {
            var result = doTestEil51(solver, timeout, 160);

            assertEquals(OPTIMAL, result.state());
            assertEquals(521.0, result.objective());
        }
    }

    // variant with 6 vehicles instead of 5. no solution in sight after 5 minutes.
    @Test
    @Disabled
    void eil51_k6() throws IOException {
        var timeout = 30_000L;

        try (var solver = newSolver()) {
            var result = doTestEil51(solver, timeout, 155);

            assertEquals(HEURISTIC, result.state());
            assertEquals(589.0, result.objective());
        }
    }


    Result doTestEil51(CVRPSolver solver, long timeout, int capacity) throws IOException {
        var vrp = TsplibArchive.loadVrpInstance("eil51.vrp");
        var dim = vrp.dimension();

        System.out.println(vrp);

        // total demand 777
        var demands = IntStream.of(0, 7, 30, 16, 9, 21, 15, 19, 23, 11, 5, 19, 29, 23, 21, 10, 15, 3, 41, 9, 28,
                        8, 8, 16, 10, 28, 7, 15, 14, 6, 19, 11, 12, 23, 26, 17, 6, 9, 15, 14, 7, 27, 13, 11, 16, 10, 5,
                        25, 17, 18, 10)
                .limit(dim)
                .mapToObj(BigDecimal::valueOf).toArray(BigDecimal[]::new);

        var costs = buildCosts(true, dim, vrp);
        var minVehicles = 1;
        var start = System.currentTimeMillis();
        var result = solver.solve(minVehicles, BigDecimal.valueOf(capacity), demands, costs, timeout);

        System.out.println("Total elapsed: " + (System.currentTimeMillis() - start) + " ms");

        return result;
    }

    // This case has been solved to optimality by others, but Lysgaard et al. had to run their code for about 33 hours.
    @Test
    @Disabled
    void eilD76() throws IOException {
        var timeout = 300_000L;

        try (var solver = newSolver()) {
            var result = doTestEilD76(solver, timeout, 220);

            assertEquals(HEURISTIC, result.state());
            assertEquals(730.0, result.objective());
        }
    }

    // I think this is probably the same as the P-n76-k4 from Lysgaard et al.
    @Test
    @Disabled
    void eilD76_k4() throws IOException {
        var timeout = 1800_000L;

        try (var solver = newSolver()) {
            var result = doTestEilD76(solver, timeout, 360);

            //assertEquals(OPTIMAL, result.state());
            assertEquals(593.0, result.objective());
        }
    }


    Result doTestEilD76(CVRPSolver solver, long timeout, int capacity) throws IOException {
        var vrp = TsplibArchive.loadVrpInstance("eilD76.vrp");
        var dim = vrp.dimension();

        System.out.println(vrp);

        // total demand 1364
        var demands = IntStream.of(0, 18, 26, 11, 30, 21, 19, 15, 16, 29, 26, 37, 16, 12, 31, 8, 19, 20,
                        13, 15, 22, 28, 12, 6, 27, 14, 18, 17, 29, 13, 22, 25, 28, 27, 19, 10, 12, 14, 24, 16,
                        33, 15, 11, 18, 17, 21, 27, 19, 20, 5, 22, 12, 19, 22, 16, 7, 26, 14, 21, 24, 13, 15, 18,
                        11, 28, 9, 37, 30, 10, 8, 11, 3, 1, 6, 10, 20)
                .limit(dim)
                .mapToObj(BigDecimal::valueOf).toArray(BigDecimal[]::new);

        var costs = buildCosts(true, dim, vrp);
        var minVehicles = 1;
        var start = System.currentTimeMillis();
        var result = solver.solve(minVehicles, BigDecimal.valueOf(capacity), demands, costs, timeout);

        System.out.println("Total elapsed: " + (System.currentTimeMillis() - start) + " ms");

        return result;
    }

    // has been solved by others, but it's very hard.
    @Test
    @Disabled
    void eilA101_k8() throws IOException {
        var timeout = 300_000L;

        try (var solver = newSolver()) {
            var result = doTestEilA101(solver, timeout, 200);

            assertEquals(OPTIMAL, result.state());
            assertEquals(521.0, result.objective());
        }
    }

    // This one's interesting. I think it's probably the same "P-n101-k4" used by Lysgaard et al.
    // Our time to bound the root node can be competitive with theirs, but we search a lot more nodes to find the
    // optimum, and sometimes ojAlgo gets stuck solving the RCC-Sep model. (It would be nice to have CPLEX here.)
    // Example log:
    // [87.473s]: Switching to best-first search. Bounds now 673.425729429/758.0 (88.84%); 118 cuts, 0 nodes. Next node has bound null
    // [211.168s]: New solution. Bounds now 675.825/685.0 (98.66%); 209 cuts, 2502 nodes. Next node has bound 681.0
    // [213.111s]: New solution. Bounds now 675.825/682.0 (99.09%); 210 cuts, 2534 nodes. Next node has bound 681.0
    // [308.909s]: New solution. Bounds now 675.825/681.0 (99.24%); 220 cuts, 3989 nodes. Next node has bound 681.0
    // 6045 nodes, 4 cycles: [[0, 27, 69, 1, 50, 33, 81, 9, 51, 30, 70, 10, 62, 11, 19, 48, 82, 7, 88, 31, 52], [0, 28, 76, 77, 3, 79, 78, 34, 35, 71, 65, 66, 20, 32, 90, 63, 64, 49, 36, 47, 46, 8, 45, 17, 84, 5, 60, 83, 18, 89], [0, 13, 58, 40, 21, 73, 72, 74, 22, 41, 75, 56, 23, 67, 39, 4, 25, 55, 54, 24, 29, 68, 80, 12, 26, 53], [0, 6, 96, 99, 59, 92, 93, 98, 37, 100, 91, 85, 61, 16, 86, 44, 38, 14, 42, 43, 15, 57, 2, 87, 97, 95, 94]]
    // Cycle demands: [300, 382, 384, 392]
    // Currently 220 cuts.
    // Total elapsed: 308915 ms
    @Test
    @Disabled
    void eilA101_k4() throws IOException {
        var timeout = 900_000L;

        try (var solver = newSolver()) {
            solver.setBestFirstMillis(timeout);
            var result = doTestEilA101(solver, timeout, 400);

            assertEquals(OPTIMAL, result.state());
            assertEquals(681.0, result.objective());
        }
    }

    Result doTestEilA101(CVRPSolver solver, long timeout, int capacity) throws IOException {
        var vrp = TsplibArchive.loadVrpInstance("eilA101.vrp");
        var dim = vrp.dimension();

        System.out.println(vrp);

        // total demand 1458

        var demands = IntStream.of(0, 10, 7, 13, 19, 26, 3, 5, 9, 16, 16, 12, 19, 23, 20, 8, 19, 2, 12, 17, 9,
                        11, 18, 29, 3, 6, 17, 16, 16, 9, 21, 27, 23, 11, 14, 8, 5, 8, 16, 31, 9, 5, 5, 7, 18, 16, 1, 27,
                        36, 30, 13, 10, 9, 14, 18, 2, 6, 7, 18, 28, 3, 13, 19, 10, 9, 20, 25, 25, 36, 6, 5, 15, 25, 9,
                        8, 18, 13, 14, 3, 23, 6, 26, 16, 11, 7, 41, 35, 26, 9, 15, 3, 1, 2, 22, 27, 20, 11, 12, 10, 9,
                        17)
                .limit(dim)
                .mapToObj(BigDecimal::valueOf).toArray(BigDecimal[]::new);

        var costs = buildCosts(true, dim, vrp);
        var minVehicles = 1;
        var start = System.currentTimeMillis();
        var result = solver.solve(minVehicles, BigDecimal.valueOf(capacity), demands, costs, timeout);

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

    @Test
    void roundBound() {
        // This case looks like numerical instability, so round to 10-digit precision before taking the ceiling.
        assertEquals(1132.0, Worker.roundBound(1132.000000148, 0));
        assertEquals(1132.0, Worker.roundBound(1131.999999904, 0));
    }
}
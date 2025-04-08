package com.github.vrpjava.atsp;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.math.BigDecimal;
import java.util.List;
import java.util.Random;

import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
import static java.util.Arrays.deepToString;
import static com.github.vrpjava.Util.setUpHardware_raptorLake;
import static org.junit.jupiter.api.Assertions.assertEquals;

class OjAlgoATSPSolverTest {
    private final ObjectMapper mapper = new ObjectMapper();

    @BeforeAll
    static void setUp() {
        setUpHardware_raptorLake();
    }

    /**
     * Solve an ATSP instance with 4 destinations A, B, C, and D.
     * <p>
     * Costs for A-B, B-A, C-D, and D-C are 1.0. All other costs are 2.0.
     * <p>
     * This implies that the solver will find two short cycles A-B-A and C-D-C
     * unless it correctly handles the single-cycle constraints.
     */
    @Test
    void findsExactlyOneCycle() {
        var result = new OjAlgoATSPSolver().solve(new BigDecimal[][]{
                        {ZERO, ONE, TWO, TWO},
                        {ONE, ZERO, TWO, TWO},
                        {TWO, TWO, ZERO, ONE},
                        {TWO, TWO, ONE, ZERO}})
                .stream().map(Edge::src).toList();

        assertEquals(List.of(0, 1, 2, 3), result);
    }

    @Test
    @Disabled
    void randomLargeProblem() {
        var size = 128;
        var rand = new Random();
        var problem = new BigDecimal[size][size];

        for (int row = 0; row < size; row++) {
            for (int col = 0; col < size; col++) {
                problem[row][col] = row == col ? ZERO : toBigDecimal(rand.nextDouble() + 0.0001);
            }
        }

        System.out.println(deepToString(problem));

        var start = System.currentTimeMillis();
        new OjAlgoATSPSolver().solve(problem);

        System.out.println("Total elapsed: " + (System.currentTimeMillis() - start) + "ms");
    }

    @Test
    @Disabled
    void staticLargeProblem() throws IOException {
        var problem = toCostMatrix(mapper.readValue(getClass().getResource("large-problem.json"), List.class));

        var start = System.currentTimeMillis();
        new OjAlgoATSPSolver().solve(problem, 10_000L);

        // with timeout: Elapsed: 97951ms; FEASIBLE 2.2934030790251323
        // without: Elapsed: 130710ms; OPTIMAL 1.751656246233827
        System.out.println("Total elapsed: " + (System.currentTimeMillis() - start) + "ms");
    }

    private static BigDecimal[][] toCostMatrix(List<?> json) {
        return json.stream().map(OjAlgoATSPSolverTest::toBigDecimals).toArray(BigDecimal[][]::new);
    }

    private static BigDecimal[] toBigDecimals(Object row) {
        return ((List<?>) row).stream().map(OjAlgoATSPSolverTest::toBigDecimal).toArray(BigDecimal[]::new);
    }

    private static BigDecimal toBigDecimal(Object n) {
        return n instanceof Integer ? BigDecimal.valueOf((Integer) n) : BigDecimal.valueOf((Double) n);
    }
}
package com.github.vrpjava.cvrp;

import org.junit.jupiter.api.Test;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.HEURISTIC;
import static com.github.vrpjava.cvrp.CVRPSolver.Result.State.OPTIMAL;
import static java.math.BigDecimal.*;
import static org.junit.jupiter.api.Assertions.*;

class FixedSolutionCVRPSolverTest {
    private static final BigDecimal[] DEMANDS = {ZERO, ONE, ONE};
    private static final BigDecimal[][] COSTS = {
            {ZERO, ZERO, ZERO}, {TWO, ZERO, ZERO}, {BigDecimal.valueOf(3), BigDecimal.valueOf(4), ZERO}};

    @Test
    void snapshotsRoutesAndRecomputesObjectiveForEachInstance() {
        var route = new ArrayList<>(List.of(0, 1, 2));
        var routes = new ArrayList<List<Integer>>();
        routes.add(route);
        var solver = new FixedSolutionCVRPSolver(routes);
        route.clear();
        routes.clear();
        var result = solver.solve(1, TWO, DEMANDS, COSTS, 0);
        assertEquals(HEURISTIC, result.state());
        assertEquals(9.0, result.objective());
        assertEquals(Set.of(List.of(0, 1, 2)), result.cycles());
        assertThrows(UnsupportedOperationException.class, () -> result.cycles().clear());
        assertThrows(UnsupportedOperationException.class, () -> result.cycles().iterator().next().clear());
        BigDecimal[][] otherCosts = {{ZERO, ZERO, ZERO}, {ONE, ZERO, ZERO}, {ONE, ONE, ZERO}};
        assertEquals(3.0, solver.solve(1, TWO, DEMANDS, otherCosts, 0).objective());
        assertThrows(IllegalArgumentException.class, () -> solver.solve(1, ONE, DEMANDS, COSTS, 0));
    }

    @Test
    void rejectsMalformedOrIncompleteRoutes() {
        List<List<List<Integer>>> invalid = List.of(
                List.of(), List.of(List.of()), List.of(List.of(0)), List.of(List.of(1, 2)),
                List.of(List.of(0, 1)), List.of(List.of(0, 1, 2, 0)), List.of(List.of(0, -1, 2)),
                List.of(List.of(0, 1, 3)), List.of(List.of(0, 1, 1, 2)),
                List.of(List.of(0, 1), List.of(0, 1, 2)),
                List.of(List.of(0, 1, 2), List.of(0, 1, 2)));
        for (var routes : invalid) {
            var solver = new FixedSolutionCVRPSolver(routes);
            assertThrows(IllegalArgumentException.class, () -> solver.solve(1, TWO, DEMANDS, COSTS, 0),
                    routes.toString());
        }
    }

    @Test
    void checksMinimumRouteCountAndCountsSingleCustomerReturnEdges() {
        var single = new FixedSolutionCVRPSolver(List.of(List.of(0, 1, 2)));
        assertThrows(IllegalArgumentException.class, () -> single.solve(2, TWO, DEMANDS, COSTS, 0));
        var multiple = new FixedSolutionCVRPSolver(List.of(List.of(0, 1), List.of(0, 2)));
        assertEquals(10.0, multiple.solve(1, ONE, DEMANDS, COSTS, 0).objective());
    }

    @Test
    void rejectsAnOverloadedRouteEvenWhenTheFleetHasEnoughTotalCapacity() {
        var solver = new FixedSolutionCVRPSolver(List.of(List.of(0, 1, 2), List.of(0, 3)));
        BigDecimal[] demands = {ZERO, TWO, TWO, ONE};
        BigDecimal[][] costs = {
                {ZERO, ZERO, ZERO, ZERO}, {ONE, ZERO, ZERO, ZERO},
                {ONE, ONE, ZERO, ZERO}, {ONE, ONE, ONE, ZERO}};
        var error = assertThrows(IllegalArgumentException.class,
                () -> solver.solve(1, BigDecimal.valueOf(3), demands, costs, 0));
        assertTrue(error.getMessage().contains("exceeds vehicle capacity"));
    }

    @Test
    void exactSolverAcceptsSeedAndCanProveItOptimal() {
        var routes = Set.of(List.of(0, 1), List.of(0, 2));
        try (var solver = new OjAlgoCVRPSolver()) {
            solver.setHeuristic(new FixedSolutionCVRPSolver(routes));
            var result = solver.solve(1, ONE, DEMANDS, COSTS, 10_000);
            assertEquals(OPTIMAL, result.state());
            assertEquals(10.0, result.objective());
            assertEquals(routes, result.cycles());
        }
    }

    @Test
    void exactSolverRejectsInvalidSeedBeforeUsingItsBound() {
        try (var solver = new OjAlgoCVRPSolver()) {
            solver.setHeuristic(new FixedSolutionCVRPSolver(List.of(List.of(0, 1))));
            assertThrows(IllegalArgumentException.class, () -> solver.solve(1, TWO, DEMANDS, COSTS, 10_000));
        }
    }
}

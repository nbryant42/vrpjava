package com.github.vrpjava.cvrp;

class NearestNeighborCVRPSolverTest extends AbstractCVRPSolverTest {
    @Override
    protected NearestNeighborCVRPSolver newSolver() {
        return new NearestNeighborCVRPSolver();
    }
}
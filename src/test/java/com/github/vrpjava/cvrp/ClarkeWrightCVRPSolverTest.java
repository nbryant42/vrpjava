package com.github.vrpjava.cvrp;

class ClarkeWrightCVRPSolverTest extends AbstractCVRPSolverTest {
    @Override
    protected CVRPSolver newSolver() {
        return new ClarkeWrightCVRPSolver();
    }
}
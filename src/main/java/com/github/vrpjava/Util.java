package com.github.vrpjava;

import org.ojalgo.OjAlgoUtils;
import org.ojalgo.machine.BasicMachine;
import org.ojalgo.machine.Hardware;
import org.ojalgo.machine.VirtualMachine;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.Variable;
import org.ojalgo.type.CalendarDateDuration;
import org.ojalgo.type.context.NumberContext;

import java.math.BigDecimal;
import java.util.Arrays;
import java.util.stream.IntStream;

import static java.lang.Math.ceilDiv;
import static org.ojalgo.type.CalendarDateUnit.MILLIS;

/**
 * Miscellaneous utilities.
 */
public class Util {
    private Util() {
    }

    private static final long K = 1024L;

    /**
     * Validate the cost matrix. For asymmetric use-cases
     * (currently only used by {@link com.github.vrpjava.atsp.ATSPSolver})
     *
     * @param costMatrix a matrix, which must be square, at least 2x2 and have zeroes on the diagonal.
     * @throws IllegalArgumentException if invalid
     */
    public static void validate(BigDecimal[][] costMatrix) {
        var size = costMatrix.length;

        if (size < 2) {
            throw new IllegalArgumentException("costMatrix must be at least 2x2");
        }
        if (Arrays.stream(costMatrix).anyMatch(row -> row.length != size)) {
            throw new IllegalArgumentException("costMatrix must be square");
        }
        if (IntStream.range(0, size).anyMatch(i -> costMatrix[i][i].signum() != 0)) {
            throw new IllegalArgumentException("costMatrix must have zeroes on the diagonal");
        }
    }

    /**
     * Indicate whether the matrix is lower-triangular, meaning the only non-zeroes are below the diagonal.
     *
     * @param matrix a square matrix.
     * @return true or false
     */
    public static boolean isLowerTriangular(BigDecimal[][] matrix) {
        var size = matrix.length;

        return IntStream.range(0, size).noneMatch(row ->
                IntStream.range(row, size).anyMatch(col ->
                        matrix[row][col].signum() != 0));
    }

    /**
     * Helper to build a new {@link ExpressionsBasedModel} for ojAlgo. This currently has a default set
     * of options to control rounding, timeouts, and use the dual simplex solver by default.
     *
     * @param deadline this will be converted to a timeout
     * @return the built model
     */
    public static ExpressionsBasedModel newModel(long deadline) {
        var options = setTimeout(deadline, new Optimisation.Options());
        //options.feasibility = NumberContext.of(14, 9);
        options.solution = NumberContext.of(14, 9); // TODO revisit this?

        // theoretically this should be better if the solver handles it correctly, but I'm not sure if I can
        // measure any improvement:
        options.linear().dual();
        return new ExpressionsBasedModel(options);
    }

    /**
     * Helper to set a timeout on an {@link Optimisation.Options} object.
     *
     * @param deadline the deadline time, which will be converted to a timeout.
     * @param opts     the options to be updated.
     * @return the same {@link Optimisation.Options} that was passed in.
     */
    public static Optimisation.Options setTimeout(long deadline, Optimisation.Options opts) {
        var duration = new CalendarDateDuration(deadline - System.currentTimeMillis(), MILLIS);
        return opts.abort(duration).suffice(duration);
    }

    /**
     * Helper to build a variable matrix. Used by {@link com.github.vrpjava.atsp.OjAlgoATSPSolver}
     *
     * @param costMatrix square matrix of distances
     * @param model      the model to add vars to.
     * @return a {@link Variable[][]} having the same dimension as <code>costMatrix</code>
     */
    public static Variable[][] buildAsymmetricVars(BigDecimal[][] costMatrix, ExpressionsBasedModel model) {
        var size = costMatrix.length;
        var vars = new Variable[size][size];

        for (var row = 0; row < size; row++) {
            var varsRow = vars[row];
            var costRow = costMatrix[row];

            for (var col = 0; col < size; col++) {
                if (row != col) {
                    varsRow[col] = model.newVariable("x" + row + "_" + col).binary().weight(costRow[col]);
                }
            }
        }
        return vars;
    }

    /**
     * Helper for ojAlgo hardware setup. This is hardcoded for my machine, but may also be suitable for
     * similar machines from Intel and AMD with caches equal or greater. Used from unit tests, but might
     * be useful from production code.
     */
    public static void setUpHardware() {
        System.setProperty("shut.up.ojAlgo", "");

        var systemThreads = VirtualMachine.getThreads();
        var ccxCount = ceilDiv(systemThreads, 16);

        // Overriding ojAlgo's hardware profile for my Zen 2. Assumes SMT and 12 or 16 threads per CCX.
        // ojAlgo appears to target 1 thread per core, not thread, for more effective L1 cache.
        var tmpL1Machine = new BasicMachine(32L * K, 2); //SMT
        var tmpL2Machine = new BasicMachine(512L * K, tmpL1Machine.threads);
        var tmpL3Machine = new BasicMachine(16L * K * K, systemThreads / ccxCount); // CCX
        var tmpSystemMachine = new BasicMachine(VirtualMachine.getMemory(), systemThreads);

        BasicMachine[] levels = {tmpSystemMachine, tmpL3Machine, tmpL2Machine, tmpL1Machine};
        OjAlgoUtils.ENVIRONMENT = new Hardware(VirtualMachine.getArchitecture(), levels).virtualise();
    }
}

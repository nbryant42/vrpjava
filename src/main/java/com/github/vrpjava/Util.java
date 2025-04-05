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

        // This is clearly much faster than the primal solver, but I wonder if ojAlgo is taking full advantage of
        // the dual variables after new constraints are added:
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
     * Helper for ojAlgo hardware setup. This is optimized for my old machine (a Ryzen 3900X), but may also be suitable
     * for similar machines from Intel and AMD with caches equal or greater. Used from unit tests, but might be useful
     * from production code.
     */
    @SuppressWarnings("unused")
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

    /**
     * Variant of {@link #setUpHardware()} hardcoded for my Core i7-14700KF.
     * <p>
     * This is a hybrid processor, so I'm going for the lowest common denominator of cache sizes between the
     * P-cores and E-cores, and faking it a little bit by telling ojAlgo that we have 20 cores & threads
     * (actually there are 20 cores and 28 threads, but this can't be divided by two.) I haven't really investigated
     * whether this is the best approach.
     */
    public static void setUpHardware_14700() {
        System.setProperty("shut.up.ojAlgo", "");

        var tmpL1Machine = new BasicMachine(32L * K, 1);
        var tmpL2Machine = new BasicMachine(4096L * K, 4);
        var tmpL3Machine = new BasicMachine(33L * K * K, 20); // CCX
        var tmpSystemMachine = new BasicMachine(VirtualMachine.getMemory(), 20);

        BasicMachine[] levels = {tmpSystemMachine, tmpL3Machine, tmpL2Machine, tmpL1Machine};
        OjAlgoUtils.ENVIRONMENT = new Hardware(VirtualMachine.getArchitecture(), levels).virtualise();
    }

    /**
     * Look up a distance between two nodes from the cost matrix. The cost matrix is lower-triangular
     * by convention, so this flips the row and column if necessary.
     *
     * @param i          the first node index
     * @param j          the second node index
     * @param costMatrix the travel distances as a lower-triangular matrix.
     */
    public static BigDecimal lookup(int i, int j, BigDecimal[][] costMatrix) {
        return i > j ? costMatrix[i][j] : costMatrix[j][i];
    }
}

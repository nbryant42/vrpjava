package org.example;

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

public class Util {
    private static final long K = 1024L;

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

    public static boolean isLowerTriangular(BigDecimal[][] matrix) {
        var size = matrix.length;

        return IntStream.range(0, size).noneMatch(row ->
                IntStream.range(row, size).anyMatch(col ->
                        matrix[row][col].signum() != 0));
    }

    public static ExpressionsBasedModel newModel(long deadline) {
        var options = setTimeout(deadline, new Optimisation.Options());
        //options.feasibility = NumberContext.of(14, 9);
        options.solution = NumberContext.of(14, 9); // TODO revisit this?

        // theoretically this should be better if the solver handles it correctly, but I'm not sure if I can
        // measure any improvement:
        options.linear().dual();
        return new ExpressionsBasedModel(options);
    }

    public static Optimisation.Options setTimeout(long deadline, Optimisation.Options opts) {
        var duration = new CalendarDateDuration(deadline - System.currentTimeMillis(), MILLIS);
        return opts.abort(duration).suffice(duration);
    }

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

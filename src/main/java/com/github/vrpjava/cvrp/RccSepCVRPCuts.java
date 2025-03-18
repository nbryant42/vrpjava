package com.github.vrpjava.cvrp;

import com.github.vrpjava.Util;
import com.github.vrpjava.cvrp.OjAlgoCVRPSolver.Cut;
import org.ojalgo.concurrent.Parallelism;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.Variable;
import org.ojalgo.optimisation.integer.IntegerStrategy;
import org.ojalgo.optimisation.integer.IntegerStrategy.ConfigurableStrategy;
import org.ojalgo.optimisation.integer.IntegerStrategy.GMICutConfiguration;
import org.ojalgo.optimisation.integer.ModelStrategy;
import org.ojalgo.type.context.NumberContext;

import java.lang.reflect.Constructor;
import java.math.BigDecimal;
import java.util.Comparator;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.IntSupplier;

import static com.github.vrpjava.cvrp.OjAlgoCVRPSolver.getVariable_noFlip;
import static java.math.BigDecimal.ONE;
import static java.math.BigDecimal.TWO;
import static java.math.BigDecimal.ZERO;
import static org.ojalgo.optimisation.integer.NodeKey.FIFO_SEQUENCE;
import static org.ojalgo.optimisation.integer.NodeKey.LARGE_DISPLACEMENT;
import static org.ojalgo.optimisation.integer.NodeKey.LIFO_SEQUENCE;
import static org.ojalgo.optimisation.integer.NodeKey.SMALL_DISPLACEMENT;

/**
 * <p>
 * An implementation of the Pavlikov-Petersen-Sørensen "RCC-Sep" algorithm.
 * Variable names are as used in (2.6) through (2.13) in section 2 of the paper.
 * </p><p>
 * This is an exact algorithm which finds the smallest cut-set for the biggest constraint violation, thus these
 * cuts tend to dominate other cuts and can minimize the total number of cuts added.
 * </p><p>
 * The cuts can be computed from a fractional relaxed solution in most cases, which speeds things up.
 * </p>
 *
 * @see <a href="https://onlinelibrary.wiley.com/doi/10.1002/net.22183">The paper.</a>
 */
class RccSepCVRPCuts {
    private RccSepCVRPCuts() {
    }

    private static final BigDecimal MINUS_ONE = ONE.negate();
    // tunable. 0.5 is used in the paper. we use this as an .upper(), so it's negated vs. the paper.
    private static final BigDecimal EPSILON1 = BigDecimal.valueOf(-0.5);

    /**
     * @return a set of generated cuts, or null when the subproblem cannot be solved to optimality
     * (which should only happen due to timeout)
     */
    static Set<Cut> generate(BigDecimal vehicleCapacity,
                             BigDecimal[] demands,
                             Optimisation.Result parentResult,
                             long deadline) {
        int size = demands.length;
        var candidates = new CutCandidates(size, parentResult);
        var result = buildSubProblem(vehicleCapacity, demands, parentResult, candidates, deadline).maximise();

        if (!result.getState().isOptimal()) {
            return null;
        }

        candidates.take(result);
        return candidates.getCuts();
    }

    /**
     * Build the ojAlgo model for the RCC-Sep sub-problem.
     * <p>
     * We use an algebraically simplified version of the objective function (2.6): it can be shown that the delta
     * weights are always -2, due to the distributive property and constraints on the parent problem.
     * After this, all terms would be multiplied by 2, so this also drops out.
     */
    private static ExpressionsBasedModel buildSubProblem(BigDecimal vehicleCapacity,
                                                         BigDecimal[] demands,
                                                         Optimisation.Result parentResult,
                                                         CutCandidates candidates,
                                                         long deadline) {
        var size = demands.length;
        var options = Util.setTimeout(deadline, new Optimisation.Options());
        //options.feasibility = NumberContext.of(14, 9);
        options.solution = NumberContext.of(14, 9); // TODO revisit this?
        options.integer(newConfigurable(candidates));

        var model = new ExpressionsBasedModel(options);

        // alpha is index 0 in the final, flat model vars array
        var alpha = model.newVariable("alpha").integer().lower(ONE).weight(ONE); // (2.12)

        // set up delta vars; these are indexes 1..n in the final, flat model vars array
        var delta = new Variable[size];
        for (int i = 1; i < size; i++) { // IntelliJ will collapse this to streams incorrectly, don't let it.
            delta[i] = model.newVariable("delta" + i).weight(MINUS_ONE).binary(); // (2.13)
        }

        // for each customer-customer edge {i, j} in C:
        // set up gamma vars from the final summation in objective (2.6).
        // note that gamma is lower-triangular.
        // also add constraints (2.9) and (2.10) (2.11 is handled as a lower(0))
        var gamma = new Variable[size][size];
        for (var i = 2; i < size; i++) {
            var gammaRow = gamma[i];

            for (var j = 1; j < i; j++) {
                setUpCustomerEdge(parentResult, model, i, j, gammaRow, delta);
            }
        }

        var c27 = model.newExpression("2.7").upper(EPSILON1).set(alpha, vehicleCapacity); // (2.7)
        for (int i = 1; i < size; i++) {
            c27.set(delta[i], demands[i].negate());
        }

        var c28 = model.newExpression("2.8").lower(TWO); // (2.8)
        for (int i = 1; i < size; i++) {
            c28.set(delta[i], ONE);
        }

        return model;
    }

    private static void setUpCustomerEdge(Optimisation.Result parentResult,
                                          ExpressionsBasedModel model,
                                          int i,
                                          int j,
                                          Variable[] gammaRow,
                                          Variable[] delta) {
        var v = getVariable_noFlip(i, j, parentResult);

        if (v.signum() != 0) {
            var thisGamma = model.newVariable("gamma" + i + "_" + j).lower(ZERO).weight(v);

            gammaRow[j] = thisGamma;

            model.newExpression("2.9_" + i + "_" + j).upper(ZERO).set(thisGamma, ONE).set(delta[i], MINUS_ONE);
            model.newExpression("2.10_" + i + "_" + j).upper(ZERO).set(thisGamma, ONE).set(delta[j], MINUS_ONE);
        }
    }

    private static final Constructor<ConfigurableStrategy> CTOR;

    static {
        try {
            CTOR = ConfigurableStrategy.class.getDeclaredConstructor(IntSupplier.class, Comparator[].class,
                    NumberContext.class, NumberContext.class, BiFunction.class, GMICutConfiguration.class);
            CTOR.setAccessible(true);
        } catch (NoSuchMethodException e) {
            throw new ExceptionInInitializerError(e);
        }
    }

    private static ConfigurableStrategy newConfigurable(CutCandidates candidates) {
        Comparator<?>[] definitions = {FIFO_SEQUENCE, SMALL_DISPLACEMENT, LIFO_SEQUENCE, LARGE_DISPLACEMENT};
        BiFunction<ExpressionsBasedModel, IntegerStrategy, ModelStrategy> factory =
                (model, strategy) -> new CallbackStrategy(model, strategy, candidates);

        try {
            return CTOR.newInstance(Parallelism.CORES.require(definitions.length), definitions, NumberContext.of(12, 8),
                    NumberContext.of(7, 8), factory, new GMICutConfiguration());
        } catch (ReflectiveOperationException e) {
            throw new Error(e);
        }
    }

}

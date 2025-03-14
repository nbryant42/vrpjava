package com.github.vrpjava.cvrp;

import org.ojalgo.function.multiary.MultiaryFunction;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.integer.IntegerStrategy;
import org.ojalgo.optimisation.integer.ModelStrategy;
import org.ojalgo.optimisation.integer.NodeKey;
import org.ojalgo.structure.Access1D;

import java.lang.reflect.Method;

public class CallbackStrategy extends ModelStrategy.AbstractStrategy {
    private final CutCandidates candidates;
    private final Method initialise;
    private final Method isCutRatherThanBranch;
    private final Method isDirect;
    private final Method markInfeasible;
    private final Method markInteger;
    private final Method toComparable;

    CallbackStrategy(final ExpressionsBasedModel model, final IntegerStrategy strategy, final CutCandidates candidates) {
        super(model, strategy);

        var c = delegate.getClass(); // forcefully call some protected methods in ojAlgo
        try {
            initialise = c.getDeclaredMethod("initialise", MultiaryFunction.TwiceDifferentiable.class, Access1D.class);
            isCutRatherThanBranch = c.getDeclaredMethod("isCutRatherThanBranch", double.class, boolean.class);
            isDirect = c.getDeclaredMethod("isDirect", NodeKey.class, boolean.class);
            markInfeasible = c.getDeclaredMethod("markInfeasible", NodeKey.class, boolean.class);
            markInteger = c.getDeclaredMethod("markInteger", NodeKey.class, Optimisation.Result.class);
            toComparable = c.getDeclaredMethod("toComparable", int.class, double.class, boolean.class);
        } catch (NoSuchMethodException e) {
            throw new Error(e);
        }

        this.candidates = candidates;
        initialise.setAccessible(true);
        isCutRatherThanBranch.setAccessible(true);
        isDirect.setAccessible(true);
        markInfeasible.setAccessible(true);
        markInteger.setAccessible(true);
        toComparable.setAccessible(true);
    }

    @Override
    protected ModelStrategy initialise(MultiaryFunction.TwiceDifferentiable<Double> function, Access1D<?> point) {
        try {
            initialise.invoke(delegate, function, point);
        } catch (ReflectiveOperationException e) {
            throw new Error(e);
        }

        return this;
    }

    @Override
    protected boolean isCutRatherThanBranch(double displacement, boolean found) {
        try {
            return (boolean) isCutRatherThanBranch.invoke(delegate, displacement, found);
        } catch (ReflectiveOperationException e) {
            throw new Error(e);
        }
    }

    @Override
    protected boolean isDirect(NodeKey node, boolean found) {
        try {
            return (boolean) isDirect.invoke(delegate, node, found);
        } catch (ReflectiveOperationException e) {
            throw new Error(e);
        }
    }

    @Override
    protected void markInfeasible(NodeKey key, boolean found) {
        try {
            markInfeasible.invoke(delegate, key, found);
        } catch (ReflectiveOperationException e) {
            throw new Error(e);
        }
    }

    @Override
    protected void markInteger(NodeKey key, Optimisation.Result result) {
        candidates.take(result);

        try {
            markInteger.invoke(delegate, key, result);
        } catch (ReflectiveOperationException e) {
            throw new Error(e);
        }
    }

    @Override
    protected double toComparable(int idx, double displacement, boolean found) {
        try {
            return (double) toComparable.invoke(delegate, idx, displacement, found);
        } catch (ReflectiveOperationException e) {
            throw new Error(e);
        }
    }
}

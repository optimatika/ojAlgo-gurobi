/*
 * Copyright 1997-2021 Optimatika
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package org.ojalgo.optimisation.solver.gurobi;

import static org.ojalgo.function.constant.PrimitiveMath.*;

import java.math.BigDecimal;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.ojalgo.array.Primitive64Array;
import org.ojalgo.optimisation.Expression;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.Variable;
import org.ojalgo.structure.Structure1D.IntIndex;
import org.ojalgo.structure.Structure2D.IntRowColumn;

import gurobi.GRB;
import gurobi.GRBEnv;
import gurobi.GRBException;
import gurobi.GRBExpr;
import gurobi.GRBLinExpr;
import gurobi.GRBModel;
import gurobi.GRBQuadExpr;
import gurobi.GRBVar;

public final class SolverGurobi implements Optimisation.Solver {

    @FunctionalInterface
    public interface Configurator {

        void configure(final GRBEnv environment, final GRBModel model, final Optimisation.Options options);

    }

    public static final class Integration extends ExpressionsBasedModel.Integration<SolverGurobi> implements AutoCloseable {

        private final GRBEnv myEnvironment;

        Integration() {
            this(null, null);
        }

        Integration(final String accessKey, final String secret) {
            super();
            GRBEnv tmpGRBEnv;
            try {
                if ((accessKey != null) && (secret != null)) {
                    tmpGRBEnv = new GRBEnv(null, accessKey, secret, null, 0);
                } else {
                    tmpGRBEnv = new GRBEnv();
                }
            } catch (final GRBException cause) {
                throw new RuntimeException(cause);
            }
            myEnvironment = tmpGRBEnv;
        }

        @Override
        public SolverGurobi build(final ExpressionsBasedModel model) {
            try {
                final GRBModel delegateSolver = new GRBModel(myEnvironment);
                final SolverGurobi retVal = new SolverGurobi(delegateSolver, model.options);

                final List<Variable> freeModVars = model.getFreeVariables();
                final Set<IntIndex> fixedModVars = model.getFixedVariables();

                final Expression modObj = model.objective().compensate(fixedModVars);

                final int numberOfVariables = freeModVars.size();

                final double[] lb = new double[numberOfVariables];
                final double[] ub = new double[numberOfVariables];
                final double[] obj = new double[numberOfVariables];
                final char[] type = new char[numberOfVariables];
                final String[] name = new String[numberOfVariables];

                for (int v = 0; v < numberOfVariables; v++) {
                    final Variable var = freeModVars.get(v);

                    lb[v] = var.getUnadjustedLowerLimit();
                    ub[v] = var.getUnadjustedUpperLimit();

                    final BigDecimal weight = var.getContributionWeight();
                    obj[v] = weight != null ? weight.doubleValue() : ZERO;

                    type[v] = GRB.CONTINUOUS;
                    if (var.isBinary()) {
                        type[v] = GRB.BINARY;
                    } else if (var.isInteger()) {
                        type[v] = GRB.INTEGER;
                    }

                    name[v] = var.getName();
                }

                delegateSolver.addVars(lb, ub, obj, type, name);
                delegateSolver.update();

                final GRBVar[] delegateVariables = delegateSolver.getVars();

                final List<Expression> tmpCollect = model.constraints().map(e -> e.compensate(fixedModVars)).collect(Collectors.toList());
                for (final Expression expr : tmpCollect) {

                    final GRBExpr solExpr = SolverGurobi.buildExpression(expr, model, delegateVariables);

                    SolverGurobi.setBounds(solExpr, expr, delegateSolver);
                }

                final GRBExpr solObj = SolverGurobi.buildExpression(modObj, model, delegateVariables);

                if (model.isMaximisation()) {
                    delegateSolver.setObjective(solObj, GRB.MAXIMIZE);
                } else {
                    delegateSolver.setObjective(solObj, GRB.MINIMIZE);
                }

                delegateSolver.update();

                return retVal;

            } catch (final GRBException exception) {
                exception.printStackTrace();
                return null;
            }

        }

        @Override
        public void close() throws Exception {
            if (myEnvironment != null) {
                myEnvironment.dispose();
            }
        }

        @Override
        public boolean isCapable(final ExpressionsBasedModel model) {
            return true;
        }

        @Override
        protected boolean isSolutionMapped() {
            return true;
        }

        GRBEnv getEnvironment() {
            return myEnvironment;
        }
    }

    public static final SolverGurobi.Integration INTEGRATION = new Integration();

    static final Configurator DEFAULT = (environment, model, options) -> {
        // TODO Auto-generated method stub
    };

    public static SolverGurobi.Integration newInstantCloudIntegration(final String accessKey, final String secret) {
        return new Integration(accessKey, secret);
    }

    static void addConstraint(final GRBModel model, final GRBExpr expr, final char sense, final double rhs, final String name) {
        try {
            if (expr instanceof GRBQuadExpr) {
                model.addQConstr((GRBQuadExpr) expr, sense, rhs, name);
            } else if (expr instanceof GRBLinExpr) {
                model.addConstr((GRBLinExpr) expr, sense, rhs, name);
            }
        } catch (final GRBException exception) {
            exception.printStackTrace();
        }
    }

    static GRBExpr buildExpression(final Expression expression, final ExpressionsBasedModel model, final GRBVar[] vars) throws GRBException {

        GRBLinExpr linExpr = null;
        GRBQuadExpr quadExpr;
        GRBExpr retVal = null;

        if (expression.isAnyLinearFactorNonZero()) {

            retVal = linExpr = new GRBLinExpr();

            for (final IntIndex key : expression.getLinearKeySet()) {

                final int freeInd = model.indexOfFreeVariable(key.index);
                if (freeInd >= 0) {
                    linExpr.addTerm(expression.getAdjustedLinearFactor(key), vars[freeInd]);
                }
            }
        }

        if (expression.isAnyQuadraticFactorNonZero()) {

            quadExpr = new GRBQuadExpr();
            if (linExpr != null) {
                quadExpr.add(linExpr);
            }
            retVal = quadExpr;

            for (final IntRowColumn key : expression.getQuadraticKeySet()) {

                final int freeRow = model.indexOfFreeVariable(key.row);
                final int freeCol = model.indexOfFreeVariable(key.column);
                if ((freeRow >= 0) && (freeCol >= 0)) {
                    quadExpr.addTerm(expression.getAdjustedQuadraticFactor(key), vars[freeRow], vars[freeCol]);
                }
            }
        }

        return retVal;
    }

    static void setBounds(final GRBExpr solExpr, final Expression modExpr, final GRBModel delegateSolver) {
        if (modExpr.isEqualityConstraint()) {
            SolverGurobi.addConstraint(delegateSolver, solExpr, GRB.EQUAL, modExpr.getAdjustedLowerLimit(), modExpr.getName());
        } else {
            if (modExpr.isLowerConstraint()) {
                SolverGurobi.addConstraint(delegateSolver, solExpr, GRB.GREATER_EQUAL, modExpr.getAdjustedLowerLimit(), modExpr.getName());
            }
            if (modExpr.isUpperConstraint()) {
                SolverGurobi.addConstraint(delegateSolver, solExpr, GRB.LESS_EQUAL, modExpr.getAdjustedUpperLimit(), modExpr.getName());
            }
        }
    }

    static State translate(final int status) {
        switch (status) {
        case GRB.Status.INFEASIBLE:
            return State.INFEASIBLE;
        case GRB.Status.ITERATION_LIMIT:
        case GRB.Status.CUTOFF:
        case GRB.Status.NODE_LIMIT:
        case GRB.Status.NUMERIC:
        case GRB.Status.SOLUTION_LIMIT:
        case GRB.Status.SUBOPTIMAL:
        case GRB.Status.TIME_LIMIT:
            return State.APPROXIMATE;
        case GRB.Status.INF_OR_UNBD:
            return State.INVALID;
        case GRB.Status.INPROGRESS:
        case GRB.Status.INTERRUPTED:
        case GRB.Status.LOADED:
            return State.UNEXPLORED;
        case GRB.Status.OPTIMAL:
            return State.OPTIMAL;
        case GRB.Status.UNBOUNDED:
            // Perhaps a problem here – ojAlgo expects a feasible solution with this state
            return State.UNBOUNDED;
        default:
            return State.FAILED;
        }
    }

    private final GRBModel myDelegateSolver;
    private final Options myOptions;

    SolverGurobi(final GRBModel model, final Options options) {
        super();
        myDelegateSolver = model;
        myOptions = options;
    }

    @Override
    public void dispose() {
        Solver.super.dispose();
        if (myDelegateSolver != null) {
            myDelegateSolver.dispose();
        }
    }

    @Override
    public Result solve(final Result kickStarter) {

        final GRBVar[] tmpVars = myDelegateSolver.getVars();

        State retState = State.UNEXPLORED;
        double retValue = NaN;
        Primitive64Array retSolution = Primitive64Array.make(myDelegateSolver.getVars().length);

        try {

            final GRBEnv tmpEnvironment = INTEGRATION.getEnvironment();

            DEFAULT.configure(tmpEnvironment, myDelegateSolver, myOptions);
            final Optional<Configurator> optional = myOptions.getConfigurator(Configurator.class);
            optional.ifPresent(configurator -> configurator.configure(tmpEnvironment, myDelegateSolver, myOptions));

            myDelegateSolver.getEnv().set(GRB.IntParam.OutputFlag, 0);

            myDelegateSolver.optimize();

            retState = SolverGurobi.translate(myDelegateSolver.get(GRB.IntAttr.Status));

            if (retState.isFeasible()) {

                retValue = myDelegateSolver.get(GRB.DoubleAttr.ObjVal);

                for (int i = 0; i < tmpVars.length; i++) {
                    retSolution.set(i, tmpVars[i].get(GRB.DoubleAttr.X));
                }
            }

        } catch (final GRBException cause) {
            throw new RuntimeException(cause);
        }

        return new Result(retState, retValue, retSolution);
    }

}

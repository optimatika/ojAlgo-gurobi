package org.ojalgo.optimisation.solver.gurobi;
/*
 * Copyright 1997-2020 Optimatika
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

import gurobi.GRB.*;
import gurobi.*;
import org.ojalgo.array.Primitive64Array;
import org.ojalgo.optimisation.Expression;
import org.ojalgo.optimisation.ExpressionsBasedModel;
import org.ojalgo.optimisation.Optimisation;
import org.ojalgo.optimisation.Variable;
import org.ojalgo.structure.Structure1D.IntIndex;
import org.ojalgo.structure.Structure2D.IntRowColumn;

import java.math.BigDecimal;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import static gurobi.GRB.*;
import static org.ojalgo.function.constant.PrimitiveMath.NaN;
import static org.ojalgo.function.constant.PrimitiveMath.ZERO;

@SuppressWarnings("restriction")
public final class SolverGurobi implements Optimisation.Solver {

    private final GRBModel myDelegateSolver;
    private final Options myOptions;

    @FunctionalInterface
    public interface Configurator {

        void configure(final GRBEnv environment, final GRBModel model, final Options options);

    }

    static final class Integration extends ExpressionsBasedModel.Integration<SolverGurobi> implements AutoCloseable {

        private final GRBEnv myEnvironment;

        Integration(final String accessKey, final String secret) {
            super();
            GRBEnv tmpGRBEnv;
            try {
                if (accessKey != null && secret != null) {
                    tmpGRBEnv = new GRBEnv(null, accessKey, secret, null, 0);
                } else {
                    tmpGRBEnv = new GRBEnv();
                }
            } catch (final GRBException anException) {
                tmpGRBEnv = null;
            }
            myEnvironment = tmpGRBEnv;
        }

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

                    type[v] = CONTINUOUS;
                    if (var.isBinary()) {
                        type[v] = BINARY;
                    } else if (var.isInteger()) {
                        type[v] = INTEGER;
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
                    delegateSolver.setObjective(solObj, MAXIMIZE);
                } else {
                    delegateSolver.setObjective(solObj, MINIMIZE);
                }

                delegateSolver.update();

                return retVal;

            } catch (final GRBException exception) {
                exception.printStackTrace();
                return null;
            }

        }

        public boolean isCapable(final ExpressionsBasedModel model) {
            return true;
        }


        @Override
        protected boolean isSolutionMapped() {
            return true;
        }

        final GRBEnv getEnvironment() {
            return myEnvironment;
        }

        @Override
        public void close() throws Exception {
            if (myEnvironment != null) {
                myEnvironment.dispose();
            }
        }
    }

    public static final Integration INTEGRATION = new Integration(null, null);

    public static Integration newInstantCloudIntegration(final String accessKey, final String secret) {
        return new Integration(accessKey, secret);
    }


    static final Configurator DEFAULT = (environment, model, options) -> {
        // TODO Auto-generated method stub
    };

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
            SolverGurobi.addConstraint(delegateSolver, solExpr, EQUAL, modExpr.getAdjustedLowerLimit(), modExpr.getName());
        } else {
            if (modExpr.isLowerConstraint()) {
                SolverGurobi.addConstraint(delegateSolver, solExpr, GREATER_EQUAL, modExpr.getAdjustedLowerLimit(), modExpr.getName());
            }
            if (modExpr.isUpperConstraint()) {
                SolverGurobi.addConstraint(delegateSolver, solExpr, LESS_EQUAL, modExpr.getAdjustedUpperLimit(), modExpr.getName());
            }
        }
    }



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

    public Result solve(final Result kickStarter) {

        final GRBVar[] tmpVars = myDelegateSolver.getVars();

        State retState = State.UNEXPLORED;
        double retValue = NaN;
        final Primitive64Array retSolution = Primitive64Array.make(myDelegateSolver.getVars().length);

        try {

            final GRBEnv tmpEnvironment = INTEGRATION.getEnvironment();

            DEFAULT.configure(tmpEnvironment, myDelegateSolver, myOptions);
            final Optional<Configurator> optional = myOptions.getConfigurator(Configurator.class);
            optional.ifPresent(configurator -> configurator.configure(tmpEnvironment, myDelegateSolver, myOptions));

            myDelegateSolver.getEnv().set(IntParam.OutputFlag, 0);

            myDelegateSolver.optimize();

            retState = this.translate(myDelegateSolver.get(IntAttr.Status));

            if (retState.isFeasible()) {

                retValue = myDelegateSolver.get(DoubleAttr.ObjVal);

                for (int i = 0; i < tmpVars.length; i++) {
                    retSolution.set(i, tmpVars[i].get(DoubleAttr.X));
                }
            }

        } catch (final GRBException exception) {
            exception.printStackTrace();
        }

        return new Result(retState, retValue, retSolution);
    }

    State translate(final int status) {
        switch (status) {
            case Status.INFEASIBLE:
                return State.INFEASIBLE;
            case Status.ITERATION_LIMIT:
            case Status.CUTOFF:
            case Status.NODE_LIMIT:
            case Status.NUMERIC:
            case Status.SOLUTION_LIMIT:
            case Status.SUBOPTIMAL:
            case Status.TIME_LIMIT:
                return State.APPROXIMATE;
            case Status.INF_OR_UNBD:
                return State.INVALID;
            case Status.INPROGRESS:
            case Status.INTERRUPTED:
            case Status.LOADED:
                return State.UNEXPLORED;
            case Status.OPTIMAL:
                return State.OPTIMAL;
            case Status.UNBOUNDED:
                return State.UNBOUNDED;
            default:
                return State.FAILED;
        }

    }

}
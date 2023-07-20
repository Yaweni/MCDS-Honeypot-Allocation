
#include <numeric>
#include <algorithm>
#include "ConstraintGenerators.h"

LBConstraintGenerator::LBConstraintGenerator(const LBValueFunction& lb, IloModel& model, IloNumVar& var, IloExpr& sum,
    IloExprArray& chi) : lb(lb), model(model), var(var), sum(sum), chi(chi) {
    for (auto&& component : lb.components) {
        IloExpr expr(model.getEnv());
        for (int i = 0; i < lb.dim; i++) {
            expr += component->a[i] * chi[i];
        }
        model.add(expr + component->b * sum - var <= 0);

        break;
    }
}

void LBConstraintGenerator::prepare_update(IloCplex& cplex, double& slack, double dual) {
    double varValue = cplex.getValue(var);
    double sumValue = cplex.getValue(sum);
    std::vector<double> chiValues(chi.getSize());
    for (int i = 0; i < chiValues.size(); i++) {
        chiValues[i] = cplex.getValue(chi[i]);
    }

    currentUpdateValue = -std::numeric_limits<double>::infinity();
    for (auto&& component : lb.components) {
        double acc = 0.0;
        for (int i = 0; i < lb.dim; i++) {
            acc += component->a[i] * chiValues[i];
        }
        acc += component->b * sumValue;

        double error = acc - varValue;
        if (error > currentUpdateValue) {
            currentUpdateValue = error;
            currentUpdate = component.get();
        }
    }

    slack -= currentUpdateValue;
}

bool LBConstraintGenerator::apply_update(IloCplex& cplex, double& slack) {
    if (slack < -1e-4) {
        auto& component = *currentUpdate;
        IloExpr expr(model.getEnv());
        for (int i = 0; i < lb.dim; i++) {
            expr += component.a[i] * chi[i];
        }
        model.add(expr + component.b * sum - var <= 0);

        return true;
    }
    else return false;
}

UBConstraintGenerator2::UBConstraintGenerator2(const UBValueFunction& ub, IloModel& model, IloNumVar& var, IloExpr& sum,
    IloExprArray& chi) : ub(ub), model(model), var(var), sum(sum), chi(chi) {
    model.add(var >= 0);

    //    IloEnv oracleEnv;
    //    IloModel oracleModel(oracleEnv);
    //    IloRangeArray dimRanges(oracleEnv, ub.dim);
    //    IloConstraint betaSum;
    //
    //    IloNumVarArray beta(oracleEnv, ub.components.size(), 0.0, CPX_INFBOUND);
    //    oracleModel.add(beta);
    //
    //    IloNumVar V(oracleEnv, -CPX_INFBOUND, CPX_INFBOUND);
    //
    //    betaSum = (IloSum(beta) == sum);
    //    model.add(betaSum);
    //
    //    for(int i = 0 ; i < ub.dim ; i++) {
    //        IloExpr dimExpr(model.getEnv());
    //
    //        int cid = 0;
    //        for(auto && component : components) {
    //            dimExpr += component.coords[i] * beta[cid++];
    //        }
    //
    //        dimRanges[i] = (dimExpr <= chi[i]);
    //    }
    //    model.add(dimRanges);
    //
    //    IloExpr vexpr(model.getEnv());
    //    int cid = 0;
    //    for(auto && component : components) {
    //        vexpr += component.v * beta[cid++];
    //    }
    //    model.add(IloMinimize(env, vexpr));
    //
    //    IloCplex cplex(model);
    //    cplex.setOut(env.getNullStream());
    //    cplex.setWarning(env.getNullStream());
    //
    ////    cplex.exportModel("/tmp/ub.lp");
    //
    //    cplex.solve();
    //
    //    double b = cplex.getDual(betaSum);
    //    std::vector<double> a(dim);
    //    for(int i = 0 ; i < dim ; i++) {
    //        a[i] = cplex.getDual(dimRanges[i]);
    //    }
    //
    //    cplex.end();
    //    env.end();
}

void UBConstraintGenerator2::prepare_update(IloCplex& cplex, double& slack, double dual) {
    double varValue = cplex.getValue(var);
    double sumValue = cplex.getValue(sum);
    std::vector<double> chiValues(chi.getSize());
    for (int i = 0; i < chiValues.size(); i++) {
        chiValues[i] = cplex.getValue(chi[i]);
    }

    currentUpdate.reset(new LBValueFunctionComponent(ub.getFacet(chiValues, sumValue)));
    double acc = currentUpdate->b * sumValue;
    for (int i = 0; i < ub.dim; i++) {
        acc += currentUpdate->a[i] * chiValues[i];
    }

    slack -= (acc - varValue);
}

bool UBConstraintGenerator2::apply_update(IloCplex& cplex, double& slack) {
    if (slack < -1e-4) {
        auto& component = *currentUpdate;
        IloExpr expr(model.getEnv());
        for (int i = 0; i < ub.dim; i++) {
            expr += component.a[i] * chi[i];
        }
        model.add(expr + component.b * sum - var <= 0);

        return true;
    }
    else return false;
}

UBConstraintGenerator::UBConstraintGenerator(const UBValueFunction& ub, IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi) : ub(ub), model(model), A(model.getEnv(), ub.dim) {
    IloNumVar beta1(model.getEnv(), 0.0, CPX_INFBOUND);
    model.add(beta1);

    auto& component = ub.components.front();

    betaSum = (beta1 - sum == 0);
    for (int i = 0; i < ub.dim; i++) {
        A[i] = (component.coords[i] * beta1 - chi[i] <= 0);
    }
    vSum = (component.v * beta1 - var == 0);

    model.add(betaSum);
    model.add(A);
    model.add(vSum);
    //
    //    int NPOINTS = ub.components.size() < 15 ? ub.components.size() : 15;
    //    IloNumVarArray beta(model.getEnv(), NPOINTS, 0.0, CPX_INFBOUND);
    //    model.add(beta);
    //
    //    betaSum = IloSum(beta) - sum == 0;
    //
    //    std::vector<int> idx(ub.components.size());
    //    std::iota(idx.begin(), idx.end(), 0);
    //    std::random_shuffle(idx.begin()+1, idx.end());
    //
    //    for(int i = 0 ; i < ub.dim ; i++) {
    //        IloExpr dimExpr(model.getEnv());
    //        for(int j = 0 ; j < NPOINTS ; j++) {
    //            auto && component = ub.components[idx[j]];
    //            dimExpr += component.coords[i] * beta[j];
    //        }
    //
    //        A[i] = dimExpr - chi[i] <= 0;
    //    }
    //
    //    IloExpr vexpr(model.getEnv());
    //    for(int j = 0 ; j < NPOINTS ; j++) {
    //        auto && component = ub.components[idx[j]];
    //        vexpr += component.v * beta[j];
    //    }
    //    vSum = vexpr - var == 0;
    //
    //    model.add(betaSum);
    //    model.add(A);
    //    model.add(vSum);
}

void UBConstraintGenerator::prepare_update(IloCplex& cplex, double& slack, double dual) {
    currentUpdateValue = -std::numeric_limits<double>::infinity();

    if (dual > -1e-4) return;

    double betaSumValue = cplex.getDual(betaSum);
    double vSumValue = cplex.getDual(vSum);
    std::vector<double> Avalues(ub.dim);
    for (int i = 0; i < ub.dim; i++) {
        Avalues[i] = cplex.getDual(A[i]);
    }

    for (auto&& component : ub.components) {
        double acc = betaSumValue + vSumValue * component.v;
        for (int i = 0; i < ub.dim; i++) {
            acc += Avalues[i] * component.coords[i];
        }

        if (acc > currentUpdateValue) {
            currentUpdateValue = acc;
            currentUpdate = &component;
        }
    }

}

bool UBConstraintGenerator::apply_update(IloCplex& cplex, double& slack) {
    if (currentUpdateValue > 1e-6) {
        IloNumVar beta(model.getEnv(), 0.0, CPX_INFBOUND);
        betaSum.setLinearCoef(beta, 1.0);
        vSum.setLinearCoef(beta, currentUpdate->v);
        for (int i = 0; i < ub.dim; i++) {
            A[i].setLinearCoef(beta, currentUpdate->coords[i]);
        }

        return true;
    }
    else return false;
}

LBConstraintGeneratorFactory::LBConstraintGeneratorFactory(const LBValueFunction& lb) : lb(lb) {}

UBConstraintGeneratorFactory::UBConstraintGeneratorFactory(const UBValueFunction& ub) : ub(ub) {}

UBConstraintGeneratorFactory2::UBConstraintGeneratorFactory2(const UBValueFunction& ub) : ub(ub) {}

LBConstraintGenerator
LBConstraintGeneratorFactory::create(IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi) const {
    return LBConstraintGenerator(lb, model, var, sum, chi);
}

UBConstraintGenerator
UBConstraintGeneratorFactory::create(IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi) const {
    return UBConstraintGenerator(ub, model, var, sum, chi);
}

UBConstraintGenerator2
UBConstraintGeneratorFactory2::create(IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi) const {
    return UBConstraintGenerator2(ub, model, var, sum, chi);
}

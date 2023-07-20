
#include <algorithm>
#include "ValueFunction.h"
#include "fast_profile.h"

void LBValueFunction::buildConstraints(IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi) const {
    for (auto&& component : components) {
        IloExpr expr(model.getEnv());
        for (int i = 0; i < dim; i++) {
            expr += component->a[i] * chi[i];
        }
        model.add(expr + component->b * sum - var <= 0);
    }
}

LBValueFunction::LBValueFunction(int dim) : dim(dim) {}

void UBValueFunction::buildConstraints(IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi) const {
    PROFILED("UBValueFunction::buildConstraints(...)")

        PROFILE("create vars", IloNumVarArray beta(model.getEnv(), components.size(), 0.0, 1.0);)
        model.add(beta);

    model.add(IloSum(beta) - sum == 0);

    for (int i = 0; i < dim; i++) {
        IloExpr dimExpr(model.getEnv());

        int cid = 0;
        for (auto&& component : components) {
            dimExpr += component.coords[i] * beta[cid++];
        }

        model.add(dimExpr - chi[i] <= 0);
    }

    IloExpr vexpr(model.getEnv());
    int cid = 0;
    for (auto&& component : components) {
        vexpr += component.v * beta[cid++];
    }
    model.add(vexpr - var == 0);
}

UBValueFunction::UBValueFunction(int dim) : dim(dim), model(env), cplex(model), maximum(std::numeric_limits<double>::infinity()) {
    cplex.setOut(env.getNullStream());
    cplex.setWarning(env.getNullStream());
    cplex.setParam(IloCplex::RootAlg, 1);

    model.add(V = IloNumVar(env, -IloInfinity, IloInfinity));
    model.add(V_constr = (-V == 0));

    model.add(beta_sum = (IloExpr(env) == 1));

    chis = IloRangeArray(env, dim);
    for (int i = 0; i < dim; i++) {
        chis[i] = (IloExpr(env) <= 0);
    }
    model.add(chis);

    model.add(IloMinimize(env, V));
}

UBValueFunction::~UBValueFunction() {
    env.end();
}

UBValueFunctionComponent::UBValueFunctionComponent(const std::vector<double>& coords, double v) : coords(coords),
v(v) {}

LBValueFunctionComponent::LBValueFunctionComponent(const std::vector<double>& a, double b) : a(a), b(b) {}

bool LBValueFunctionComponent::dominates(const LBValueFunctionComponent& other) const {
    double acc = b - other.b;
    for (int i = 0; i < a.size(); i++) {
        double diff = a[i] - other.a[i];
        if (diff < 0) acc += diff;
    }

    return acc >= 1e-6;
}

double LBValueFunctionComponent::get_value(const std::vector<double>& chi) const {
    double acc = b;
    for (int i = 0; i < chi.size(); i++) {
        acc += chi[i] * a[i];
    }
    return acc;
}

void LBValueFunction::add(const LBValueFunctionComponent& component) {
    components.emplace_back(new LBValueFunctionComponent(component));

    for (int i = 0; i < components.size(); i++) {
        if (component.dominates(*components[i])) {
            components[i] = std::move(components.back());
            components.pop_back();

            std::cout << "." << std::endl;
        }
    }
}

void UBValueFunction::add(const UBValueFunctionComponent& component) {
    components.push_back(component);

    IloNumVar beta(env, 0.0, IloInfinity);
    model.add(beta);
    beta_vars.push_back(beta);

    beta_sum.setLinearCoef(beta, 1.0);
    V_constr.setLinearCoef(beta, component.v);
    for (int i = 0; i < dim; i++) {
        chis[i].setLinearCoef(beta, component.coords[i]);
    }

    for (int i = 0; i < components.size(); i++) {
        if (getValue(components[i].coords) < components[i].v - 1e-6) {
            components[i] = std::move(components.back());
            components.pop_back();

            model.remove(beta_vars[i]);
            beta_vars[i] = beta_vars.back();
            beta_vars.pop_back();

            i--;
        }
    }

    if (std::all_of(component.coords.begin(), component.coords.end(), [](double d) { return d < 1e-6; })) {
        maximum = std::min(maximum, component.v);
    }
}


double LBValueFunction::getValue(const std::vector<double>& chi) const {
    PROFILED("LBValueFunction::getValue(...)")
        double opt = -std::numeric_limits<double>::infinity();
    for (auto&& component : components) {
        double scalprod = 0.0;
        for (int i = 0; i < dim; i++) {
            scalprod += chi[i] * component->a[i];
        }
        opt = std::max(opt, scalprod + component->b);
    }
    return opt;
}

double UBValueFunction::getValue(const std::vector<double>& _chi) const {
    PROFILED("UBValueFunction::getValue(...)")

        auto chi = _chi;
    for (double& v : chi) v = std::min(1.0, std::max(0.0, v));

    beta_sum.setBounds(1.0, 1.0);
    for (int i = 0; i < dim; i++) {
        chis[i].setUB(chi[i]);
    }

    cplex.solve();
    return cplex.getObjValue();
}

LBValueFunctionComponent UBValueFunction::getFacet(std::vector<double>& _chi, double sum) const {
    PROFILED("UBValueFunction::getFacet(...)")

        auto chi = _chi;
    for (double& v : chi) v = std::min(1.0, std::max(0.0, v));

    beta_sum.setBounds(sum, sum);
    for (int i = 0; i < dim; i++) {
        chis[i].setUB(chi[i]);
    }

    cplex.solve();

    double b = cplex.getDual(beta_sum);
    std::vector<double> a(dim);
    for (int i = 0; i < dim; i++) {
        a[i] = cplex.getDual(chis[i]);
    }

    return LBValueFunctionComponent(a, b);
}

double UBValueFunction::getValueApproximate(const std::vector<double>& chi) const {
    double best = std::numeric_limits<double>::infinity();
    for (auto& point : components) {
        double q = 1.0;
        for (int i = 0; i < dim; i++) {
            q = std::min(q, chi[i] / point.coords[i]);
        }
        best = std::min(best, q * point.v + (1 - q) * maximum);
    }

    return best;
}

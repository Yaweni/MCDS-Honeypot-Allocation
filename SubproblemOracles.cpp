
#include "SubproblemOracles.h"
#include "OracledStageGameLP.h"

#include <cmath>

SubproblemOracle::~SubproblemOracle() {}
SubproblemOracleFactory::~SubproblemOracleFactory() {}

bool LBSubproblemOracle::prepare() {
    auto env = cplex.getEnv();
    auto model = cplex.getModel();

    double Vh_value = cplex.getValue(subproblem.Vh);

    IloNumArray chi_values(env, subproblem.chi_primes.getSize());
    cplex.getValues(chi_values, subproblem.chi_primes);
    double prob = cplex.getValue(subproblem.prob);

    double best_value = 1e-6;
    best = nullptr;
    for (auto&& component : lb.components) {
        double acc = 0.0;
        for (int i = 0; i < lb.dim; i++) {
            acc += component->a[i] * chi_values[i];
        }
        acc += component->b * prob;

        double error = acc - Vh_value;
        if (error > best_value) {
            best_value = error;
            best = component.get();
        }
    }

    return best != nullptr;
}

LBSubproblemOracle::~LBSubproblemOracle() {}

LBSubproblemOracle::LBSubproblemOracle(const LBValueFunction& lb, const HoneypotAllocationSubproblem& subproblem,
    const IloCplex& cplex) : lb(lb), subproblem(subproblem), cplex(cplex) {
    for (auto&& component : lb.components) {
        add_component(*component);
        break;
    }
}

void LBSubproblemOracle::apply() {
    if (best != nullptr) {
        add_component(*best);
    }
}

void LBSubproblemOracle::add_component(const LBValueFunctionComponent& component) {
    auto model = cplex.getModel();
    IloExpr expr(model.getEnv());
    for (int i = 0; i < lb.dim; i++) {
        expr += component.a[i] * subproblem.chi_primes[i];
    }

    IloRange range = (expr + component.b * subproblem.prob - subproblem.Vh <= 0);
    model.add(range);

    ranges.push_back(range);
    components.push_back(component);
}

LBValueFunctionComponent LBSubproblemOracle::extract_gadget() {
    int chi_dim = subproblem.chi_primes.getSize();
    LBValueFunctionComponent out(std::vector<double>(chi_dim, 0.0), 0.0);
    double dual_sum = 0.0;
    for (int i = 0; i < ranges.size(); i++) {
        double dual = -cplex.getDual(ranges[i]);
        if (dual < 1e-6) continue;

        dual_sum += dual;
        for (int j = 0; j < chi_dim; j++) {
            out.a[j] += dual * components[i].a[j];
        }
        out.b += dual * components[i].b;
    }

    for (double& d : out.a) d /= dual_sum;
    out.b /= dual_sum;

    return out;
}

void LBSubproblemOracle::gadgetify() {
    auto gadget = extract_gadget();
    bool invalid = std::isnan(gadget.b);

    auto model = cplex.getModel();

    if (!invalid) {
        IloExpr expr(model.getEnv());
        for (int i = 0; i < lb.dim; i++) {
            expr += gadget.a[i] * subproblem.chi_primes[i];
        }
        model.add(expr + gadget.b * subproblem.prob - subproblem.Vh <= 0);
    }
    else {
        model.add(-subproblem.Vh <= 0);
    }

    for (auto&& range : ranges) {
        model.remove(range);
    }

    ranges.clear();
    components.clear();

    cplex.solve();
}

void LBSubproblemOracle::generate_all() {
    bool first = true;
    for (auto&& component : lb.components) {
        if (first) {
            first = false;
            continue;
        }

        add_component(*component);
    }
}

void LBSubproblemOracle::point_added(const UBValueFunctionComponent& point) {
    // do nothing
}

void LBSubproblemOracle::alpha_added(const UBValueFunctionComponent& alpha) {
    // do nothing
}

void LBSubproblemOracle::reset() {
    auto model = cplex.getModel();
    for (int i = 1; i < ranges.size(); i++) {
        model.remove(ranges[i]);
    }
    ranges.resize(1);
    components.resize(1);
}






UBSubproblemOracle::UBSubproblemOracle(const UBValueFunction& ub, const HoneypotAllocationSubproblem& subproblem,
    const IloCplex& cplex) : ub(ub), subproblem(subproblem), cplex(cplex) {
    auto model = cplex.getModel();
    model.add(subproblem.Vh >= 0);
}

bool UBSubproblemOracle::prepare() {
    double varValue = cplex.getValue(subproblem.Vh);
    double sumValue = cplex.getValue(subproblem.prob);
    std::vector<double> chiValues(subproblem.chi_primes.getSize());
    for (int i = 0; i < chiValues.size(); i++) {
        chiValues[i] = cplex.getValue(subproblem.chi_primes[i]);
    }

    currentUpdate.reset(new LBValueFunctionComponent(ub.getFacet(chiValues, sumValue)));
    double acc = currentUpdate->b * sumValue;
    for (int i = 0; i < ub.dim; i++) {
        acc += currentUpdate->a[i] * chiValues[i];
    }

    error = acc - varValue;
    return error > 1e-5;

    //    slack -= (acc - varValue);
}

UBSubproblemOracle::~UBSubproblemOracle() {}

void UBSubproblemOracle::apply() {
    if (error > 1e-5) {
        auto model = cplex.getModel();
        auto& component = *currentUpdate;
        IloExpr expr(model.getEnv());
        for (int i = 0; i < ub.dim; i++) {
            expr += component.a[i] * subproblem.chi_primes[i];
        }

        IloRange range = (expr + component.b * subproblem.prob - subproblem.Vh <= 0);
        model.add(range);
        ranges.push_back(range);

        components.push_back(std::move(component));
    }
}

LBValueFunctionComponent UBSubproblemOracle::extract_gadget() {
    int chi_dim = subproblem.chi_primes.getSize();
    LBValueFunctionComponent out(std::vector<double>(chi_dim, 0.0), 0.0);
    double dual_sum = 0.0;
    for (int i = 0; i < ranges.size(); i++) {
        double dual = -cplex.getDual(ranges[i]);
        if (dual < 1e-6) continue;

        dual_sum += dual;
        for (int j = 0; j < chi_dim; j++) {
            out.a[j] += dual * components[i].a[j];
        }
        out.b += dual * components[i].b;
    }

    for (double& d : out.a) d /= dual_sum;
    out.b /= dual_sum;

    return out;
}

void UBSubproblemOracle::gadgetify() {
    throw "Not implemented";
}

void UBSubproblemOracle::generate_all() {
    throw "Not implemented";
}

void UBSubproblemOracle::point_added(const UBValueFunctionComponent& point) {
    auto model = cplex.getModel();
    for (int i = 0; i < components.size(); i++) {
        if (components[i].get_value(point.coords) > point.v) {
            model.remove(ranges[i]);
            ranges[i] = ranges.back(); ranges.pop_back();
            components[i] = components.back(); components.pop_back();
        }
    }
}

void UBSubproblemOracle::alpha_added(const UBValueFunctionComponent& alpha) {
    // do nothing
}

void UBSubproblemOracle::reset() {
    auto model = cplex.getModel();
    for (auto& range : ranges) {
        model.remove(range);
    }
    ranges.clear();
    components.clear();
}

unique_ptr<SubproblemOracle>
LBSubproblemOracleFactory::create_oracle(IloCplex cplex, const HoneypotAllocationSubproblem& subproblem) {
    return std::unique_ptr<SubproblemOracle>(new LBSubproblemOracle(lb, subproblem, cplex));
}

LBSubproblemOracleFactory::LBSubproblemOracleFactory(const LBValueFunction& lb) : lb(lb) {}

UBSubproblemOracleFactory::UBSubproblemOracleFactory(const UBValueFunction& ub) : ub(ub) {}

unique_ptr<SubproblemOracle>
UBSubproblemOracleFactory::create_oracle(IloCplex cplex, const HoneypotAllocationSubproblem& subproblem) {
    return std::unique_ptr<SubproblemOracle>(new UBSubproblemOracle(ub, subproblem, cplex));
}

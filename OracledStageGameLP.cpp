
#include "OracledStageGameLP.h"

#include <algorithm>

void OracledStageGameLP::apply_interaction(AttackPlanIndices& attack_plan, HoneypotAllocationIndices& honeypot_allocation) {
    int first_hit = honeypot_allocation.allocation->size();
    double cost = 0;
    std::vector<bool> effects(game.numNodes - 1);

    bool hit = false;
    for (auto&& edge : attack_plan.plan.edges) {
        if (edge->dst->id > 0) effects[edge->dst->id - 1] = true;

        auto edgeIt = std::find(honeypot_allocation.allocation->begin(), honeypot_allocation.allocation->end(), edge);
        if (edgeIt == honeypot_allocation.allocation->end()) {
            cost += edge->normalCost;
        }
        else {
            cost += edge->honeypotCost;
            first_hit = edgeIt - honeypot_allocation.allocation->begin();
            break;
        }
    }

    honeypot_allocation.V_constraint.setLinearCoef(attack_plan.sigma, cost);

    if (effects.back()) return;

    auto& subproblem = honeypot_allocation.subproblems[first_hit];
    for (int i = 0; i < game.numNodes - 1; i++) {
        subproblem.chi_prime_constraints[i].setLinearCoef(effects[i] ? attack_plan.sigma : attack_plan.xis[i], 1.0);
    }
    subproblem.prob_constraint.setLinearCoef(attack_plan.sigma, 1.0);
}

OracledStageGameLP::OracledStageGameLP(const GameGraph& game, const std::vector<double>& chi, const std::unique_ptr<SubproblemOracleFactory>& oracle_factory)
    : game(game), oracle_factory(oracle_factory), env(), model(env), cplex(model), chi(chi) {

    cplex.setParam(IloCplex::RootAlg, oracle_factory->get_root_alg());
    cplex.setOut(env.getNullStream());
    cplex.setWarning(env.getNullStream());

    V = IloNumVar(env, -IloInfinity, IloInfinity, ILOFLOAT, "V");

    sigma_sum = (IloExpr(env) == 1);
    chi_sums = IloRangeArray(env, game.numNodes - 1);

    for (int i = 0; i < game.numNodes - 1; i++) {
        chi_sums[i] = (IloExpr(env) == chi[i]);
    }

    model.add(V);
    model.add(sigma_sum);
    model.add(chi_sums);

    objective = IloMinimize(env, V);
    model.add(objective);
}

void OracledStageGameLP::add_attack_plan(const AttackPlan& plan) {
    AttackPlanIndices indices(plan);

    char name[32];
    sprintf(name, "sigma(pi%d)", attack_plans.size());

    indices.sigma = IloNumVar(env, 0.0, IloInfinity, ILOFLOAT, name);
    indices.xis = IloNumVarArray(env, game.numNodes - 1, 0, IloInfinity);

    for (int i = 0; i < indices.xis.getSize(); i++) {
        sprintf(name, "xi(pi%d,v%d)", attack_plans.size(), i + 1);
        indices.xis[i].setName(name);
    }

    indices.xi_constraints = IloRangeArray(env, game.numNodes - 1);

    std::vector<bool> preconditions(game.numNodes - 1);
    for (auto&& pre : plan.preconditions) {
        if (pre->id > 0) preconditions[pre->id - 1] = true;
    }
    for (int i = 0; i < game.numNodes - 1; i++) {
        IloExpr expr = indices.xis[i] - indices.sigma;
        indices.xi_constraints[i] = (preconditions[i] ? expr == 0 : expr <= 0);

        chi_sums[i].setLinearCoef(indices.xis[i], 1.0);
    }
    sigma_sum.setLinearCoef(indices.sigma, 1.0);

    for (auto&& allocation : honeypot_allocations) {
        apply_interaction(indices, allocation);
    }

    model.add(indices.sigma);
    model.add(indices.xis);
    model.add(indices.xi_constraints);

    attack_plans.push_back(std::move(indices));
}

void OracledStageGameLP::add_honeypot_allocation(const HoneypotAllocation& allocation) {
    honeypot_allocations.emplace_back(allocation);
    auto& indices = honeypot_allocations.back();

    char name[32];

    indices.subproblems.resize(allocation.size() + 1);

    IloExpr Vexpr(env);
    for (int i = 0; i <= allocation.size(); i++) {
        auto& subproblem = indices.subproblems[i];
        subproblem.Vh = IloNumVar(env, -IloInfinity, IloInfinity);

        sprintf(name, "Vh(pi%d,%d)", honeypot_allocations.size() - 1, i);
        subproblem.Vh.setName(name);

        subproblem.chi_primes = IloNumVarArray(env, game.numNodes - 1, 0.0, IloInfinity);
        for (int j = 0; j < game.numNodes - 1; j++) {
            sprintf(name, "chiPrime(pi%d,%d,v%d)", honeypot_allocations.size() - 1, i, j + 1);
            subproblem.chi_primes[j].setName(name);
        }

        subproblem.chi_prime_constraints = IloRangeArray(env, game.numNodes - 1);

        subproblem.prob = IloNumVar(env);
        subproblem.prob_constraint = (-subproblem.prob == 0);

        for (int j = 0; j < game.numNodes - 1; j++) {
            subproblem.chi_prime_constraints[j] = (-subproblem.chi_primes[j] == 0);
        }

        Vexpr += subproblem.Vh;
    }
    indices.V_constraint = (Vexpr - V <= 0);

    sprintf(name, "pi%d", honeypot_allocations.size() - 1);
    indices.V_constraint.setName(name);

    for (auto&& plan : attack_plans) {
        apply_interaction(plan, indices);
    }

    auto& hp_allocation = honeypot_allocations.back();
    for (int i = 0; i <= allocation.size(); i++) {
        hp_allocation.subproblems[i].oracle = oracle_factory->create_oracle(cplex, hp_allocation.subproblems[i]);
    }

    model.add(indices.V_constraint);
    for (auto&& subproblem : indices.subproblems) {
        model.add(subproblem.Vh);
        model.add(subproblem.chi_primes);
        model.add(subproblem.chi_prime_constraints);
        model.add(subproblem.prob);
        model.add(subproblem.prob_constraint);
    }
}

void OracledStageGameLP::solve() {
    bool modified = true;
    while (modified) {
        modified = false;
        cplex.solve();

        //        export_model("/tmp/oracled.lp");

        for (auto& allocation : honeypot_allocations) {
            for (auto& subproblem : allocation.subproblems) {
                modified = subproblem.oracle->prepare() || modified;
            }
        }

        if (!modified) break;

        for (auto& allocation : honeypot_allocations) {
            for (auto& subproblem : allocation.subproblems) {
                subproblem.oracle->apply();
            }
        }
    }
}

double OracledStageGameLP::get_value() {
    return cplex.getObjValue();
}

void OracledStageGameLP::export_model(const std::string& file) {
    cplex.exportModel(file.c_str());
}

OracledStageGameLP::~OracledStageGameLP() {
    cplex.end();
    env.end();
}

std::vector<hpa_prob> OracledStageGameLP::get_defender_strategy() {
    std::vector<hpa_prob> strategy;
    for (auto&& allocation : honeypot_allocations) {
        double prob = -cplex.getDual(allocation.V_constraint);
        if (prob > 1e-6) {
            strategy.emplace_back(*allocation.allocation, prob);
            auto& hpa = strategy.back();

            for (auto&& subproblem : allocation.subproblems) {
                hpa.gadgets.push_back(subproblem.oracle->extract_gadget());
            }
        }
    }

    return strategy;
}

std::vector<ap_prob> OracledStageGameLP::get_attacker_strategy() {
    std::vector<ap_prob> strategy;
    for (auto&& plan : attack_plans) {
        double prob = cplex.getValue(plan.sigma);
        if (prob > 1e-6) {
            strategy.emplace_back(plan.plan, prob);
            auto& s = strategy.back();

            IloNumArray xi(env);
            cplex.getValues(xi, plan.xis);

            s.xi.resize(xi.getSize());
            for (int i = 0; i < xi.getSize(); i++) {
                s.xi[i] = xi[i];
            }
        }
    }

    return strategy;
}

void OracledStageGameLP::gadgetify() {
    for (auto&& allocation : honeypot_allocations) {
        for (auto&& subproblem : allocation.subproblems) {
            subproblem.oracle->gadgetify();
        }
    }
}

void OracledStageGameLP::fix_defender_strategy() {
    auto def_strategy = get_defender_strategy();
    gadgetify();

    int i = 0;
    for (auto&& allocation : honeypot_allocations) {
        if (i >= def_strategy.size()) break;
        if (def_strategy[i].allocation != allocation.allocation) continue;

        IloNumVar zeta(env, -IloInfinity, IloInfinity);
        allocation.V_constraint.setLinearCoef(zeta, 1.0);

        objective.setLinearCoef(zeta, -def_strategy[i].probability);

        i++;
    }
}

LBValueFunctionComponent OracledStageGameLP::extract_gadget() {
    IloNumArray a_duals(env, game.numNodes - 1);
    cplex.getDuals(a_duals, chi_sums);
    double b_dual = cplex.getDual(sigma_sum);

    std::vector<double> a(game.numNodes - 1);
    for (int i = 0; i < game.numNodes - 1; i++) a[i] = a_duals[i];

    return LBValueFunctionComponent(a, b_dual);
}

void OracledStageGameLP::apply_gadget(const LBValueFunctionComponent& gadget) {
    IloNumVarArray a_gadget_vars(env, game.numNodes - 1, -IloInfinity, 0.0, ILOFLOAT);
    IloNumVar b_gadget_var(env, -IloInfinity, 0.0);

    model.add(a_gadget_vars);
    model.add(b_gadget_var);

    for (int i = 0; i < game.numNodes - 1; i++) {
        chi_sums[i].setLinearCoef(a_gadget_vars[i], 1.0);
        objective.setLinearCoef(a_gadget_vars[i], gadget.a[i]);
    }

    sigma_sum.setLinearCoef(b_gadget_var, 1.0);
    objective.setLinearCoef(b_gadget_var, gadget.b);
}

void OracledStageGameLP::point_added(const UBValueFunctionComponent& point) {
    for (auto& allocation : honeypot_allocations) {
        for (auto& subproblem : allocation.subproblems) {
            subproblem.oracle->point_added(point);
        }
    }
}

void OracledStageGameLP::alpha_added(const UBValueFunctionComponent& alpha) {
    for (auto& allocation : honeypot_allocations) {
        for (auto& subproblem : allocation.subproblems) {
            subproblem.oracle->alpha_added(alpha);
        }
    }
}

void OracledStageGameLP::reset_oracles() {
    for (auto& allocation : honeypot_allocations) {
        for (auto& subproblem : allocation.subproblems) {
            subproblem.oracle->reset();
        }
    }
}

const std::vector<double>& OracledStageGameLP::get_chi() const {
    return chi;
}

AttackPlanIndices::AttackPlanIndices(const AttackPlan& plan) : plan(plan) {}

HoneypotAllocationIndices::HoneypotAllocationIndices(const HoneypotAllocation& allocation) : allocation(&allocation) {}

hpa_prob::hpa_prob(const HoneypotAllocation& allocation, double probability) : allocation(&allocation),
probability(probability) {}

ap_prob::ap_prob(const AttackPlan& plan, double probability) : plan(plan), probability(probability) {}

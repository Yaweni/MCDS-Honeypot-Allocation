
#ifndef MARGINALIZATION_ORACLEDSTAGEGAMELP_H
#define MARGINALIZATION_ORACLEDSTAGEGAMELP_H

#define IL_STD
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

#include "GameGraph.h"
#include "Actions.h"
#include "SubproblemOracles.h"

class hpa_prob {
public:
    const HoneypotAllocation* allocation;
    double probability;
    std::vector<LBValueFunctionComponent> gadgets;

    hpa_prob(const HoneypotAllocation& allocation, double probability);
};

class ap_prob {
public:
    const AttackPlan& plan;
    double probability;
    std::vector<double> xi;

    ap_prob(const AttackPlan& plan, double probability);
};


class AttackPlanIndices {
public:
    const AttackPlan& plan;

    IloNumVar sigma;
    IloNumVarArray xis;

    IloRangeArray xi_constraints;

    AttackPlanIndices(const AttackPlan& plan);
};

class HoneypotAllocationSubproblem {
public:
    IloNumVar Vh;
    IloNumVarArray chi_primes;
    IloRangeArray chi_prime_constraints;

    IloNumVar prob;
    IloRange prob_constraint;

    std::unique_ptr<SubproblemOracle> oracle;
};

class HoneypotAllocationIndices {
public:
    const HoneypotAllocation* allocation;

    std::vector<HoneypotAllocationSubproblem> subproblems;
    IloRange V_constraint;

    HoneypotAllocationIndices(const HoneypotAllocation& allocation);

    HoneypotAllocationIndices(HoneypotAllocationIndices&&) = default;
    HoneypotAllocationIndices& operator=(HoneypotAllocationIndices&&) = default;
};

class OracledStageGameLP {
    const GameGraph& game;
    const std::unique_ptr<SubproblemOracleFactory>& oracle_factory;

    std::vector<double> chi;

    IloEnv env;
    IloModel model;
    IloCplex cplex;

    IloObjective objective;

    IloNumVar V;

    IloRange sigma_sum;
    IloRangeArray chi_sums;

    std::vector<AttackPlanIndices> attack_plans;
    std::vector<HoneypotAllocationIndices> honeypot_allocations;

    void apply_interaction(AttackPlanIndices& attack_plan, HoneypotAllocationIndices& honeypot_allocation);

public:
    OracledStageGameLP(const GameGraph& game, const std::vector<double>& chi, const std::unique_ptr<SubproblemOracleFactory>& oracle_factory);

    OracledStageGameLP(const OracledStageGameLP&) = delete;
    OracledStageGameLP& operator=(const OracledStageGameLP&) = delete;

    void add_attack_plan(const AttackPlan& plan);
    void add_honeypot_allocation(const HoneypotAllocation& allocation);

    void solve();

    double get_value();

    std::vector<hpa_prob> get_defender_strategy();
    std::vector<ap_prob> get_attacker_strategy();

    void gadgetify();
    void fix_defender_strategy();

    void export_model(const std::string& file);

    LBValueFunctionComponent extract_gadget();
    void apply_gadget(const LBValueFunctionComponent& gadget);

    void point_added(const UBValueFunctionComponent& point);
    void alpha_added(const UBValueFunctionComponent& alpha);
    void reset_oracles();

    const std::vector<double>& get_chi() const;

    virtual ~OracledStageGameLP();
};


#endif //MARGINALIZATION_ORACLEDSTAGEGAMELP_H
#pragma once

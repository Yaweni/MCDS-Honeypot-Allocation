
#ifndef MARGINALIZATION_HSVI_H
#define MARGINALIZATION_HSVI_H

#include "GameGraph.h"
#include "ValueFunction.h"
#include "OracledStageGameLP.h"

#include <list>
#include <functional>

class HSVI {
    const GameGraph& game;
    const std::vector<HoneypotAllocation>& hp_allocations;
    const std::vector<AttackPlan>& attack_plans;
    LBValueFunction& lb;
    UBValueFunction& ub;
    const int K;

    std::list<HoneypotAllocation> oracled_allocations;
    std::list<AttackPlan> oracled_plans;

    std::set<std::vector<const Edge*>> oracled_plans_set;
    std::set<std::vector<const Edge*>> oracled_allocations_set;

    void explore(const std::vector<double>& chi, double epsilon, const std::function<double()>& measure);
    void oracled_solve(OracledStageGameLP& lblp, OracledStageGameLP& ublp);

    template <class VF>
    bool run_oracle(OracledStageGameLP& oracled_lp, const VF& value_function, OracledStageGameLP& auxiliary_lp, int abr_root_alg);

    void add(OracledStageGameLP& lp1, OracledStageGameLP& lp2, const HoneypotAllocation& allocation);
    void add(OracledStageGameLP& lp1, OracledStageGameLP& lp2, const AttackPlan& plan);

    void update_bounds(OracledStageGameLP& lblp, OracledStageGameLP& ublp);

public:
    HSVI(const GameGraph& game, const std::vector<HoneypotAllocation>& hp_allocations,
        const std::vector<AttackPlan>& attack_plans, LBValueFunction& lb, UBValueFunction& ub, int K);

    void solve(const std::vector<double>& init_chi, double epsilon, const std::function<double()>& measure);

    void export_bounds(const std::string& file_name);
};

#endif //MARGINALIZATION_HSVI_H
#pragma once

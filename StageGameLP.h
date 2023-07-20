
#ifndef MARGINALIZATION_STAGEGAMELP_H
#define MARGINALIZATION_STAGEGAMELP_H


#include "GameGraph.h"
#include "Actions.h"
#include "ValueFunction.h"

class StageGameResult {
public:
    double value;

    std::vector<double> chiSlacks;

    std::vector<double> a;
    double b;

    std::vector<double> hProbs;

    double nextChiValue;
    std::vector<double> nextChi;
};

class StageGameLP {

    const GameGraph& game;
    int n = 0;

    std::vector<AttackPlan> attackPlans;
    std::vector<HoneypotAllocation> honeypotAllocations;

    std::vector<std::vector<double>> costMat;
    std::vector<std::vector<int>> triggered;

public:
    StageGameLP(const GameGraph& game);

    void addAttackPlan(const AttackPlan& plan);
    void addHoneypotAllocation(const HoneypotAllocation& allocation);

    const GameGraph& getGame() const;

    const std::vector<AttackPlan>& getAttackPlans() const;

    const std::vector<HoneypotAllocation>& getHoneypotAllocations() const;

    template <class Factory>
    StageGameResult solve(const std::vector<double>& chi, const Factory& factory, const LBValueFunction& lb, const UBValueFunction& ub, const std::vector<double>& hProbs);

};


#endif //MARGINALIZATION_STAGEGAMELP_H
#pragma once

#ifndef MARGINALIZATION_GLOBAL_BR_H
#define MARGINALIZATION_GLOBAL_BR_H

#include <vector>
#include "GameGraph.h"
#include "StageGameLP.h"
#include "SubproblemOracles.h"

double global_br(const GameGraph& game, const std::vector<HoneypotAllocation>& allocations, const std::vector<AttackPlan>& plans,
    const std::unique_ptr<SubproblemOracleFactory>& oracle_factory, const std::vector<double>& chi,
    const LBValueFunctionComponent& gadget, double best_so_far);

#endif //MARGINALIZATION_GLOBAL_BR_H
#pragma once

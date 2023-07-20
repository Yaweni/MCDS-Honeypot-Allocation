
#ifndef MARGINALIZATION_BESTRESPONSES_H
#define MARGINALIZATION_BESTRESPONSES_H

#include "Actions.h"
#include "OracledStageGameLP.h"

AttackPlan compute_BR(const GameGraph& g, const std::vector<hpa_prob>& defender_strategy, const Node& root);

#endif //MARGINALIZATION_BESTRESPONSES_H
#pragma once

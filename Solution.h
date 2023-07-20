
#ifndef STOCHASTICGAMESCPP_SOLUTION_H
#define STOCHASTICGAMESCPP_SOLUTION_H

#include <vector>
#include "GameGraph.h"

class StrategyEntry {
public:
    std::vector<const Edge*> configuration;
    double realizationProbability;

    StrategyEntry(const std::vector<const Edge*>& configuration, double realizationProbability);
};
typedef std::vector<StrategyEntry> strategy_t;

class Solution {
public:
    std::vector<double> valuation;
    std::vector<strategy_t> strategies;

    Solution(unsigned int numNodes);
};


#endif //STOCHASTICGAMESCPP_SOLUTION_H
#pragma once


#include "Solution.h"

Solution::Solution(unsigned int numNodes) : valuation(numNodes), strategies(numNodes) {}

StrategyEntry::StrategyEntry(const std::vector<const Edge*>& configuration, double realizationProbability)
    : configuration(configuration), realizationProbability(realizationProbability) {}

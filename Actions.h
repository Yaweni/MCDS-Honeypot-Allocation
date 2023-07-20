
#ifndef MARGINALIZATION_ATTACKPLAN_H
#define MARGINALIZATION_ATTACKPLAN_H

#include <vector>
#include <set>
#include "GameGraph.h"

using HoneypotAllocation = std::vector<const Edge*>;

class AttackPlan {
public:
    std::vector<const Edge*> edges;
    std::set<const Node*> preconditions;
    std::set<const Node*> effects;

public:
    AttackPlan();
    AttackPlan(std::initializer_list<const Edge*> edges);

    void add(const Edge* edge);
};

#endif //MARGINALIZATION_ATTACKPLAN_H

#pragma once





#include <iostream>
#include "Actions.h"

void AttackPlan::add(const Edge* edge) {
    if (!effects.count(edge->src)) {
        preconditions.insert(edge->src);
    }
    effects.insert(edge->dst);
    edges.push_back(edge);
}

AttackPlan::AttackPlan() {}

AttackPlan::AttackPlan(std::initializer_list<const Edge*> edges) {
    for (auto&& edge : edges) {
        add(edge);
    }
}

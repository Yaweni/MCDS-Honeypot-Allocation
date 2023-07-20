
#ifndef MARGINALIZATION_BEST_RESPONSES_H
#define MARGINALIZATION_BEST_RESPONSES_H

#define IL_STD
#include <ilcplex/ilocplex.h>
#include "GameGraph.h"
#include "OracledStageGameLP.h"
#include "fast_profile.h"
#include <algorithm>
#include <queue>

ILOSTLBEGIN

class DefenderBR {
public:
    double value;
    HoneypotAllocation allocation;

    DefenderBR(double value, const HoneypotAllocation& allocation);
};

//template <typename VF>
//DefenderBR defender_br_rec(const GameGraph & game, const std::vector<ap_prob> & attacker_strategy, const std::vector<bool> & honeypot, int K, VF & value_function);

template <typename VF>
DefenderBR defender_br_rec(const GameGraph& game, const std::vector<ap_prob>& attacker_strategy, const std::vector<bool>& honeypot, int K, VF& value_function) {
    PROFILED("defender_br_rec")
        std::vector<bool> best_allocation;
    double best = -std::numeric_limits<double>::infinity();

    for (int e = 0; e < game.numEdges; e++) {
        if (honeypot[e]) continue;

        std::vector<bool> current_hp = honeypot;
        current_hp[e] = true;

        std::vector<std::vector<double>> subgames(game.numEdges);
        std::vector<double> subgame_probs(game.numEdges);
        double total_cost = 0.0;
        for (auto&& path : attacker_strategy) {
            std::vector<bool> infected(game.numNodes - 1);
            double cost = 0.0;
            for (auto& edge : path.plan.edges) {
                if (edge->dst->id > 0) infected[edge->dst->id - 1] = true;

                if (current_hp[edge->id]) {
                    cost += edge->honeypotCost;

                    if (edge->dst != &game.nodes.back()) {
                        auto& subgame = subgames[edge->id];
                        if (subgame.size() == 0) subgame.resize(game.numNodes - 1, 0.0);

                        for (int i = 0; i < game.numNodes - 1; i++) {
                            if (infected[i]) subgame[i] += path.probability;
                            else subgame[i] += path.xi[i];
                        }
                        subgame_probs[edge->id] += path.probability;
                    }

                    break;
                }
                else {
                    cost += edge->normalCost;
                }
            }

            total_cost += path.probability * cost;
        }

        for (int i = 0; i < game.numEdges; i++) {
            auto& subgame = subgames[i];
            auto& sum = subgame_probs[i];

            if (subgame.size() == 0) continue;
            else {
                for (double& d : subgame) d /= sum;
                total_cost += sum * value_function.getValue(subgame);
            }
        }

        if (total_cost > best) {
            best = total_cost;
            best_allocation = std::move(current_hp);
        }
    }

    if (K == 1) {
        HoneypotAllocation allocation;
        for (int e = 0; e < game.numEdges; e++) {
            if (best_allocation[e]) allocation.push_back(game.edges[e].get());
        }

        return DefenderBR(best, allocation);
    }
    else return defender_br_rec(game, attacker_strategy, best_allocation, K - 1, value_function);
}

static int calls = 0;
static int expansions = 0;

template <typename VF>
DefenderBR defender_br_bnb(const GameGraph& game, const std::vector<ap_prob>& attacker_strategy, int K, VF& value_function) {
    PROFILED("defender_br_bnb")

        calls++;

    class search_node {
    public:
        double lb;
        double ub;
        int used_hps;
        std::vector<bool> allocation;

        search_node(const double lb, const double ub, const int used_hps, std::vector<bool>&& allocation)
            : lb(lb), ub(ub), used_hps(used_hps), allocation(std::move(allocation)) {}

        bool operator<(const search_node& other) const {
            return ub < other.ub;
        }
    };

    std::priority_queue<search_node> queue;

    auto add_node = [&game, &attacker_strategy, K, &value_function, &queue](std::vector<bool>&& allocation) {
        std::vector<std::vector<double>> chi_prime(game.numEdges);
        std::vector<double> subgame_probs(game.numEdges);

        double total_cost = 0.0;
        for (auto&& path : attacker_strategy) {
            std::vector<bool> infected(game.numNodes - 1);
            double cost = 0.0;
            for (auto& edge : path.plan.edges) {
                if (edge->dst->id > 0) infected[edge->dst->id - 1] = true;

                auto& subgame = chi_prime[edge->id];

                if (edge->dst != &game.nodes.back()) {
                    if (subgame.size() == 0) subgame.resize(game.numNodes - 1, 0.0);
                    for (int i = 0; i < game.numNodes - 1; i++) {
                        if (infected[i]) subgame[i] += path.probability;
                        else subgame[i] += path.xi[i];
                    }
                }
                subgame_probs[edge->id] += path.probability;

                if (allocation[edge->id]) {
                    cost += edge->honeypotCost;
                    break;
                }
                else {
                    cost += edge->normalCost;
                }
            }

            total_cost += path.probability * cost;
        }

        for (int i = 0; i < game.numEdges; i++) {
            auto& subgame = chi_prime[i];
            auto& sum = subgame_probs[i];

            if (subgame.empty() || !allocation[i]) continue;
            else {
                for (double& d : subgame) d /= sum;
                total_cost += sum * value_function.getValue(subgame);
            }
        }

        int remaining_hps = K;
        for (const auto& b : allocation) {
            if (b) remaining_hps -= 1;
        }

        std::vector<double> deltas;
        for (int e = 0; e < game.numEdges; e++) {
            if (allocation[e]) continue;

            auto& edge = *game.edges[e];
            auto& subgame = chi_prime[e];
            double delta = subgame_probs[e] * (edge.honeypotCost - edge.normalCost);

            if (!subgame.empty()) {
                for (double& d : subgame) d /= subgame_probs[e];
                delta += subgame_probs[e] * value_function.getValueApproximate(subgame);
            }

            deltas.push_back(delta);
        }

        std::sort(deltas.begin(), deltas.end(), std::greater<>());
        double total_delta = 0.0;
        for (int i = 0; i < remaining_hps; i++) total_delta += deltas[i];

        queue.emplace(total_cost, total_cost + total_delta, K - remaining_hps, std::move(allocation));
    };

    add_node(std::vector<bool>(game.numEdges));

    while (!queue.empty()) {
        expansions++;

        search_node current(std::move(const_cast<search_node&>(queue.top()))); queue.pop();

        if (current.used_hps == K) {
            HoneypotAllocation allocation;
            for (int e = 0; e < game.numEdges; e++) {
                if (current.allocation[e]) allocation.push_back(game.edges[e].get());
            }
            return DefenderBR(current.lb, allocation);
        }

        for (int e = 0; e < game.numEdges; e++) {
            if (!current.allocation[e]) {
                std::vector<bool> new_allocation = current.allocation;
                new_allocation[e] = true;
                add_node(std::move(new_allocation));
            }
        }
    }

    throw "This should not ever happen!";
}

class AttackerBR {
    IloEnv env;
    IloModel model;
    IloCplex cplex;

    const GameGraph& game;
    const std::vector<hpa_prob>& def_strategy;
    const LBValueFunctionComponent& expected;

    IloBoolVarArray path;
    IloNumVarArray delta;

    IloNumVarArray xi;

    IloNumVar mult(IloExpr x, IloExpr y);
    IloExpr add_allocation(const hpa_prob& strategy);

    int allocation_id;

public:
    AttackerBR(const GameGraph& game, const vector<hpa_prob>& def_strategy, const LBValueFunctionComponent& expected, int root_alg);
    virtual ~AttackerBR();

    void solve();
    double get_value();
    AttackPlan get_plan();

    void export_model(const std::string& file_name) const;
};

#endif //MARGINALIZATION_BEST_RESPONSES_H
#pragma once

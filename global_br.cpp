
#include <algorithm>

#include "global_br.h"
#include "OracledStageGameLP.h"

#define PRUNING_BR 1

#ifdef PRUNING_BR

double global_br(const GameGraph& game, const std::vector<HoneypotAllocation>& allocations, const std::vector<AttackPlan>& plans,
    const std::unique_ptr<SubproblemOracleFactory>& oracle_factory, const std::vector<double>& chi,
    const LBValueFunctionComponent& gadget, double best_so_far) {

    OracledStageGameLP lblp(game, chi, oracle_factory);
    for (auto&& plan : plans) lblp.add_attack_plan(plan);
    for (auto&& allocation : allocations) lblp.add_honeypot_allocation(allocation);

    lblp.apply_gadget(gadget);

    lblp.solve();
    //    lblp.export_model("/tmp/badass_gadget.lp");

        // If suboptimal, return immediately
    if (lblp.get_value() > best_so_far) return lblp.get_value();

    auto def_strategy = lblp.get_defender_strategy();
    auto att_strategy = lblp.get_attacker_strategy();

    std::vector<const AttackPlan*> attack_plans;
    for (auto& plan : att_strategy) {
        attack_plans.push_back(&plan.plan);
    }
    for (auto& plan : plans) {
        attack_plans.push_back(&plan);
    }

    double value = lblp.get_value();
    double acc = 0.0;

    for (auto&& _plan : attack_plans) {
        auto& plan = *_plan;
        auto& next_node = *plan.edges.front()->dst;
        double total_cost = 0.0;

        std::vector<double> lb_costs(def_strategy.size());
        std::vector<double> probs(def_strategy.size());

        int da = 0;

        for (auto&& v : plan.preconditions) {
            if (v->id > 0 && chi[v->id - 1] < 1e-6) goto next_plan;
        }

        if (next_node.id == game.numNodes - 1);
        else if (next_node.id == 0 || chi[next_node.id - 1] > 1e-6) goto next_plan;

        std::sort(def_strategy.begin(), def_strategy.end(),
            [](const hpa_prob& a, const hpa_prob& b) { return a.probability > b.probability; });

        for (auto&& def_action : def_strategy) {
            auto&& allocation = def_action.allocation;
            std::vector<double> next_chi = chi;
            double cost = 0.0;

            decltype(allocation->begin()) it;

            for (auto&& edge : plan.edges) {
                if (edge->dst->id > 0) next_chi[edge->dst->id - 1] = 1.0;

                it = std::find(allocation->begin(), allocation->end(), edge);

                if (it == allocation->end()) cost += edge->normalCost;
                else {
                    cost += edge->honeypotCost;
                    break;
                }
            }

            if (it != allocation->end()) {
                int gid = it - allocation->begin();
                auto& gadget = def_action.gadgets[gid];
                cost += gadget.b;
                for (int j = 0; j < next_chi.size(); j++) {
                    if (next_chi[j] > 1e-6) cost += gadget.a[j];
                }
            }

            probs[da] = def_action.probability;
            lb_costs[da++] = cost;
        }

        da = 0;
        for (auto&& def_action : def_strategy) {
            acc = 0.0;
            double rem = best_so_far;
            for (int j = 0; j < def_strategy.size(); j++) {
                acc += probs[j] * lb_costs[j];
                if (j != da) rem -= probs[j] * lb_costs[j];
            }
            if (acc > best_so_far) break;

            auto&& allocation = def_action.allocation;
            std::vector<double> next_chi = chi;
            double cost = 0.0;

            for (auto&& edge : plan.edges) {
                if (edge->dst->id > 0) next_chi[edge->dst->id - 1] = 1.0;

                auto it = std::find(allocation->begin(), allocation->end(), edge);

                if (it == allocation->end()) cost += edge->normalCost;
                else {
                    cost += edge->honeypotCost;
                    if (next_chi.back() < 1e-6) {
                        double subgame_cost = global_br(game, allocations, plans, oracle_factory, next_chi, def_action.gadgets[it - allocation->begin()], rem / def_action.probability - cost);
                        cost += subgame_cost;
                    }
                    break;
                }
            }

            lb_costs[da++] = cost;
        }

        acc = 0.0;
        for (int j = 0; j < def_strategy.size(); j++) {
            acc += probs[j] * lb_costs[j];
        }
        best_so_far = std::min(best_so_far, acc);

    next_plan:;
    }

    return best_so_far;

}

#else

double global_br(const StageGameLP& lp, const std::unique_ptr<SubproblemOracleFactory>& oracle_factory, const std::vector<double>& chi,
    const LBValueFunctionComponent& gadget, double best_so_far) {

    auto& game = lp.getGame();

    std::vector<hpa_prob> def_strategy;
    double value;

    {
        OracledStageGameLP lblp(game, chi, oracle_factory);
        for (auto&& plan : lp.getAttackPlans()) lblp.add_attack_plan(plan);
        for (auto&& allocation : lp.getHoneypotAllocations()) lblp.add_honeypot_allocation(allocation);

        lblp.apply_gadget(gadget);

        lblp.solve();
        def_strategy = lblp.get_defender_strategy();

        value = lblp.get_value();
    }

    double best = std::numeric_limits<double>::infinity();

    for (auto&& plan : lp.getAttackPlans()) {
        auto& next_node = *plan.edges.front()->dst;
        double total_cost = 0.0;

        for (auto&& v : plan.preconditions) {
            if (v->id > 0 && chi[v->id - 1] < 1e-6) goto next_plan;
        }

        if (next_node.id == game.numNodes - 1);
        else if (next_node.id == 0 || chi[next_node.id - 1] > 1e-6) goto next_plan;

        for (auto&& def_action : def_strategy) {
            auto&& allocation = def_action.allocation;
            std::vector<double> next_chi = chi;
            double cost = 0.0;

            for (auto&& edge : plan.edges) {
                if (edge->dst->id > 0) next_chi[edge->dst->id - 1] = 1.0;

                auto it = std::find(allocation.begin(), allocation.end(), edge);

                if (it == allocation.end()) cost += edge->normalCost;
                else {
                    cost += edge->honeypotCost;
                    if (next_chi.back() < 1e-6) cost += global_br(lp, oracle_factory, next_chi, def_action.gadgets[it - allocation.begin()], best_so_far);
                    break;
                }
            }

            total_cost += def_action.probability * cost;
        }

        best = std::min(best, total_cost);

    next_plan:;
    }

    return best;

}

#endif
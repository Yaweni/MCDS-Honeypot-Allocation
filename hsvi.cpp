
#include "hsvi.h"
#include "OracledStageGameLP.h"
#include "best_responses.h"

#include "cpptoml.h"

#ifndef DEF_ORACLE
// #error DEF_ERROR
#endif

#ifndef ATT_ORACLE
// #error ATT_ERROR
#endif

constexpr double ZERO = 1e-4;

HSVI::HSVI(const GameGraph& game, const std::vector<HoneypotAllocation>& hp_allocations,
    const std::vector<AttackPlan>& attack_plans, LBValueFunction& lb, UBValueFunction& ub, int K)
    : game(game), hp_allocations(hp_allocations), attack_plans(attack_plans), lb(lb), ub(ub), K(K) {

    std::vector<bool> origins(game.numNodes);
    origins.back() = true;

#if ATT_ORACLE > 0
    for (auto& plan : attack_plans) {
        int src_id = plan.edges.front()->src->id;
        if (!origins[src_id]) {
            oracled_plans.push_back(plan);
            oracled_plans_set.insert(plan.edges);
            origins[src_id] = true;
        }
    }
#else
    for (auto& plan : attack_plans) {
        oracled_plans.push_back(plan);
        oracled_plans_set.insert(plan.edges);
    }
#endif

#if DEF_ORACLE > 0
    oracled_allocations.push_back(hp_allocations.front());
    oracled_allocations_set.insert(hp_allocations.front());
#else
    for (auto& allocation : hp_allocations) {
        oracled_allocations.push_back(allocation);
        //TODO        oracled_allocations_set.insert(allocation);
    }
#endif

}

std::function<double()> glob_measure;

void HSVI::solve(const std::vector<double>& init_chi, double epsilon, const std::function<double()>& measure) {

    glob_measure = measure;

    int iteration = 0;

    while (true) {
        double lb_value = lb.getValue(init_chi);
        double ub_value = ub.getValue(init_chi);

        std::cout << measure() << " Iteration " << ++iteration
            << " [ " << lb_value << "\t..\t" << ub_value << " ]\t"
            << " lb_size=" << lb.components.size() << " ub_size=" << ub.components.size() << std::endl;

        if (ub_value - lb_value < epsilon) {
            break;
        }

        explore(init_chi, epsilon, measure);

        lb_value = lb.getValue(init_chi);
        ub_value = ub.getValue(init_chi);

        std::cout << measure() << " Iteration " << iteration
            << " [ " << lb_value << "\t..\t" << ub_value << " ]\t"
            << " lb_size=" << lb.components.size() << " ub_size=" << ub.components.size() << std::endl;
    }
}

void HSVI::explore(const std::vector<double>& chi, double epsilon, const std::function<double()>& measure) {

    std::cout << measure() << "\t";
    for (double d : chi) std::cout << d << " ";
    std::cout << std::endl;

    std::unique_ptr<SubproblemOracleFactory> lb_oracle(new LBSubproblemOracleFactory(lb));
    std::unique_ptr<SubproblemOracleFactory> ub_oracle(new UBSubproblemOracleFactory(ub));

    OracledStageGameLP lblp(game, chi, lb_oracle);
    OracledStageGameLP ublp(game, chi, ub_oracle);

    for (auto& allocation : oracled_allocations) {
        lblp.add_honeypot_allocation(allocation);
        ublp.add_honeypot_allocation(allocation);
    }
    for (auto& plan : oracled_plans) {
        lblp.add_attack_plan(plan);
        ublp.add_attack_plan(plan);
    }

    oracled_solve(lblp, ublp);

    auto def_strategy = ublp.get_defender_strategy();
    auto att_strategy = lblp.get_attacker_strategy();

    update_bounds(lblp, ublp);

    class subgame_stats {
    public:
        std::vector<double> chi;
        double gap;
        double reach_prob;

        subgame_stats(int dim) : chi(dim), gap(NAN), reach_prob(0.0) {}
        subgame_stats(const subgame_stats&) = delete;
        subgame_stats& operator=(const subgame_stats&) = delete;
        subgame_stats(subgame_stats&&) = default;
        subgame_stats& operator=(subgame_stats&&) = default;
    };
    std::vector<subgame_stats> subgames;

    double uncaught_probability = 0.0;
    for (auto&& allocation : def_strategy) {
        std::vector<subgame_stats> edge_subgames;
        for (int i = 0; i < allocation.allocation->size(); i++) edge_subgames.emplace_back(game.numNodes - 1);

        std::vector<bool> indicator(game.numEdges);
        for (auto& edge : *allocation.allocation) {
            if (edge->dst != &game.nodes.back()) indicator[edge->id] = true;
        }

        double local_uncaught_prob = 0.0;
        for (auto&& plan : att_strategy) {
            std::vector<bool> infected(game.numNodes - 1);
            bool detected = false;
            for (auto& edge : plan.plan.edges) {
                if (edge->dst->id > 0) infected[edge->dst->id - 1] = true;
                if (indicator[edge->id]) {
                    int idx = std::find(allocation.allocation->begin(), allocation.allocation->end(), edge) - allocation.allocation->begin();
                    auto& subgame = edge_subgames[idx];
                    subgame.reach_prob += plan.probability;
                    for (int i = 0; i < game.numNodes - 1; i++) {
                        subgame.chi[i] += (infected[i] ? plan.probability : plan.xi[i]);
                    }

                    detected = true;
                    break;
                }
            }

            if (!detected) {
                local_uncaught_prob += plan.probability;
            }
        }

        uncaught_probability += allocation.probability * local_uncaught_prob;

        for (int i = 0; i < edge_subgames.size(); i++) {
            auto& subgame = edge_subgames[i];
            if (subgame.reach_prob < 1e-8) continue;

            for (double& d : subgame.chi) d /= subgame.reach_prob;
            subgame.reach_prob *= allocation.probability;
            subgame.gap = allocation.gadgets[i].get_value(subgame.chi) - lb.getValue(subgame.chi);

            subgames.push_back(std::move(subgame));
        }
    }

    double next_epsilon = epsilon;// / (1-uncaught_probability);
    double best = 0.0;
    const std::vector<double>* best_chi = nullptr;
    for (auto& subgame : subgames) {
        double weighted_excess = subgame.reach_prob * (subgame.gap - next_epsilon);
        if (weighted_excess > best) {
            best = weighted_excess;
            best_chi = &subgame.chi;
        }
    }

    if (best_chi != nullptr) {
        explore(*best_chi, next_epsilon, measure);

        oracled_solve(lblp, ublp);
        update_bounds(lblp, ublp);
    }

}

void HSVI::update_bounds(OracledStageGameLP& lblp, OracledStageGameLP& ublp) {
    lb.add(lblp.extract_gadget());

    const UBValueFunctionComponent ub_point(ublp.get_chi(), ublp.get_value());
    ub.add(ub_point);
    ublp.point_added(ub_point);
}

void HSVI::oracled_solve(OracledStageGameLP& lblp, OracledStageGameLP& ublp) {
    //    std::cout << glob_measure() << " LB ORACLE" << std::endl;
    PROFILE("lower bound",
        bool lb_changed = run_oracle(lblp, lb, ublp, 2);
    )
        //    std::cout << glob_measure() << " UB ORACLE" << std::endl;
        PROFILE("upper bound",
            bool ub_changed = run_oracle(ublp, ub, lblp, 2);
    )

        if (ub_changed) lblp.solve();
}

template<class VF>
bool HSVI::run_oracle(OracledStageGameLP& oracled_lp, const VF& value_function, OracledStageGameLP& auxiliary_lp, int abr_root_alg) {
    bool changed = false;
    while (true) {
        oracled_lp.solve();
        double current_value = oracled_lp.get_value();

        auto att_strategy = oracled_lp.get_attacker_strategy();
        auto def_strategy = oracled_lp.get_defender_strategy();
        auto gadget = oracled_lp.extract_gadget();

#if DEF_ORACLE == 1
        auto def_br = defender_br_rec(game, att_strategy, std::vector<bool>(game.numEdges), K, value_function);
        if (def_br.value > current_value + ZERO && !oracled_allocations_set.count(def_br.allocation)) {
            add(oracled_lp, auxiliary_lp, def_br.allocation);
            changed = true;
            continue;
        }
#endif

#if ATT_ORACLE == 1
        //        std::cout << glob_measure() << " computing abr on DS with support " << def_strategy.size() << std::endl;
        AttackerBR abr(game, def_strategy, gadget, abr_root_alg);
        abr.solve();
        //        std::cout << glob_measure() << " computed abr " << abr.get_value() << ", ";
        auto plan = abr.get_plan();
        //        for (auto & edge : plan.edges) {
        //            std::cout << "(" << edge->src->id << "," << edge->dst->id << ") ";
        //        }
        //        std::cout << std::endl;

        if (abr.get_value() > ZERO && !oracled_plans_set.count(plan.edges)) {
            add(oracled_lp, auxiliary_lp, plan);
            changed = true;
            continue;
        }
#endif

#if DEF_ORACLE == 2
        {
            auto def_br = defender_br_bnb(game, att_strategy, K, value_function);
            if (def_br.value > current_value + ZERO && !oracled_allocations_set.count(def_br.allocation)) {
                add(oracled_lp, auxiliary_lp, def_br.allocation);
                changed = true;
                continue;
            }
        }
#endif

        break;
    }

    return changed;
}

void HSVI::add(OracledStageGameLP& lp1, OracledStageGameLP& lp2, const HoneypotAllocation& allocation) {
    oracled_allocations.push_back(allocation);
    oracled_allocations_set.insert(allocation);
    lp1.add_honeypot_allocation(oracled_allocations.back());
    lp2.add_honeypot_allocation(oracled_allocations.back());
}

void HSVI::add(OracledStageGameLP& lp1, OracledStageGameLP& lp2, const AttackPlan& plan) {
    oracled_plans.push_back(plan);
    oracled_plans_set.insert(plan.edges);
    lp1.add_attack_plan(oracled_plans.back());
    lp2.add_attack_plan(oracled_plans.back());
}

void HSVI::export_bounds(const std::string& file_name) {
    auto vectors = cpptoml::make_table_array();
    auto points = cpptoml::make_table_array();

    auto add_vector = [&vectors, this](const LBValueFunctionComponent& vector) {
        auto table = cpptoml::make_table();
        auto a_array = cpptoml::make_array();
        for (double d : vector.a) a_array->push_back(d);
        table->insert("b", vector.b);
        table->insert("a", a_array);

        vectors->push_back(table);
    };

    auto add_point = [&points, this](const UBValueFunctionComponent& point) {
        auto table = cpptoml::make_table();
        auto coords_array = cpptoml::make_array();
        for (double d : point.coords) coords_array->push_back(d);
        table->insert("value", point.v);
        table->insert("coords", coords_array);

        points->push_back(table);
    };

    for (auto& vector : lb.components) add_vector(*vector);
    for (auto& point : ub.components) add_point(point);

    auto root = cpptoml::make_table();
    root->insert("ub", points);
    root->insert("lb", vectors);

    std::ofstream out(file_name);
    out << (*root);
    out.close();
}


#include "best_responses.h"

#include <algorithm>

//template <typename VF>
//DefenderBR defender_br_rec(const GameGraph & game, const std::vector<ap_prob> & attacker_strategy, const std::vector<bool> & honeypot, int K, VF & value_function) {
//    std::vector<bool> best_allocation;
//    double best = -std::numeric_limits<double>::infinity();
//
//    for (int e = 0 ; e < game.numEdges ; e++) {
//        if(honeypot[e]) continue;
//
//        std::vector<bool> current_hp = honeypot;
//        current_hp[e] = true;
//
//        std::vector<std::vector<double>> subgames(game.numEdges);
//        double total_cost = 0.0;
//        for (auto && path : attacker_strategy) {
//            std::vector<bool> infected(game.numNodes-1);
//            double cost = 0.0;
//            for (auto & edge : path.plan.edges) {
//                if(edge->dst->id > 0) infected[edge->dst->id - 1] = true;
//
//                if(current_hp[edge->id]) {
//                    cost += edge->honeypotCost;
//
//                    if(edge->dst != &game.nodes.back()) {
//                        auto & subgame = subgames[edge->id];
//                        if(subgame.size() == 0) subgame.resize(game.numNodes-1, 0.0);
//
//                        for(int i = 0 ; i < game.numNodes-1 ; i++) {
//                            if(infected[i]) subgame[i] += path.probability;
//                            else subgame[i] += path.xi[i];
//                        }
//                    }
//
//                    break;
//                } else {
//                    cost += edge->normalCost;
//                }
//            }
//
//            total_cost += cost;
//        }
//
//        for (auto & subgame : subgames) {
//            if(subgame.size() == 0) continue;
//            else {
//                total_cost += value_function.getValue(subgame);
//            }
//        }
//
//        if (total_cost > best) {
//            best = total_cost;
//            best_allocation = std::move(current_hp);
//        }
//    }
//
//    if (K == 1) {
//        HoneypotAllocation allocation;
//        for (int e = 0 ; e < game.numEdges ; e++) {
//            if (best_allocation[e]) allocation.push_back(game.edges[e].get());
//        }
//
//        return DefenderBR(best, allocation);
//    } else return defender_br_rec(game, attacker_strategy, best_allocation, K-1, value_function);
//}

//template<>
//DefenderBR defender_br_rec<LBValueFunction>(const GameGraph & game, const std::vector<ap_prob> & attacker_strategy, const std::vector<bool> & honeypot, int K, LBValueFunction & value_function);
//
//template<>
//DefenderBR defender_br_rec<UBValueFunction>(const GameGraph & game, const std::vector<ap_prob> & attacker_strategy, const std::vector<bool> & honeypot, int K, UBValueFunction & value_function);

AttackerBR::AttackerBR(const GameGraph& game, const vector<hpa_prob>& def_strategy, const LBValueFunctionComponent& expected, int root_alg)
    : game(game), def_strategy(def_strategy), expected(expected), model(env), cplex(model) {

    cplex.setOut(env.getNullStream());
    cplex.setWarning(env.getNullStream());
    cplex.setParam(IloCplex::Threads, 1);
    cplex.setParam(IloCplex::RootAlg, root_alg);

    char cpx_name[32];

    model.add(path = IloBoolVarArray(env, game.numEdges));
    model.add(delta = IloNumVarArray(env, game.numNodes - 1, 0.0, 1.0));

    for (int i = 0; i < game.numEdges; i++) {
        auto& edge = game.edges[i];
        sprintf(cpx_name, "p(%d,%d)", edge->src->id, edge->dst->id);
        path[i].setName(cpx_name);
    }
    for (int i = 0; i < game.numNodes - 1; i++) {
        sprintf(cpx_name, "delta(v%d)", i);
        delta[i].setName(cpx_name);
    }

    // Flow constraints
    for (int i = 0; i < game.numNodes - 1; i++) {
        auto& node = game.nodes[i];

        IloExpr expr = delta[i];
        for (auto& in : node.in) expr += 1.0 * path[in->id];
        for (auto& out : node.out) expr += -1.0 * path[out->id];
        model.add(expr == 0);
    }

    // There is exactly one source
    model.add(IloSum(delta) == 1);

    // Target constraint (perhaps not necessary?)
    {
        IloExpr expr(env);
        for (auto& in : game.nodes.back().in) {
            expr += 1.0 * path[in->id];
        }
        model.add(expr == 1);
    }

    model.add(xi = IloNumVarArray(env, game.numNodes - 2, 0.0, 1.0));
    for (int i = 0; i < game.numNodes - 2; i++) {
        model.add(xi[i] >= delta[i + 1]);
    }

    for (int i = 0; i < game.numNodes - 2; i++) {
        sprintf(cpx_name, "xi(v%d)", i + 1);
        xi[i].setName(cpx_name);
    }

    IloExpr obj(env);
    obj += expected.b;
    for (int i = 0; i < game.numNodes - 2; i++) {
        obj += expected.a[i] * xi[i];
    }
    for (auto&& s : def_strategy) {
        obj -= add_allocation(s);
        allocation_id++;
    }

    model.add(IloMaximize(env, obj));

}

IloExpr AttackerBR::add_allocation(const hpa_prob& strategy) {
    char cpx_name[32];
    auto& allocation = strategy.allocation;

    IloNumVarArray allocation_flow(env, game.numEdges, 0.0, 1.0);
    model.add(allocation_flow);

    for (int i = 0; i < game.numEdges; i++) {
        model.add(allocation_flow[i] <= path[i]);
    }

    for (int i = 0; i < game.numNodes - 1; i++) {
        auto& node = game.nodes[i];

        IloExpr expr = delta[i];
        for (auto& in : node.in) {
            bool found = std::find(allocation->begin(), allocation->end(), in) != allocation->end();
            if (!found) expr += 1.0 * allocation_flow[in->id];
        }
        for (auto& out : node.out) expr += -1.0 * allocation_flow[out->id];
        model.add(expr == 0);
    }

    IloExpr cost(env);
    for (auto& edge : game.edges) {
        cost += edge->normalCost * allocation_flow[edge->id];
    }
    for (auto& edge : *allocation) {
        cost += (edge->honeypotCost - edge->normalCost) * allocation_flow[edge->id];
    }

    IloNumVarArray chi_prime(env, game.numNodes - 2, 0.0, 1.0);
    model.add(chi_prime);
    for (int i = 0; i < game.numNodes - 2; i++) {
        auto& node = game.nodes[i + 1];
        IloNumVar chi_i = chi_prime[i];

        IloExpr infected(env);
        for (auto& in : node.in) infected += allocation_flow[in->id];

        model.add(chi_i == infected + mult(xi[i], 1 - infected));
    }

    for (int e = 0; e < allocation->size(); e++) {
        auto& edge = (*allocation)[e];
        if (edge->dst == &game.nodes.back()) continue;

        auto& gadget = strategy.gadgets[e];

        IloNumVar Vh(env, -IloInfinity, IloInfinity);
        model.add(Vh);

        sprintf(cpx_name, "V(H%d,%d)", allocation_id, e);
        Vh.setName(cpx_name);

        IloExpr gadget_value(env);
        gadget_value += gadget.b * allocation_flow[edge->id];
        for (int i = 0; i < game.numNodes - 2; i++) {
            gadget_value += gadget.a[i] * mult(chi_prime[i], allocation_flow[edge->id]);
        }

        model.add(Vh == gadget_value);
        cost += Vh;
    }

    return strategy.probability * cost;
}

void AttackerBR::solve() {
    PROFILED("AttackerBR::solve()")
        cplex.solve();
}

double AttackerBR::get_value() {
    return cplex.getObjValue();
}

AttackPlan AttackerBR::get_plan() {
    IloNumArray path_values(env, path.getSize());
    cplex.getValues(path_values, path);
    AttackPlan plan;

    for (auto& node : game.nodes) {
        for (auto& out : node.out) {
            if (path_values[out->id] > 0.5) plan.add(out);
        }
    }

    return plan;
}

void AttackerBR::export_model(const std::string& file_name) const {
    cplex.exportModel(file_name.c_str());
}

IloNumVar AttackerBR::mult(IloExpr x, IloExpr y) {
    IloNumVar z(env, 0.0, 1.0);
    model.add(z);

    model.add(z <= x);
    model.add(z <= y);
    model.add(z >= x + y - 1.0);

    return z;
}

AttackerBR::~AttackerBR() {
    env.end();
}

DefenderBR::DefenderBR(double value, const HoneypotAllocation& allocation) : value(value), allocation(allocation) {}

/*

#include <iostream>
#include <random>
#include <chrono>
#include <functional>
#include <algorithm>
#include <list>
#include <sstream>
#include <fstream>
#include "GameGraph.h"
#include "ActionGenerator.h"
#include "fast_profile.h"
#include "Actions.h"
#include "../ThesisP/solver.h"

using namespace std;

void exportAsOS_POSG(const GameGraph& g, const int HP_EDGES,
    const std::vector<HoneypotAllocation>& allocations, const std::vector<AttackPlan>& plans) {
    int n_nt_states = 1 << (g.numNodes - 2);
    std::vector<std::vector<int>> f_actions(n_nt_states);
    for (int i = 0; i < n_nt_states; i++) {
        int j = -1;
        for (auto&& plan : plans) {
            ++j;

            auto first_inf = plan.edges.front()->dst;
            if (first_inf->id == 0) goto ignore;
            if (i & (1 << (first_inf->id - 1))) goto ignore;

            for (auto&& pre : plan.preconditions) {
                if (pre->id == 0) continue;
                if (pre->id == g.numNodes - 1) goto ignore;
                if (!(i & (1 << (pre->id - 1)))) goto ignore;
            }

            f_actions[i].push_back(j);

        ignore:;
        }
    }

    std::vector<std::string> transitions;
    std::vector<std::string> rewards;
    for (int i = 0; i < n_nt_states; i++) {
        for (int di = 0; di < allocations.size(); di++) {
            auto& allocation = allocations[di];
            for (int ai : f_actions[i]) {
                double cost = 0.0;
                int next = i;
                bool reached = false;

                decltype(allocation.begin()) it;

                for (auto&& edge : plans[ai].edges) {
                    if (edge->dst == &g.nodes.back()) reached = true;
                    else if (edge->dst->id > 0) {
                        next |= 1 << (edge->dst->id - 1);
                    }

                    it = std::find(allocation.begin(), allocation.end(), edge);
                    if (it == allocation.end()) cost += edge->normalCost;
                    else {
                        cost += edge->honeypotCost;
                        break;
                    }
                }

                stringstream tss;
                if (reached) {
                    tss << i << " " << di << " " << ai << " " << HP_EDGES << " " << n_nt_states << " " << 1.0;
                }
                else {
                    tss << i << " " << di << " " << ai << " " << (it - allocation.begin()) << " " << next << " "
                        << 1.0;
                }
                transitions.push_back(tss.str());

                stringstream rss;
                rss << i << " " << di << " " << ai << " " << cost;
                rewards.push_back(rss.str());
            }
        }
    }

    std::ofstream out("game.posg");
    out << (n_nt_states + 1) << " ";
    out << 2 << " ";
    out << (allocations.size() + 1) << " ";
    out << (plans.size() + 1) << " ";
    out << (HP_EDGES + 1) << " ";
    out << transitions.size() + 1 << " ";
    out << rewards.size() + 1 << " ";
    out << 1 << std::endl;

    for (int i = 0; i < n_nt_states; i++) {
        out << i << " 0" << std::endl;
    }
    out << n_nt_states << " 1" << std::endl;

    for (int di = 0; di <= allocations.size(); di++) {
        out << "d" << di << std::endl;
    }
    for (int ai = 0; ai <= plans.size(); ai++) {
        out << "a" << ai << std::endl;
    }

    for (int i = 0; i < HP_EDGES; i++) out << "cont" << i << std::endl;
    out << "end" << std::endl;

    for (auto& fas : f_actions) {
        for (auto fa : fas) out << fa << " ";
        out << std::endl;
    }
    out << plans.size() << std::endl;

    for (int di = 0; di < allocations.size(); di++) {
        out << di << " ";
    }
    out << std::endl;
    out << allocations.size() << std::endl;

    for (auto& t : transitions) out << t << std::endl;
    out << n_nt_states << " " << allocations.size() << " " << plans.size()
        << " 1 " << n_nt_states << " 1.0" << std::endl;

    for (auto& r : rewards) out << r << std::endl;
    out << n_nt_states << " " << allocations.size() << " " << plans.size()
        << " 0.0" << std::endl;

    out << "0";
    for (int i = 0; i < n_nt_states; i++) {
        if (i == 0) out << " " << 1.0;
        else out << " " << 0.0;
        //            out << " " << (1.0 / n_nt_states);
    }
    out << std::endl;

    out.close();
}

int main(int argc, char** argv) {
    PROFILED("main()")
        //        const unsigned int NUM_NODES = argc > 1 ? atoi(argv[1]) : 10;
        const unsigned int HP_EDGES = argc > 2 ? atoi(argv[2]) : 1;
    //        GameGraph g = buildGameGraph(NUM_NODES, (argc > 3 ? atoi(argv[3]) : 0));
    GameGraph g(1);

    try {
        std::ifstream in(argv[1]);
        unsigned int numNodes, numEdges;

        in >> numNodes >> numEdges;
        g = GameGraph(numNodes);

        for (unsigned int i = 0; i < numEdges; i++) {
            unsigned int src, dst;
            double cost, honeyCost;
            in >> src >> dst >> cost >> honeyCost;

            g.connect(g[src], g[dst], cost, honeyCost);
        }
    }
    catch (std::exception e) {
        std::cerr << e.what() << std::endl;
    }

    for (auto&& edge : g.edges) {
        std::cout << edge->src->id << "->" << edge->dst->id << " " << edge->normalCost << "/" << edge->honeypotCost
            << std::endl;
    }




    // Compute dummy lower and upper bound
    std::vector<double> ubPaths(g.numNodes, std::numeric_limits<double>::infinity());
    std::vector<double> lbPaths(g.numNodes, std::numeric_limits<double>::infinity());
    lbPaths.back() = 0.0;
    ubPaths.back() = 0.0;

    for (int i = g.numNodes - 1; i >= 0; i--) {
        for (auto&& edge : g.nodes[i].out) {
            ubPaths[i] = std::min(ubPaths[i], edge->honeypotCost + ubPaths[edge->dst->id]);
            lbPaths[i] = std::min(lbPaths[i], edge->normalCost + lbPaths[edge->dst->id]);
        }
    }

    // Override ubPaths using the perfect info heuristic
    auto pi_solution = pi_solve(g, HP_EDGES);
    ubPaths = pi_solution.valuation;

    ofstream gap("gap.txt");
    gap << 0.01 * (ubPaths[0] - lbPaths[0]) << std::endl;
    gap.close();



    PathAttackerActionGenerator agen(g, 99);
    //        int ai = 0;
    //        for (auto &&action : agen.actions) {
    //            std::cout << (ai++) << ": ";
    //            for (auto &&e : action) {
    //                std::cout << e->src->id << "->" << e->dst->id << " ";
    //            }
    //            std::cout << std::endl;
    //        }

    std::cout << " --- " << std::endl;

    DefenderActionGenerator dgen(g, HP_EDGES);
    //        for (auto &&action : dgen.actions) {
    //            for (auto &&e : action) {
    //                std::cout << e->src->id << "->" << e->dst->id << " ";
    //            }
    //            std::cout << std::endl;
    //        }

    std::cout << " --- " << std::endl;

    std::vector<AttackPlan> attack_plans;
    for (auto&& action : agen.actions) {
        AttackPlan p;
        for (auto&& edge : action) p.add(edge);
        attack_plans.push_back(std::move(p));
    }

    std::cout << "|Adef| = " << dgen.actions.size() << std::endl;
    std::cout << "|Aatt| = " << agen.actions.size() << std::endl;

    exportAsOS_POSG(g, HP_EDGES, dgen.actions, attack_plans);


#ifdef PROFILING
    std::ofstream out;
    out.open("profile.html");
    PROFILING_INFO(out);
    out.close();
#endif

    return 0;
}*/
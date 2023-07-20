
#include <queue>
#include <algorithm>
#include "BestResponses.h"

class br_node {
public:
    const Node* node;
    std::vector<bool> infected;
    std::vector<bool> active_allocations;
    std::vector<const Edge*> path;
    double g;
    double h;

    double uncaught_probability;

    br_node(const GameGraph& g, const Node* node, int n_allocations, double h)
        : node(node), infected(g.numNodes - 1, false), active_allocations(n_allocations, true), g(0.0), h(h), uncaught_probability(1.0) {
        if (node->id > 0) infected[node->id - 1] = true;
    }

    br_node(const Node* node, const vector<bool>& infected, const vector<bool>& active_allocations,
        const vector<const Edge*>& path, double g, double h, double uncaught_probability) : node(node),
        infected(infected),
        active_allocations(
            active_allocations),
        path(path), g(g), h(h),
        uncaught_probability(
            uncaught_probability) {}

    void expand(const std::vector<hpa_prob>& defender_strategy, const std::vector<double>& h_values, std::priority_queue<br_node>& pq) {
        std::vector<int> force_path = { 0, 10, 38, 43 };

        for (const Edge* edge : node->out) {
            if (edge->id != force_path[path.size()]) continue;

            std::vector<bool> new_infected = infected;
            if (edge->dst->id > 0) new_infected[edge->dst->id - 1] = true;

            std::vector<bool> new_active_allocations = active_allocations;

            std::vector<const Edge*> new_path = path;
            new_path.push_back(edge);

            double capture_prob = 0.0;
            double capture_cost = 0.0;

            double remaining_prob = 0.0;
            for (int i = 0; i < defender_strategy.size(); i++) {
                if (new_active_allocations[i]) remaining_prob += defender_strategy[i].probability;
            }

            for (int i = 0; i < defender_strategy.size(); i++) {
                if (!new_active_allocations[i]) continue;

                auto hit = std::find(defender_strategy[i].allocation->begin(), defender_strategy[i].allocation->end(), edge);
                if (hit == defender_strategy[i].allocation->end()) continue;

                new_active_allocations[i] = false;

                int hit_index = hit - defender_strategy[i].allocation->begin();
                auto& gadget = defender_strategy[i].gadgets[hit_index];

                capture_prob += defender_strategy[i].probability;

                double this_cost = edge->honeypotCost + gadget.b;
                for (int j = 0; j < new_infected.size(); j++) {
                    if (new_infected[j]) this_cost += gadget.a[j];
                }

                capture_cost += defender_strategy[i].probability * this_cost;
            }

            if (remaining_prob < 1e-6 && uncaught_probability > 1e-6) {
                std::cout << "this should not happen" << std::endl;
            }

            if (remaining_prob > 1e-6) {
                capture_prob /= remaining_prob;
                capture_cost /= remaining_prob;
            }

            double new_cost = g + uncaught_probability * capture_cost;
            new_cost += uncaught_probability * (1 - capture_prob) * edge->normalCost;

            double new_uncaught_probability = uncaught_probability * (1 - capture_prob);

            pq.emplace(edge->dst, new_infected, new_active_allocations, new_path, new_cost, new_uncaught_probability * h_values[edge->dst->id], new_uncaught_probability);
        }
    }

    bool operator<(const br_node& other) const {
        return g + h > other.g + other.h;
    }
};

AttackPlan compute_BR(const GameGraph& g, const std::vector<hpa_prob>& defender_strategy, const Node& root) {
    std::vector<double> h_values(g.numNodes);
    for (int i = g.numNodes - 2; i >= 0; i--) {
        h_values[i] = std::numeric_limits<double>::infinity();
        for (Edge* e : g.nodes[i].out) {
            h_values[i] = std::min(h_values[i], e->normalCost + h_values[e->dst->id]);
        }
    }

    std::priority_queue<br_node> pq;
    pq.emplace(g, &root, h_values[root.id], defender_strategy.size());

    while (!pq.empty()) {
        br_node current = std::move(const_cast<br_node&>(pq.top())); pq.pop();

        if (current.infected.back()) {
            AttackPlan p;
            for (auto edge : current.path) p.add(edge);
            return p;
        }
        else {
            current.expand(defender_strategy, h_values, pq);
        }
    }

    throw "This should not happen!";
}

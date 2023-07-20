
#include <algorithm>
#include "ActionGenerator.h"

AttackerActionGenerator::AttackerActionGenerator(const GameGraph& game) {
    std::set<const Node*> infected;
    infected.insert(&game.nodes[0]);

    std::vector<const Edge*> seq;
    generate(game, infected, seq);

    std::vector<std::vector<const Edge*>> truncated;
    for (auto&& action : actions) {
        for (int i = 1; i < action.size(); i++) {
            std::vector<const Edge*> truncAction;
            for (int j = i; j < action.size(); j++) truncAction.push_back(action[j]);
            truncated.push_back(truncAction);
        }
    }

    actions.insert(actions.end(), truncated.begin(), truncated.end());

    std::sort(actions.begin(), actions.end());
    actions.erase(std::unique(actions.begin(), actions.end()), actions.end());

    actions.push_back({ game.nodes.back().out[0] });
}

void AttackerActionGenerator::generate(const GameGraph& game, std::set<const Node*>& infected,
    std::vector<const Edge*>& seq) {
    for (auto&& n : infected) {
        for (auto&& e : n->out) {
            if (infected.count(e->dst)) continue;
            else {
                seq.push_back(e);
                if (e->dst == &game.nodes.back()) {
                    actions.push_back(seq);
                }
                else {
                    infected.insert(e->dst);
                    generate(game, infected, seq);
                    infected.erase(e->dst);
                }
                seq.pop_back();
            }
        }
    }
}

void
DefenderActionGenerator::generate(const GameGraph& game, int toAdd, int addFrom, std::vector<const Edge*>& honeypots) {
    if (toAdd == 0) actions.push_back(honeypots);
    else {
        for (int i = addFrom; i < game.edges.size(); i++) {
            //            if (game.edges[i] == game.edges.back()) continue;

            honeypots.push_back(game.edges[i].get());
            generate(game, toAdd - 1, i + 1, honeypots);
            honeypots.pop_back();

#if DEF_ORACLE > 0
            break;
#endif
        }
    }
}

DefenderActionGenerator::DefenderActionGenerator(const GameGraph& game, int honeypotCount) {
    std::vector<const Edge*> honeypots;
    generate(game, honeypotCount, 0, honeypots);
}

void LimitedAttackerActionGenerator::generate(const GameGraph& game, std::set<const Node*> infected,
    std::vector<const Edge*>& seq, int limit) {
    if (limit == 0) {
        actions.push_back(seq);
    }
    else {
        for (int i = 0; i < game.edges.size(); i++) {
            auto&& edge = game.edges[i];

            if (edge == game.edges.back()) continue;
            else if (infected.count(edge->dst)) continue;

            seq.push_back(edge.get());

            if (edge->dst == &game.nodes.back()) {
                actions.push_back(seq);
            }
            else {
                std::set<const Node*> newInfected = infected;
                newInfected.insert(edge->src);
                newInfected.insert(edge->dst);

                generate(game, newInfected, seq, limit - 1);
            }

            seq.pop_back();
        }
    }
}

LimitedAttackerActionGenerator::LimitedAttackerActionGenerator(const GameGraph& game, int limit) {
    std::set<const Node*> infected;
    std::vector<const Edge*> seq;
    generate(game, infected, seq, limit);

    seq.push_back(game.nodes.back().out.front());
    actions.push_back(seq);
}

void PathAttackerActionGenerator::generate(const GameGraph& game, const Node* node, std::vector<const Edge*>& seq,
    int limit) {
    if (limit == 0) actions.push_back(seq);
    else {
        for (auto edge : node->out) {
            seq.push_back(edge);

            if (edge->dst == &game.nodes.back()) {
                actions.push_back(seq);
            }
            else generate(game, edge->dst, seq, limit - 1);

#if ATT_ORACLE > 0
            break;
#endif

            seq.pop_back();
        }
    }
}

PathAttackerActionGenerator::PathAttackerActionGenerator(const GameGraph& game, int limit) {
    for (auto&& node : game.nodes) {
        std::vector<const Edge*> seq;
        //        for(int i = 1 ; i <= limit ; i++) generate(game, &node, seq, i);
        generate(game, &node, seq, 999);
        //        prepare(game, &node, seq, limit);
    }

    std::sort(actions.begin(), actions.end());
    actions.erase(std::unique(actions.begin(), actions.end()), actions.end());
}
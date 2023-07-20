
#ifndef MARGINALIZATION_ACTIONGENERATOR_H
#define MARGINALIZATION_ACTIONGENERATOR_H


#include <set>
#include "GameGraph.h"

class AttackerActionGenerator {
    void generate(const GameGraph& game, std::set<const Node*>& infected, std::vector<const Edge*>& seq);

public:
    std::vector<std::vector<const Edge*>> actions;

    AttackerActionGenerator(const GameGraph& game);
};

class LimitedAttackerActionGenerator {
    void generate(const GameGraph& game, std::set<const Node*> infected, std::vector<const Edge*>& seq, int limit);

public:
    std::vector<std::vector<const Edge*>> actions;

    LimitedAttackerActionGenerator(const GameGraph& game, int limit);
};

class PathAttackerActionGenerator {
    void generate(const GameGraph& game, const Node* node, std::vector<const Edge*>& seq, int limit);

public:
    std::vector<std::vector<const Edge*>> actions;

    PathAttackerActionGenerator(const GameGraph& game, int limit);
};

class DefenderActionGenerator {
    void generate(const GameGraph& game, int toAdd, int addFrom, std::vector<const Edge*>& honeypots);

public:
    std::vector<std::vector<const Edge*>> actions;

    DefenderActionGenerator(const GameGraph& game, int honeypotCount);
};


#endif //MARGINALIZATION_ACTIONGENERATOR_H
#pragma once

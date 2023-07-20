
#ifndef STOCHASTICHONEYPOTGAME_ATTACKERORACLEMILP_H
#define STOCHASTICHONEYPOTGAME_ATTACKERORACLEMILP_H

#include <string>

#define IL_STD
#include <ilcplex/ilocplex.h>
#include "GameGraph.h"

ILOSTLBEGIN


class AttackerOracle {

    const GameGraph& game;
    const Node* const sourceVertex;
    const std::vector<double> valuation;

    IloEnv env;
    IloModel model;
    IloCplex cplex;
    IloObjective objective;
    IloNumVarArray primaryFlowVars;
    IloNumVarArray confValues;
    IloRangeArray c;

    std::vector<bool> reachableNodes;
    std::vector<const Node*> nodes;
    std::vector<const Edge*> edges;
    unsigned int edgeCount;

    std::vector<int> indexTable;

    unsigned int baseVariables;
    unsigned int baseConstraints;

    unsigned int configurationCount;


public:
    AttackerOracle(const GameGraph& game, const Node* const sourceVertex, const std::vector<double> valuation);
    void addDefensiveConfiguration(const std::vector<const Edge*> configuration);

    void solve(const std::vector<double>& probabilities);
    double getValue();
    std::vector<const Edge*> exportPath();

    void exportModel(std::string outputFile);

private:
    void buildFlowConstraints();
    IloExpr primaryFlowVar(const Edge* edge);

};


#endif //STOCHASTICHONEYPOTGAME_ATTACKERORACLEMILP_H
#pragma once

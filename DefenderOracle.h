
#ifndef STOCHASTICHONEYPOTGAME_DEFENDERORACLE_H
#define STOCHASTICHONEYPOTGAME_DEFENDERORACLE_H

#include <string>

#define IL_STD
#include <ilcplex/ilocplex.h>
#include "GameGraph.h"

ILOSTLBEGIN


class DefenderOracle {

    const GameGraph & game;
    const Node * const sourceVertex;
    const std::vector<double> valuation;
    unsigned int maxHPs;

    IloEnv env;
    IloModel model;
    IloCplex cplex;
    IloObjective objective;
    IloNumVarArray confVars;
    IloNumVarArray pathValues;

    std::vector<bool> reachableNodes;
    std::vector<const Node*> nodes;
    std::vector<const Edge*> edges;
    unsigned int edgeCount;

    std::vector<int> indexTable;

    unsigned int baseVariables;
    unsigned int baseConstraints;

    unsigned int numPaths;

public:
    DefenderOracle(const GameGraph &game, const Node * const sourceVertex, const std::vector<double> valuation, unsigned int maxHPs);
    void addPath(const std::vector<const Edge*> path);

    void solve(const std::vector<double> & probabilities);
    double getValue();
    std::vector<const Edge*> exportConfiguration();

    void exportModel(std::string outputFile);

private:
    void buildConfConstraints();

    IloExpr confVar(const Edge * edge);

};


#endif //STOCHASTICHONEYPOTGAME_DEFENDERORACLE_H
#pragma once


#ifndef STOCHASTICGAMESCPP_MAINPROBLEM_H
#define STOCHASTICGAMESCPP_MAINPROBLEM_H

#include <string>

#define IL_STD
#include <ilcplex/ilocplex.h>
#include "GameGraph.h"

ILOSTLBEGIN

class MainProblem {

    const GameGraph& game;
    const Node* const sourceVertex;
    const std::vector<double> valuation;

    IloEnv env;
    IloModel model;
    IloCplex cplex;
    IloNumVarArray x;
    IloRangeArray c;
    IloNumVar V;
    IloRange probSum;

    std::vector<std::vector<const Edge*>> paths;
    std::vector<std::vector<const Edge*>> confs;

public:
    MainProblem(const GameGraph& game, const Node* const sourceVertex, const vector<double>& valuation);

    void addPath(const std::vector<const Edge*>& path);
    void addDefensiveConfiguration(const std::vector<const Edge*>& conf);

    void solve();
    double getObjValue();
    std::vector<double> getDefenderStrategy();
    std::vector<double> getAttackerStrategy();

    void exportModel(std::string outputFile);

private:
    void buildLP();
    double computeValue(const std::vector<const Edge*> conf, const std::vector<const Edge*> path);

};


#endif //STOCHASTICGAMESCPP_MAINPROBLEM_H
#pragma once

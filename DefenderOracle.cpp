
#include <set>
#include "DefenderOracle.h"

void _DefenderOracle_walk(const Node* node, std::vector<bool>& reachableNodes);

DefenderOracle::DefenderOracle(const GameGraph& game, const Node* const sourceVertex, const std::vector<double> valuation, unsigned int maxHPs) :
    game(game), sourceVertex(sourceVertex), valuation(valuation), maxHPs(maxHPs),
    model(env), cplex(model), confVars(env), pathValues(env), objective(IloMaximize(env, 0)),
    reachableNodes(game.numNodes),
    indexTable(game.numEdges, -1), numPaths(0) {
    /*
     * IDENTIFY REACHABLE NODES AND EDGES
     */
    _DefenderOracle_walk(sourceVertex, reachableNodes);

    for (unsigned int i = 0; i < game.numNodes; i++) {
        if (reachableNodes[i]) nodes.push_back(&game.nodes[i]);
    }

    edgeCount = 0;
    for (auto&& edge : game.edges) {
        if (reachableNodes[edge->src->id]) {
            edges.push_back(edge.get());
            indexTable[edge->id] = edgeCount++;
        }
    }

    buildConfConstraints();
    model.add(objective);

    cplex.setOut(env.getNullStream());
    cplex.setError(env.getNullStream());
    cplex.setWarning(env.getNullStream());
    cplex.setWarning(env.getNullStream());
}

void DefenderOracle::buildConfConstraints() {
    char name[20];
    IloExpr expr(env);
    for (auto&& edge : edges) {
        sprintf(name, "l(%d,%d)", edge->src->id, edge->dst->id);
        IloIntVar var(env, 0, 1, name);
        confVars.add(var);
        expr += var;
    }

    model.add(expr - std::min(edgeCount, maxHPs) == 0);
}

void DefenderOracle::addPath(const std::vector<const Edge*> path) {
    numPaths++;

    IloNumVarArray vars(env);
    IloExpr expr(env);
    char name[20];

    for (unsigned int i = 0; i < path.size(); i++) {
        auto&& edge = path[i];

        sprintf(name, "b%d(%d,%d)", numPaths, edge->src->id, edge->dst->id);
        IloNumVar bVar(env, 0.0, 1.0, name);
        vars.add(bVar);
        model.add(bVar);

        if (i == 0) {
            model.add(bVar - confVar(edge) == 0);
            expr += (edge->honeypotCost + valuation[edge->dst->id] - edge->normalCost) * confVar(edge) + edge->normalCost;
        }
        else {
            model.add(bVar - confVar(edge) >= 0);
            model.add(bVar - vars[i - 1] >= 0);
            model.add(bVar - confVar(edge) - vars[i - 1] <= 0);

            expr += (edge->honeypotCost + valuation[edge->dst->id] - edge->normalCost) * bVar - (edge->honeypotCost + valuation[edge->dst->id]) * vars[i - 1] + edge->normalCost;
        }
    }

    IloNumVar V(env);
    model.add(V);
    pathValues.add(V);

    model.add(V - expr == 0);
}

void DefenderOracle::solve(const std::vector<double>& probabilities) {
    IloExpr expr(env);
    for (unsigned int i = 0; i < probabilities.size(); i++) {
        expr += probabilities[i] * pathValues[i];
    }
    objective.setExpr(expr);

    cplex.solve();
}

double DefenderOracle::getValue() {
    return cplex.getObjValue();
}

std::vector<const Edge*> DefenderOracle::exportConfiguration() {
    std::vector<const Edge*> conf;
    IloNumArray confValues(env, edgeCount);

    cplex.getValues(confVars, confValues);
    for (unsigned int i = 0; i < edgeCount; i++) {
        if (confValues[i] > 1 - 1e-4 && confValues[i] < 1 + 1e-4) {
            conf.push_back(edges[i]);
        }
        else if (confValues[i] < -1e-4 || confValues[i] > 1e-4) {
            throw "It seems that the configuration is not integral!";
        }
    }

    return conf;
}

IloExpr DefenderOracle::confVar(const Edge* edge) {
    if (!reachableNodes[edge->src->id] || !reachableNodes[edge->dst->id]) {
        return IloExpr(env);
    }
    else {
        return confVars[indexTable[edge->id]];
    }
}

void DefenderOracle::exportModel(std::string outputFile) {
    cplex.exportModel(outputFile.c_str());
}

void _DefenderOracle_walk(const Node* node, std::vector<bool>& reachableNodes) {
    if (reachableNodes[node->id]) return;
    else {
        reachableNodes[node->id] = true;
        for (auto&& edge : node->out) {
            _DefenderOracle_walk(edge->dst, reachableNodes);
        }
    }
}
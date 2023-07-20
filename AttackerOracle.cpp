
#include <set>
#include "AttackerOracle.h"

void _AttackerOracleMILP_walk(const Node* node, std::vector<bool>& reachableNodes);

AttackerOracle::AttackerOracle(const GameGraph& game, const Node* const sourceVertex, const std::vector<double> valuation) :
    game(game), sourceVertex(sourceVertex), valuation(valuation),
    model(env), cplex(model), primaryFlowVars(env), confValues(env), objective(IloMinimize(env, 0)), c(env),
    reachableNodes(game.numNodes),
    indexTable(game.numEdges, -1), configurationCount(0) {
    /*
     * IDENTIFY REACHABLE NODES AND EDGES
     */
    _AttackerOracleMILP_walk(sourceVertex, reachableNodes);

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

    buildFlowConstraints();
    model.add(objective);

    cplex.setOut(env.getNullStream());
    cplex.setError(env.getNullStream());
    cplex.setWarning(env.getNullStream());
}

void AttackerOracle::buildFlowConstraints() {
    // Build variables
    for (auto&& edge : edges) {
        char name[20];

        // Primary flow
        sprintf(name, "f(%d,%d)", edge->src->id, edge->dst->id);
        primaryFlowVars.add(IloIntVar(env, 0, 1, name));
    }

    // Build constraints
    for (auto&& node : nodes) {
        if (node == sourceVertex) {
            // Primary flow
            IloExpr expr(env);
            for (auto&& edge : node->out) {
                expr += primaryFlowVar(edge);
            }
            c.add(expr == 1);
        }
        else if (node == game.target) {
            // Primary flow
            IloExpr expr(env);
            for (auto&& edge : node->in) {
                expr += primaryFlowVar(edge);
            }
            c.add(expr == 1);
        }
        else {
            // Primary flow
            IloExpr expr(env);
            for (auto&& edge : node->in) {
                expr += primaryFlowVar(edge);
            }
            for (auto&& edge : node->out) {
                expr -= primaryFlowVar(edge);
            }
            c.add(expr == 0);
        }
    }

    model.add(c);

    baseVariables = primaryFlowVars.getSize();
    baseConstraints = c.getSize();
}

void AttackerOracle::addDefensiveConfiguration(const std::vector<const Edge*> configuration) {
    IloNumVarArray vs(env);

    configurationCount++;
    char name[20];
    for (auto&& edge : edges) {
        sprintf(name, "v%d(%d,%d)", configurationCount, edge->src->id, edge->dst->id);
        vs.add(IloNumVar(env, 0.0, IloInfinity, name));
    }
    model.add(vs);

    IloExpr expr(env);

    unsigned int hpIdx = 0;
    unsigned int vsIdx = 0;
    for (auto&& edge : edges) {
        while (hpIdx < configuration.size() && configuration[hpIdx]->id < edge->id) hpIdx++;

        double cost = (hpIdx < configuration.size() && configuration[hpIdx] == edge) ?
            (edge->honeypotCost + valuation[edge->dst->id]) : edge->normalCost;

        IloExpr vsExpr = primaryFlowVar(edge);
        for (unsigned int i = 0; i < hpIdx; i++) {
            vsExpr -= primaryFlowVar(configuration[i]);
        }

        model.add(vs[vsIdx] - cost * vsExpr >= 0);
        expr += vs[vsIdx];
        vsIdx++;
    }

    IloNumVar vConf(env);
    confValues.add(vConf);
    model.add(vConf);

    model.add(vConf - expr == 0);
}

void AttackerOracle::solve(const std::vector<double>& probabilities) {
    IloExpr expr(env);
    for (unsigned int i = 0; i < probabilities.size(); i++) {
        expr += probabilities[i] * confValues[i];
    }
    objective.setExpr(expr);

    cplex.solve();
}

double AttackerOracle::getValue() {
    return cplex.getObjValue();
}

std::vector<const Edge*> AttackerOracle::exportPath() {
    std::vector<const Edge*> path;
    IloNumArray flowValues(env, edgeCount);

    try {
        cplex.getValues(primaryFlowVars, flowValues);
        for (unsigned int i = 0; i < edgeCount; i++) {
            if (flowValues[i] > 1 - 1e-4) path.push_back(edges[i]);
        }
    }
    catch (IloCplex::Exception e) {
        cplex.exportModel("/tmp/attoracle_err.lp");
        std::cerr << e.getMessage() << " " << e.getStatus() << std::endl;

        cplex.clear();
        cplex.importModel(model, "/tmp/attoracle_err.lp");

        cplex.solve();
        std::cout << cplex.getStatus() << std::endl;

        throw e;
    }

    return path;
}

IloExpr AttackerOracle::primaryFlowVar(const Edge* edge) {
    if (!reachableNodes[edge->src->id] || !reachableNodes[edge->dst->id]) {
        return IloExpr(env);
    }
    else {
        return primaryFlowVars[indexTable[edge->id]];
    }
}

void AttackerOracle::exportModel(std::string outputFile) {
    cplex.exportModel(outputFile.c_str());
}

void _AttackerOracleMILP_walk(const Node* node, std::vector<bool>& reachableNodes) {
    if (reachableNodes[node->id]) return;
    else {
        reachableNodes[node->id] = true;
        for (auto&& edge : node->out) {
            _AttackerOracleMILP_walk(edge->dst, reachableNodes);
        }
    }
}
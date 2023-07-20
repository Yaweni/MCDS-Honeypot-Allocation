
#include <set>
#include "MainProblem.h"

MainProblem::MainProblem(const GameGraph& game, const Node* const sourceVertex, const vector<double>& valuation)
    : game(game), sourceVertex(sourceVertex), valuation(valuation),
    model(env), cplex(model), x(env), c(env), V(env, "V"), probSum(env, 1.0, 1.0) {

    buildLP();

    cplex.setOut(env.getNullStream());
    cplex.setError(env.getNullStream());
    cplex.setWarning(env.getNullStream());
}

void MainProblem::buildLP() {
    model.add(V);
    model.add(IloMinimize(env, V));
    model.add(probSum);
}

void MainProblem::addPath(const std::vector<const Edge*>& path) {
    paths.push_back(path);

    IloNumVar prob(env, 0.0, 1.0);
    x.add(prob);
    model.add(prob);

    probSum.setLinearCoef(prob, 1.0);
    for (unsigned int i = 0; i < confs.size(); i++) {
        c[i].setLinearCoef(prob, -computeValue(confs[i], path));
    }
}

void MainProblem::addDefensiveConfiguration(const std::vector<const Edge*>& conf) {
    confs.push_back(conf);

    IloExpr expr = V;
    for (unsigned int i = 0; i < paths.size(); i++) {
        expr -= computeValue(conf, paths[i]) * x[i];
    }

    auto range = (expr >= 0);
    c.add(range);
    model.add(range);
}

double MainProblem::computeValue(const std::vector<const Edge*> conf, const std::vector<const Edge*> path) {
    std::set<const Edge*> honeypots;
    for (auto&& edge : conf) honeypots.insert(edge);

    double value = 0.0;

    for (auto&& pathEdge : path) {
        if (honeypots.count(pathEdge)) {
            value += pathEdge->honeypotCost + valuation[pathEdge->dst->id];
            break;
        }
        else {
            value += pathEdge->normalCost;
        }
    }

    return value;
}

void MainProblem::solve() {
    cplex.solve();
}

double MainProblem::getObjValue() {
    return cplex.getObjValue();
}

std::vector<double> MainProblem::getDefenderStrategy() {
    IloNumArray values(env, confs.size());
    cplex.getDuals(values, c);

    double sum = 0.0;
    std::vector<double> probs(confs.size());
    for (unsigned int i = 0; i < confs.size(); i++) {
        probs[i] = values[i];
        sum += (probs[i] < 0 ? -probs[i] : probs[i]);
    }

    return probs;
}

std::vector<double> MainProblem::getAttackerStrategy() {
    IloNumArray values(env, paths.size());
    cplex.getValues(values, x);

    std::vector<double> probs(paths.size());
    for (unsigned int i = 0; i < paths.size(); i++) {
        probs[i] = values[i];
    }
    return probs;
}

void MainProblem::exportModel(std::string outputFile) {
    cplex.exportModel(outputFile.c_str());
}

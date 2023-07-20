#include "StageGameLP.h"
#include "ValueFunction.h"
#include "fast_profile.h"
#include "ConstraintGenerators.h"

#include <algorithm>

#define IL_STD
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

constexpr bool incompletePlans = true;

void computeOutcome(const AttackPlan& plan, const HoneypotAllocation& allocation, double& cost, int& firstHit);

void StageGameLP::addAttackPlan(const AttackPlan& plan) {
    costMat.emplace_back();
    triggered.emplace_back();

    auto&& costMatRow = costMat.back();
    auto&& triggeredRow = triggered.back();
    costMatRow.reserve(honeypotAllocations.size());
    triggeredRow.reserve(honeypotAllocations.size());

    for (auto&& allocation : honeypotAllocations) {
        double cost;
        int first;

        computeOutcome(plan, allocation, cost, first);
        costMatRow.push_back(cost);
        triggeredRow.push_back(first);
    }
    attackPlans.push_back(plan);
}

void StageGameLP::addHoneypotAllocation(const HoneypotAllocation& allocation) {
    for (int i = attackPlans.size() - 1; i >= 0; --i) {
        double cost;
        int first;

        computeOutcome(attackPlans[i], allocation, cost, first);
        costMat[i].push_back(cost);
        triggered[i].push_back(first);
    }
    honeypotAllocations.push_back(allocation);
}

void computeOutcome(const AttackPlan& plan, const HoneypotAllocation& allocation, double& cost, int& firstHit) {
    cost = 0;
    firstHit = -1;

    for (auto&& edge : plan.edges) {
        auto edgeIt = std::find(allocation.begin(), allocation.end(), edge);
        if (edgeIt == allocation.end()) {
            cost += edge->normalCost;
        }
        else {
            cost += edge->honeypotCost;
            firstHit = edgeIt - allocation.begin();
            break;
        }
    }
}

template <class CGType>
class HData {
public:
    IloExpr sum;
    IloExprArray chiExpressions;
    CGType constraintGenerator;

    HData(IloExpr sum, IloExprArray chiExpressions, CGType&& constraintGenerator) : sum(sum), chiExpressions(chiExpressions), constraintGenerator(constraintGenerator) {}
};

template <class CGType>
class HoneyChi {
public:
    IloRange hConstraint;
    std::vector<HData<CGType>> data;
    double remainingSlack;
};

template <class Factory>
StageGameResult StageGameLP::solve(const std::vector<double>& chi, const Factory& factory, const LBValueFunction& lb, const UBValueFunction& ub, const std::vector<double>& hProbs) {
    PROFILED("StageGameLP::solve(...)")
        //    std::vector<AttackPlan> attackPlans;
        //    for(auto && attackPlan : this->attackPlans) {
        //        if(!attackPlan.preconditions.count(&game.nodes.back())) {
        //            for (auto &&pre : attackPlan.preconditions) {
        //                if (pre->id == 0) continue;
        //                else if (chi[pre->id - 1] < 1e-6) goto skip;
        //            }
        //            for (auto &&eff : attackPlan.effects) {
        //                if (eff->id == 0) continue;
        //                else if (chi[eff->id - 1] > 1 - 1e-6) goto skip;
        //            }
        //        }
        //
        //        attackPlans.push_back(attackPlan);
        //        continue;
        //
        //        skip: ;
        //        std::cout << "Plan skipped:" << std::endl << " ";
        //        for(auto && edge : attackPlan.edges) {
        //            std::cout << " " << edge->src->id << "->" << edge->dst->id;
        //        }
        //        std::cout << std::endl;
        //    }

        PROFILE("construction",
            IloEnv env;
    IloModel model(env);

    IloNumVarArray sigmaA(env, attackPlans.size(), 0.0, CPX_INFBOUND);
    IloNumVarArray xi(env, attackPlans.size() * (game.numNodes - 1), 0.0, CPX_INFBOUND);

    model.add(sigmaA);
    model.add(xi);

    //    for(int i = 0 ; i < attackPlans.size() ; i++) {
    //        char varName[16];
    //        sprintf(varName, "sigma(pi%d)", i);
    //        sigmaA[i].setName(varName);
    //    }

    auto Bexpr = (IloSum(sigmaA) == 1);
    model.add(Bexpr);
    int xiVarId = 0;
    int sigmaVarId = 0;
    for (auto&& attackPlan : attackPlans) {
        for (int nodeId = 1; nodeId < game.numNodes; nodeId++) {
            auto&& node = &game.nodes[nodeId];
            if (attackPlan.preconditions.count(node)) {
                model.add(xi[xiVarId] - sigmaA[sigmaVarId] == 0);
            }
            else if (attackPlan.effects.count(node)) {
                //                model.add(xi[xiVarId] == 0);
                model.add(xi[xiVarId] - sigmaA[sigmaVarId] <= 0);
            }
            else {
                model.add(xi[xiVarId] - sigmaA[sigmaVarId] <= 0);
            }

            //            char varName[16];
            //            sprintf(varName, "xi(pi%d,v%d)", sigmaVarId, nodeId);
            //            xi[xiVarId].setName(varName);

            xiVarId++;
        }
        sigmaVarId++;
    }

    IloRangeArray Aexprs(env);
    int nPlans = attackPlans.size();
    for (int v = 0; v < game.numNodes - 1; v++) {
        IloExpr expr(env);
        for (int p = v; p < xi.getSize(); p += game.numNodes - 1) {
            expr += xi[p];
        }
        Aexprs.add(expr == chi[v]);
    }
    model.add(Aexprs);

    IloNumVar V(env, -CPX_INFBOUND, CPX_INFBOUND, IloNumVar::Type::Float, "V");
    model.add(V);

    int allocationId = 0;
    std::vector<HoneyChi<typename Factory::Type>> honeyChis;
    int vhi = 0;
    int vi = 0;
    int chipi = 0;
    int sumi = 0;
    for (auto&& allocation : honeypotAllocations) {
        IloNumVarArray Vh(env, allocation.size() + (incompletePlans ? 1 : 0), -CPX_INFBOUND, CPX_INFBOUND);

        //        char cpxname[32];
        //        for(int i = 0 ; i < Vh.getSize() ; i++) {
        //            sprintf(cpxname, "vh%d", vhi++);
        //            Vh[i].setName(cpxname);
        //        }

        IloExpr expr(env);

        HoneyChi<typename Factory::Type> hc;

        for (int i = 0; i < nPlans; i++) {
            expr += costMat[i][allocationId] * sigmaA[i];
        }
        expr += IloSum(Vh);
        hc.hConstraint = (expr - V <= 0);
        model.add(hc.hConstraint);

        //        sprintf(cpxname, "v%d", vi++);
        //        hc.hConstraint.setName(cpxname);

        int honeyedgeId = 0;
        for (auto&& honeyedge : allocation) {
            IloExpr sum(env);
            IloExprArray chiExpressions(env, game.numNodes - 1);
            for (int i = 0; i < game.numNodes - 1; i++) chiExpressions[i] = IloExpr(env);

            for (unsigned int i = 0; i < nPlans; i++) {
                if (triggered[i][allocationId] == honeyedgeId) {
                    sum += sigmaA[i];
                    for (auto&& edge : attackPlans[i].edges) {
                        chiExpressions[edge->dst->id - 1] += (sigmaA[i] - xi[i * (game.numNodes - 1) + edge->dst->id - 1]);
                        if (edge == honeyedge) break;
                    }

                    for (int j = 0; j < game.numNodes - 1; j++) {
                        chiExpressions[j] += xi[i * (game.numNodes - 1) + j];
                    }
                }
            }

            //            for(int i = 0 ; i < chiExpressions.getSize() ; i++) {
            //                IloRange rng = (chiExpressions[i] >= -100000);
            //                sprintf(cpxname, "chip%d", chipi++);
            //                rng.setName(cpxname);
            //                model.add(rng);
            //            }

            //            {
            //                IloRange rng = (sum >= -100000);
            //                sprintf(cpxname, "sum%d", sumi++);
            //                rng.setName(cpxname);
            //                model.add(rng);
            //            }

            //            vf.buildConstraints(model, Vh[honeyedgeId], hdata.sum, hdata.chiExpressions);
            hc.data.push_back(HData<typename Factory::Type>(sum, chiExpressions, factory.create(model, Vh[honeyedgeId], sum, chiExpressions)));

            honeyedgeId++;
        }

        if (incompletePlans) {
            IloExpr sum(env);
            IloExprArray chiExpressions(env, game.numNodes - 1);
            for (int i = 0; i < game.numNodes - 1; i++) chiExpressions[i] = IloExpr(env);

            for (unsigned int i = 0; i < nPlans; i++) {
                if (triggered[i][allocationId] < 0 && !attackPlans[i].effects.count(&game.nodes.back())) {
                    sum += sigmaA[i];
                    for (auto&& edge : attackPlans[i].edges) {
                        chiExpressions[edge->dst->id - 1] += (sigmaA[i] - xi[i * (game.numNodes - 1) + edge->dst->id - 1]);
                    }

                    for (int j = 0; j < game.numNodes - 1; j++) {
                        chiExpressions[j] += xi[i * (game.numNodes - 1) + j];
                    }
                }
            }

            //            for(int i = 0 ; i < chiExpressions.getSize() ; i++) {
            //                IloRange rng = (chiExpressions[i] >= -100000);
            //                sprintf(cpxname, "chip%d", chipi++);
            //                rng.setName(cpxname);
            //                model.add(rng);
            //            }

            //            {
            //                IloRange rng = (sum >= -100000);
            //                sprintf(cpxname, "sum%d", sumi++);
            //                rng.setName(cpxname);
            //                model.add(rng);
            //            }

            //            vf.buildConstraints(model, Vh[honeyedgeId], hdata.sum, hdata.chiExpressions);
            hc.data.push_back(HData<typename Factory::Type>(sum, chiExpressions, factory.create(model, Vh[honeyedgeId], sum, chiExpressions)));
        }

        honeyChis.push_back(std::move(hc));

        allocationId++;
    }

    model.add(IloMinimize(env, V));

    IloCplex cplex(model);
    cplex.setOut(env.getNullStream());
    cplex.setWarning(env.getNullStream());
    )

        cplex.setParam(IloCplex::RootAlg, factory.get_cplex_alg());
    //    cplex.setParam(IloCplex::Threads, 8);

    //    if(n) cplex.exportModel("/tmp/test1.lp");
    //    else cplex.exportModel("/tmp/test0.lp");
    //    n = (n+1) % 2;

    cplex.solve();
    bool modified = true;
    while (modified) {
        //        cplex.exportModel("/tmp/test0.lp");

        modified = false;

        PROFILE("oracle prepare_update",
            for (auto&& hc : honeyChis) {
                hc.remainingSlack = cplex.getSlack(hc.hConstraint);
                double dual = cplex.getDual(hc.hConstraint);
                for (auto&& hdata : hc.data) {
                    hdata.constraintGenerator.prepare_update(cplex, hc.remainingSlack, dual);
                }
            }
        )
            PROFILE("oracle apply_update",
                for (auto&& hc : honeyChis) {
                    for (auto&& hdata : hc.data) {
                        modified = hdata.constraintGenerator.apply_update(cplex, hc.remainingSlack) || modified;
                    }
                }
        )

            if (modified) {
                PROFILE("solving",
                    cplex.solve();
                )
            }
    }

    //    if(n) cplex.exportModel("/tmp/test1.lp");
    //    else cplex.exportModel("/tmp/test0.lp");
    //    n = (n+1) % 2;

    StageGameResult result;
    result.a.resize(game.numNodes - 1);
    result.chiSlacks.resize(game.numNodes - 1);

    result.value = cplex.getObjValue();
    for (int i = 0; i < game.numNodes - 1; i++) {
        result.a[i] = cplex.getDual(Aexprs[i]);
        result.chiSlacks[i] = cplex.getSlack(Aexprs[i]);
    }
    result.b = cplex.getDual(Bexpr);

    for (auto&& hc : honeyChis) {
        result.hProbs.push_back(-cplex.getDual(hc.hConstraint));
    }

    //    if(!hProbs.empty()) {
    //        double opt = -std::numeric_limits<double>::infinity();
    //        std::vector<double> optChi;
    //        int hcId = 0;
    //        for (auto &&hc : honeyChis) {
    //            double ph = hProbs[hcId++];
    //            for (auto &&hdata : hc.data) {
    //                double sum = cplex.getValue(hdata.sum);
    //                if (sum < 1e-6) continue;
    //
    //                std::vector<double> chiPrime(game.numNodes - 1);
    //                for (int i = 0; i < game.numNodes - 1; i++) {
    //                    chiPrime[i] = cplex.getValue(hdata.chiExpressions[i]) / sum;
    //                }
    //
    //                double wgap = ph * sum * (ub.getValue(chiPrime) - lb.getValue(chiPrime));
    //                if (wgap > opt) {
    //                    opt = wgap;
    //                    optChi = std::move(chiPrime);
    //                }
    //            }
    //        }
    //
    //        result.nextChiValue = opt;
    //        result.nextChi = std::move(optChi);
    //    }

    IloNumArray sigmaAValues(env, sigmaA.getSize());
    cplex.getValues(sigmaA, sigmaAValues);
    allocationId = 0;
    double currentBRValue = -std::numeric_limits<double>::infinity();
    for (auto&& hc : honeyChis) {
        double util = 0.0;
        double nextChiValue = -std::numeric_limits<double>::infinity();
        std::vector<double> nextChi;

        for (auto&& hdata : hc.data) {
            double sum = cplex.getValue(hdata.sum);
            if (sum < 1e-6) continue;

            std::vector<double> chiPrime(game.numNodes - 1);
            for (int i = 0; i < game.numNodes - 1; i++) {
                if (cplex.getValue(hdata.chiExpressions[i]) / sum > 1.00001) {
                    double chiExprValue = cplex.getValue(hdata.chiExpressions[i]);
                    std::cerr << "This should not happen" << std::endl;
                }
                chiPrime[i] = cplex.getValue(hdata.chiExpressions[i]) / sum;
            }
            double ubValue = ub.getValue(chiPrime);
            util += sum * ubValue;
            double gap = ubValue - lb.getValue(chiPrime);
            if (sum * gap > nextChiValue) {
                nextChiValue = sum * gap;
                nextChi = std::move(chiPrime);
            }
        }

        for (int i = 0; i < nPlans; i++) {
            util += costMat[i][allocationId] * sigmaAValues[i];
        }

        if (util > currentBRValue) {
            currentBRValue = util;
            result.nextChiValue = nextChiValue;
            result.nextChi = std::move(nextChi);
        }

        allocationId++;
    }

    cplex.end();
    env.end();

    return result;
}

const GameGraph& StageGameLP::getGame() const {
    return game;
}

StageGameLP::StageGameLP(const GameGraph& game) : game(game) {}

const vector<AttackPlan>& StageGameLP::getAttackPlans() const {
    return attackPlans;
}

const vector<HoneypotAllocation>& StageGameLP::getHoneypotAllocations() const {
    return honeypotAllocations;
}

template StageGameResult StageGameLP::solve<LBConstraintGeneratorFactory>(const std::vector<double>& chi, const LBConstraintGeneratorFactory& vf, const LBValueFunction& lb, const UBValueFunction& ub, const std::vector<double>& hProbs);
template StageGameResult StageGameLP::solve<UBConstraintGeneratorFactory>(const std::vector<double>& chi, const UBConstraintGeneratorFactory& vf, const LBValueFunction& lb, const UBValueFunction& ub, const std::vector<double>& hProbs);
template StageGameResult StageGameLP::solve<UBConstraintGeneratorFactory2>(const std::vector<double>& chi, const UBConstraintGeneratorFactory2& vf, const LBValueFunction& lb, const UBValueFunction& ub, const std::vector<double>& hProbs);

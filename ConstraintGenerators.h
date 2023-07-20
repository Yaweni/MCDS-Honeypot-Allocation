
#ifndef MARGINALIZATION_CONSTRAINTGENERATORS_H
#define MARGINALIZATION_CONSTRAINTGENERATORS_H

#include <memory>

#define IL_STD
#include <ilcplex/ilocplex.h>
#include "ValueFunction.h"

ILOSTLBEGIN

class LBConstraintGenerator {
    const LBValueFunction& lb;
    IloModel model;
    IloNumVar var;
    IloExpr sum;
    IloExprArray chi;

    double currentUpdateValue;
    const LBValueFunctionComponent* currentUpdate;

public:
    LBConstraintGenerator(const LBValueFunction& lb, IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi);

    void prepare_update(IloCplex& cplex, double& slack, double dual);
    bool apply_update(IloCplex& cplex, double& slack);
};

class UBConstraintGenerator2 {
    const UBValueFunction& ub;
    IloModel model;
    IloNumVar var;
    IloExpr sum;
    IloExprArray chi;

    double currentUpdateValue;
    std::shared_ptr<LBValueFunctionComponent> currentUpdate;

public:
    UBConstraintGenerator2(const UBValueFunction& ub, IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi);

    void prepare_update(IloCplex& cplex, double& slack, double dual);
    bool apply_update(IloCplex& cplex, double& slack);
};


class UBConstraintGenerator {
    const UBValueFunction& ub;
    IloModel& model;

    IloRange betaSum;
    IloRangeArray A;
    IloRange vSum;

    double currentUpdateValue;
    const UBValueFunctionComponent* currentUpdate;

public:
    UBConstraintGenerator(const UBValueFunction& ub, IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi);
    void prepare_update(IloCplex& cplex, double& slack, double dual);
    bool apply_update(IloCplex& cplex, double& slack);
};

class LBConstraintGeneratorFactory {
    const LBValueFunction& lb;

public:
    typedef LBConstraintGenerator Type;

    LBConstraintGeneratorFactory(const LBValueFunction& lb);
    LBConstraintGenerator create(IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi) const;
    int get_cplex_alg() const { return 1; }
};

class UBConstraintGeneratorFactory {
    const UBValueFunction& ub;

public:
    typedef UBConstraintGenerator Type;

    UBConstraintGeneratorFactory(const UBValueFunction& ub);
    UBConstraintGenerator create(IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi) const;
    int get_cplex_alg() const { return 2; }
};

class UBConstraintGeneratorFactory2 {
    const UBValueFunction& ub;

public:
    typedef UBConstraintGenerator2 Type;

    UBConstraintGeneratorFactory2(const UBValueFunction& ub);
    UBConstraintGenerator2 create(IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi) const;
    int get_cplex_alg() const { return 2; }
};

#endif //MARGINALIZATION_CONSTRAINTGENERATORS_H
#pragma once

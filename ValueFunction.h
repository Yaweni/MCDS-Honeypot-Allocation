
#ifndef MARGINALIZATION_VALUEFUNCTION_H
#define MARGINALIZATION_VALUEFUNCTION_H

#include <vector>

#define IL_STD
#include <ilcplex/ilocplex.h>
#include "Actions.h"


class LBValueFunctionComponent {
public:
    std::vector<double> a;
    double b;

    LBValueFunctionComponent() {};
    LBValueFunctionComponent(const std::vector<double>& a, double b);
    bool dominates(const LBValueFunctionComponent& other) const;

    double get_value(const std::vector<double>& chi) const;
};

class LBValueFunction {
public:
    std::vector<std::unique_ptr<LBValueFunctionComponent>> components;
    const int dim;

    void buildConstraints(IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi) const;

    LBValueFunction(int dim);
    void add(const LBValueFunctionComponent& component);
    double getValue(const std::vector<double>& chi) const;
    double getValueApproximate(const std::vector<double>& chi) const {
        return getValue(chi);
    }
};

class UBValueFunctionComponent {
public:
    std::vector<double> coords;
    double v;

    UBValueFunctionComponent(const std::vector<double>& coords, double v);
};

class UBValueFunction {
public:
    std::vector<UBValueFunctionComponent> components;
    std::vector<IloNumVar> beta_vars;
    const int dim;

    IloEnv env;
    IloModel model;
    mutable IloCplex cplex;

    IloNumVar V;

    IloRange V_constr;
    mutable IloRange beta_sum;
    IloRangeArray chis;

    double maximum;

    void buildConstraints(IloModel& model, IloNumVar& var, IloExpr& sum, IloExprArray& chi) const;

    UBValueFunction(int dim);
    ~UBValueFunction();
    void add(const UBValueFunctionComponent& component);
    double getValue(const std::vector<double>& chi) const;
    double getValueApproximate(const std::vector<double>& chi) const;

    LBValueFunctionComponent getFacet(std::vector<double>& chi, double sum) const;
};


#endif //MARGINALIZATION_VALUEFUNCTION_H
#pragma once


#ifndef MARGINALIZATION_SUBPROBLEMORACLES_H
#define MARGINALIZATION_SUBPROBLEMORACLES_H

#define IL_STD
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

#include <memory>
#include "ValueFunction.h"

class HoneypotAllocationSubproblem;

class SubproblemOracle {
public:
    virtual bool prepare() = 0;
    virtual void apply() = 0;
    virtual LBValueFunctionComponent extract_gadget() = 0;
    virtual void gadgetify() = 0;
    virtual void generate_all() = 0;
    virtual void point_added(const UBValueFunctionComponent& point) = 0;
    virtual void alpha_added(const UBValueFunctionComponent& alpha) = 0;
    virtual void reset() = 0;
    virtual ~SubproblemOracle() noexcept;
};
class SubproblemOracleFactory {
public:
    virtual std::unique_ptr<SubproblemOracle> create_oracle(IloCplex cplex, const HoneypotAllocationSubproblem& subproblem) = 0;
    virtual ~SubproblemOracleFactory() noexcept;
    virtual int get_root_alg() = 0;
};

class LBSubproblemOracle : public SubproblemOracle {
    const LBValueFunction& lb;
    const HoneypotAllocationSubproblem& subproblem;
    IloCplex cplex;

    std::vector<IloRange> ranges;
    std::vector<LBValueFunctionComponent> components;

    const LBValueFunctionComponent* best;

public:
    LBSubproblemOracle(const LBValueFunction& lb, const HoneypotAllocationSubproblem& subproblem,
        const IloCplex& cplex);

    bool prepare() override;
    void apply() override;

    LBValueFunctionComponent extract_gadget() override;

    void gadgetify() override;

    void add_component(const LBValueFunctionComponent& component);

    void generate_all() override;

    void point_added(const UBValueFunctionComponent& point) override;

    void alpha_added(const UBValueFunctionComponent& alpha) override;

    void reset() override;

    ~LBSubproblemOracle() noexcept override;
};

class UBSubproblemOracle : public SubproblemOracle {
    const UBValueFunction& ub;
    const HoneypotAllocationSubproblem& subproblem;
    IloCplex cplex;

    double error;
    std::shared_ptr<LBValueFunctionComponent> currentUpdate;

    std::vector<IloRange> ranges;
    std::vector<LBValueFunctionComponent> components;

public:
    UBSubproblemOracle(const UBValueFunction& ub, const HoneypotAllocationSubproblem& subproblem,
        const IloCplex& cplex);

    bool prepare() override;

    void apply() override;

    LBValueFunctionComponent extract_gadget() override;

    void gadgetify() override;

    void generate_all() override;

    void point_added(const UBValueFunctionComponent& point) override;

    void alpha_added(const UBValueFunctionComponent& alpha) override;

    void reset() override;

    ~UBSubproblemOracle() noexcept override;
};



class LBSubproblemOracleFactory : public SubproblemOracleFactory {
    const LBValueFunction& lb;

public:
    LBSubproblemOracleFactory(const LBValueFunction& lb);
    unique_ptr<SubproblemOracle> create_oracle(IloCplex cplex, const HoneypotAllocationSubproblem& subproblem) override;
    int get_root_alg() override { return 1; }
};

class UBSubproblemOracleFactory : public SubproblemOracleFactory {
    const UBValueFunction& ub;

public:
    UBSubproblemOracleFactory(const UBValueFunction& ub);
    unique_ptr<SubproblemOracle> create_oracle(IloCplex cplex, const HoneypotAllocationSubproblem& subproblem) override;
    int get_root_alg() override { return 2; }
};


#endif //MARGINALIZATION_SUBPROBLEMORACLES_H
#pragma once

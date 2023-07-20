
#include "solver.h"
#include "Solution.h"
#include "TermModifier.h"
#include "AttackerOracle.h"
#include "DefenderOracle.h"
#include "MainProblem.h"

#include <iostream>
#include <chrono>

Solution pi_solve(const GameGraph& g, unsigned int honeypots) {
    using namespace std::chrono;

    Solution sol(g.numNodes);
    for (int node = g.numNodes - 2; node >= 0; node--) {
        std::cout << VT::Bold
            << "Node " << setw(3) << node << ": "
            << VT::Reset
            << "Solving"
            << std::flush;

        AttackerOracle attOracle(g, g[node], sol.valuation);
        DefenderOracle defOracle(g, g[node], sol.valuation, honeypots);
        MainProblem mp(g, g[node], sol.valuation);
        std::vector<std::vector<const Edge*>> defensiveConfigurations;

        attOracle.addDefensiveConfiguration({});
        mp.addDefensiveConfiguration({});
        defensiveConfigurations.push_back({});

        attOracle.solve({ 1.0 });
        std::vector<const Edge*> initPath = attOracle.exportPath();
        defOracle.addPath(initPath);
        mp.addPath(initPath);

        unsigned long defOracleTime = 0;
        unsigned long attOracleTime = 0;
        unsigned long mpTime = 0;

        double value, attBR, defBR;
        do {
            auto begin = steady_clock::now();
            mp.solve();
            mpTime += duration_cast<milliseconds>(steady_clock::now() - begin).count();
            value = mp.getObjValue();
            //            mp.exportModel("/tmp/mp.lp");

            begin = steady_clock::now();
            defOracle.solve(mp.getAttackerStrategy());
            defOracleTime += duration_cast<milliseconds>(steady_clock::now() - begin).count();
            defBR = defOracle.getValue();
            //            defOracle.exportModel("/tmp/deforacle.lp");

            begin = steady_clock::now();
            attOracle.solve(mp.getDefenderStrategy());
            long time = duration_cast<milliseconds>(steady_clock::now() - begin).count();
            attOracleTime += time;
            attBR = attOracle.getValue();
            //            attOracle.exportModel("/tmp/attoracle.lp");

            std::vector<const Edge*> path = attOracle.exportPath();
            std::vector<const Edge*> conf = defOracle.exportConfiguration();

            if (attBR < value - 1e-4) {
                defOracle.addPath(path);
                mp.addPath(path);
            }

            if (defBR > value + 1e-4) {
                attOracle.addDefensiveConfiguration(conf);
                mp.addDefensiveConfiguration(conf);
                defensiveConfigurations.push_back(conf);
            }

            std::cout << "\r\33[2K"
                << VT::Bold
                << "Node " << setw(3) << node << ": "
                << VT::Reset
                << "Solving   [ " << attBR << " .. " << value << " .. " << defBR << " ]"
                << std::flush;
        } while (value - attBR > 1e-4 || defBR - value > 1e-4);

        std::cout << "\r\33[2K"
            << VT::Bold
            << "Node " << setw(3) << node << ": " << value
            << VT::Reset << std::endl << std::flush;

        sol.valuation[node] = value;

        mp.solve();
        auto defStrategy = mp.getDefenderStrategy();
        for (unsigned int i = 0; i < defStrategy.size(); i++) {
            if (defStrategy[i] > 0.0) {
                sol.strategies[node].emplace_back(defensiveConfigurations[i], defStrategy[i]);
            }
        }
    }

    return sol;
}
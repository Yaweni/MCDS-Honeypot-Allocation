
#include <iostream>
#include <vector>
#include <cmath>
#include <ilcplex/cplex.h>

int maine(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Please provide LP file name" << std::endl;
    }
    else {
        int status;
        CPXENVptr env = CPXopenCPLEX(&status);
        CPXLPptr lp = CPXcreateprob(env, &status, "ImportedLP");

        CPXreadcopyprob(env, lp, argv[1], "lp");

        int nVars = CPXgetnumcols(env, lp);
        int nConstr = CPXgetnumrows(env, lp);
        int nnz = CPXgetnumnz(env, lp);

        int orig_objsen = CPXgetobjsen(env, lp);

        CPXLPptr dual = CPXcreateprob(env, &status, "DualLP");

        std::vector<double> objective(nConstr);
        std::vector<double> rhs(nVars);
        std::vector<char> sense(nVars);
        std::vector<int> matbeg(nConstr);
        std::vector<int> matcnt(nConstr);
        std::vector<int> matind(nnz);
        std::vector<double> matval(nnz);
        std::vector<double> lb(nConstr);
        std::vector<double> ub(nConstr);

        int rp_ccnt = 0;
        int rp_nzcnt = 0;
        std::vector<double> rp_obj;
        std::vector<int> rp_cmatbeg;
        std::vector<int> rp_cmatind;
        std::vector<double> rp_cmatval;
        std::vector<double> rp_lb;
        std::vector<double> rp_ub;

        CPXgetrhs(env, lp, &objective[0], 0, nConstr - 1);
        CPXgetobj(env, lp, &rhs[0], 0, nVars - 1);

        // Determine sense of dual constraints
        std::vector<double> orig_lb(nVars);
        std::vector<double> orig_ub(nVars);
        CPXgetlb(env, lp, &orig_lb[0], 0, nVars - 1);
        CPXgetub(env, lp, &orig_ub[0], 0, nVars - 1);
        for (int i = 0; i < nVars; i++) {
            if (orig_lb[i] == -CPX_INFBOUND && orig_ub[i] == CPX_INFBOUND) sense[i] = 'E';
            else if (orig_lb[i] == -CPX_INFBOUND && abs(orig_ub[i]) < 1e-6) {
                if (orig_objsen == CPX_MIN) sense[i] = 'G';
                else sense[i] = 'L';
            }
            else if (abs(orig_lb[i]) < 1e-6 && orig_ub[i] == CPX_INFBOUND) {
                if (orig_objsen == CPX_MIN) sense[i] = 'L';
                else sense[i] = 'G';
            }
            else {
                //                std::cerr << "We cannot process ranged variables yet, " << orig_lb[i] << " <= x <= " << orig_ub[i] << std::endl;
                //                exit(1);

                sense[i] = 'E';

                double alpha = (orig_objsen == CPX_MIN ? orig_lb[i] : orig_ub[i]);
                double beta = (orig_objsen == CPX_MIN ? orig_ub[i] : orig_lb[i]);

                rp_cmatbeg.push_back(rp_nzcnt);
                rp_cmatbeg.push_back(rp_nzcnt + 1);
                rp_obj.push_back(alpha);
                rp_obj.push_back(beta);
                rp_cmatind.push_back(i);
                rp_cmatind.push_back(i);
                rp_cmatval.push_back(1.0);
                rp_cmatval.push_back(1.0);
                rp_lb.push_back(0.0);
                rp_ub.push_back(CPX_INFBOUND);
                rp_lb.push_back(-CPX_INFBOUND);
                rp_ub.push_back(0.0);
                rp_ccnt += 2;
                rp_nzcnt += 2;
            }
        }

        // Determine bounds on dual variables
        std::vector<char> orig_sense(nConstr);
        CPXgetsense(env, lp, &orig_sense[0], 0, nConstr - 1);
        for (int i = 0; i < nConstr; i++) {
            if (orig_sense[i] == 'G') {
                if (orig_objsen == CPX_MIN) {
                    lb[i] = 0.0;
                    ub[i] = CPX_INFBOUND;
                }
                else {
                    lb[i] = -CPX_INFBOUND;
                    ub[i] = 0.0;
                }
            }
            else if (orig_sense[i] == 'L') {
                if (orig_objsen == CPX_MIN) {
                    lb[i] = -CPX_INFBOUND;
                    ub[i] = 0.0;
                }
                else {
                    lb[i] = 0.0;
                    ub[i] = CPX_INFBOUND;
                }
            }
            else {
                lb[i] = -CPX_INFBOUND;
                ub[i] = CPX_INFBOUND;
            }
        }

        CPXgetrows(env, lp, &nnz, &matbeg[0], &matind[0], &matval[0], nnz, &status, 0, nConstr - 1);
        for (int i = 0; i < nConstr; i++) {
            if (i < nConstr - 1) matcnt[i] = matbeg[i + 1] - matbeg[i];
            else matcnt[i] = nnz - matbeg[i];
        }

        CPXcopylp(env, dual, nConstr, nVars, orig_objsen == CPX_MIN ? CPX_MAX : CPX_MIN, &objective[0], &rhs[0], &sense[0], &matbeg[0], &matcnt[0], &matind[0], &matval[0], &lb[0], &ub[0], nullptr);

        if (rp_nzcnt > 0) {
            std::cout << "Dual rows: " << CPXgetnumrows(env, dual) << std::endl;
            std::cout << "Dual cols: " << CPXgetnumcols(env, dual) << std::endl;

            CPXaddcols(env, dual, rp_ccnt, rp_nzcnt, &rp_obj[0], &rp_cmatbeg[0], &rp_cmatind[0], &rp_cmatval[0], &rp_lb[0], &rp_ub[0], nullptr);
        }

        CPXwriteprob(env, dual, "/tmp/dual.lp", "lp");
    }
    return 0;
}

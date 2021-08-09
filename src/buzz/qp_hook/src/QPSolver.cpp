//
// Created by samuel rac on 11/3/19.
//

#include "include/QPSolver.h"

QPSolver::QPSolver(const std::vector<double> &mA, const std::vector<double> &mB, const std::vector<double> &mC) : m_A(
        mA), m_b(mB), m_C(mC) {}

QPSolver::QPSolver(const std::vector<double> &mA, const std::vector<double> &mB, const std::vector<double> &mC,
                   const std::vector<double> &mBndl, const std::vector<double> &mBndu) : m_A(mA), m_b(mB), m_C(mC),
                                                                                         m_bndl(mBndl), m_bndu(mBndu) {}

QPSolver::~QPSolver() {

}

std::vector<double> QPSolver::solve()
{
    real_2d_array a;
    a.setcontent(sqrt(m_A.size()), sqrt(m_A.size()), m_A.data());
    real_1d_array b;
    b.setcontent(m_b.size(), m_b.data());
//    real_1d_array s = "[1,1]";
//    int s_size = sqrt(m_A.size());
//    long s_data[s_size];
//    for (int i=0; i < s_size; i++)
//        s_data[i]=1;
//    integer_1d_array s;
//    s.setcontent(s_size, s_data);
    real_2d_array c;
    c.setcontent(m_C.size()/(sqrt(m_A.size())+1), sqrt(m_A.size())+1, m_C.data());
    int ct_size = m_C.size() / (sqrt(m_A.size()) + 1);
    // long ct_data[ct_size];
    int ct_data[ct_size];
    for (int i=0; i < ct_size; i++)
        ct_data[i]=-1;
    integer_1d_array ct;
    ct.setcontent(ct_size, ct_data);
    real_1d_array x;
    minqpstate state;
    minqpreport rep;

    // create solver, set quadratic/linear terms
    minqpcreate(2, state);
    minqpsetquadraticterm(state, a);
    minqpsetlinearterm(state, b);
    minqpsetlc(state, c, ct);
    // if bounds are set
    if (!m_bndu.empty() && !m_bndl.empty()) {
        real_1d_array bndl;
        bndl.setcontent(m_bndl.size(), m_bndl.data());
        real_1d_array bndu;
        bndu.setcontent(m_bndu.size(), m_bndu.data());
        minqpsetbc(state, bndl, bndu);
    }

    // Set scale of the parameters.
//    minqpsetscale(state, s);


    // Solve problem with BLEIC-based QP solver.
//    minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
//    minqpoptimize(state);
//    minqpresults(state, x, rep);
//    printf("%s\n", x.tostring(1).c_str()); // EXPECTED: [1.500,0.500]

    //
    // Solve problem with DENSE-AUL solver.
    minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 5);
    minqpoptimize(state);
    minqpresults(state, x, rep);
    printf("%s\n", x.tostring(1).c_str());

    for(int i=0; i<x.length(); i++)
        m_solution.push_back(x(i));
    return m_solution;
}

const std::vector<double> &QPSolver::get_solution() const {
    return m_solution;
}

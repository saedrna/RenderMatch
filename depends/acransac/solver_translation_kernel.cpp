/*!
 * \file solver_translation_kernel.cpp
 *
 * \author Han
 * \date 2017/05/09
 *
 *
 */
#include "solver_translation_kernel.hpp"
namespace openMVG {
namespace translation {
namespace kernel {
void Translation2dSolver::Solve(const Mat & x, const Mat & y, vector<Mat3>* Ts)
{
    assert(x.rows() == 2);
    assert(x.cols() == 1);
    assert(x.rows() == y.rows());
    assert(x.cols() == y.cols());
    
    Mat3 Tm = Mat3::Identity();
    Vec2 T = y.col(0) - x.col(0);
    Tm(0, 2) = T(0);
    Tm(1, 2) = T(1);
    Ts->push_back(Tm);
}
}
}
}
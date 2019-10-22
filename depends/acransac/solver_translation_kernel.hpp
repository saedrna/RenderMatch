/*!
 * \file solver_translation_kernel.hpp
 *
 * \author Han
 * \date 2017/05/09
 *
 * 
 */
#pragma once
#include "numeric.h"
namespace openMVG {
namespace translation {
namespace kernel {

using namespace std;
struct Translation2dSolver {
    enum { MINIMUM_SAMPLES = 1 };
    enum { MAX_MODELS = 1 };

    /**
     * \brief y = x + T
     * \param x
     * \param y
     * \param Ts
     */
    static void Solve(const Mat &x, const Mat &y, vector<Mat3> *Ts);
};

struct AsymmetricError {
    static double Error(const Mat &T, const Vec2 &x1, const Vec2 &x2) {
        return (x2 - (T * x1.homogeneous()).eval().hnormalized()).squaredNorm();
    }
};
}
}
}
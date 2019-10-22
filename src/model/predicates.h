/*!
 * \file predicates.h
 *
 * \author Han
 * \date 2017/04/25
 *
 * 几何判断正反的一些函数
 */
#pragma once

#include <base/common.h>
namespace h2o {

constexpr static double EPSILON_LENGTH = 0.0005; // Sketchup support 1/1000 precision
constexpr static double EPSILON_ANGLE = 0.0000000000005;
constexpr static double INV_EPSILON_LENGTH = 2000.0;
typedef Eigen::Matrix<int64_t, 3, 1> Vector3i64;
inline Vector3d round_point(const Vector3d &point) {
    Vector3i64 pointi = (point * INV_EPSILON_LENGTH).cast<int64_t>();
    return pointi.cast<double>() * EPSILON_LENGTH;
}

template <typename T> inline int orient2d(const T &pa, const T &pb, const T &pc) {
    double acx, bcx, acy, bcy;
    acx = pa[0] - pc[0];
    bcx = pb[0] - pc[0];
    acy = pa[1] - pc[1];
    bcy = pb[1] - pc[1];

    double val = acx * bcy - acy * bcx;
    if (val < -EPSILON_LENGTH) {
        return -1;
    } else if (val > EPSILON_LENGTH) {
        return 1;
    } else {
        return 0;
    }
    return 0;
}

template <typename T> inline int incircle(const T &pa, const T &pb, const T &pc, const T &pd) {
    double adx, ady, bdx, bdy, cdx, cdy;
    double abdet, bcdet, cadet;
    double alift, blift, clift;

    adx = pa[0] - pd[0];
    ady = pa[1] - pd[1];
    bdx = pb[0] - pd[0];
    bdy = pb[1] - pd[1];
    cdx = pc[0] - pd[0];
    cdy = pc[1] - pd[1];

    abdet = adx * bdy - bdx * ady;
    bcdet = bdx * cdy - cdx * bdy;
    cadet = cdx * ady - adx * cdy;
    alift = adx * adx + ady * ady;
    blift = bdx * bdx + bdy * bdy;
    clift = cdx * cdx + cdy * cdy;

    double val = alift * bcdet + blift * cadet + clift * abdet;
    if (val < -EPSILON_LENGTH) {
        return -1;
    } else if (val > EPSILON_LENGTH) {
        return 1;
    } else {
        return 0;
    }
    return 0;
}
} // end h2o
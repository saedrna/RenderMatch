#ifndef __FASTANN_DIST_L2_HPP
#define __FASTANN_DIST_L2_HPP

#include <stddef.h> // size_t

namespace fastann {

typedef void (*cl2func)(const unsigned char *, const unsigned char *, unsigned, unsigned, unsigned *); //   cl2func;
typedef void (*sl2func)(const float *, const float *, unsigned, unsigned, float *);    //                      sl2func;
typedef void (*dl2func)(const double *, const double *, unsigned, unsigned, double *); //                   dl2func;

template <class Float> struct dist_l2_wrapper {};

template <> struct dist_l2_wrapper<unsigned char> {
    cl2func func;

    typedef unsigned char Float;
    typedef unsigned AccumFloat;
};

template <> struct dist_l2_wrapper<float> {
    sl2func func;

    typedef float Float;
    typedef float AccumFloat;
};

template <> struct dist_l2_wrapper<double> {
    dl2func func;

    typedef double Float;
    typedef double AccumFloat;
};

/**
 * Returns a best effort distance function.
 *
 * \c D is optional hint of the incoming dimensionality and may be
 * used to tune the distance function returned.
 */
template <class Float> dist_l2_wrapper<Float> dist_l2_best(unsigned D = 0);
}

#endif

/**
 * Generates various types of random points for use in testing.
 */

#ifndef __FASTANN_RAND_POINT_GEN_HPP
#define __FASTANN_RAND_POINT_GEN_HPP

#include "randomkit.h"

namespace fastann {

/**
 * Generates \c N points from a \c D dimensional unit cube.
 *
 * Though easy, this is a _very_ unrealistic source of data for
 * testing approximate NN.
 */
template <class Float> Float *gen_unit_random(unsigned N, unsigned D, unsigned seed) {
    rk_state state;
    Float *ret;

    rk_seed(seed, &state);

    ret = new Float[N * D];

    for (unsigned n = 0; n < N; ++n) {
        for (unsigned d = 0; d < D; ++d) {
            ret[n * D + d] = (Float)rk_double(&state);
        }
    }

    return ret;
}
}

#endif

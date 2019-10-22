#include "dist_l2.hpp"
#include "dist_l2_funcs.hpp"

namespace fastann {

template <> dist_l2_wrapper<unsigned char> dist_l2_best(unsigned D) {
    dist_l2_wrapper<unsigned char> ret;
#ifdef FASTANN_HAS_AVX
    ret.func = &cl2v_2_64;
#else
    ret.func = &cl2v_2_32;
#endif

    return ret;
}

template <> dist_l2_wrapper<float> dist_l2_best(unsigned D) {
    dist_l2_wrapper<float> ret;
#ifdef __SSE__
#ifdef EXPERIMENTAL_ASM
    ret.func = &sl2u_2_16_exp;
#else
    ret.func = &sl2u_2_8;
#endif
#else
    ret.func = &sl2f_1_8;
#endif
    return ret;
}

template <> dist_l2_wrapper<double> dist_l2_best(unsigned D) {
    dist_l2_wrapper<double> ret;
#ifdef __SSE2__
    ret.func = &dl2v_2_8;
#else
    ret.func = &dl2f_1_8;
#endif
    return ret;
}
}

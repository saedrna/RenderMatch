/**
 * A bunch of different distance routines. This file shouldn't be
 * included externally unless you want very fine grained control
 * over the distance function to use.
 **/
#ifndef __FASTANN_DIST_L2_FUNCS_HPP
#define __FASTANN_DIST_L2_FUNCS_HPP

#include <stdint.h>

#include "dist_l2.hpp"
#ifdef _MSC_VER
#include <intrin.h>
#else
#include <x86intrin.h>
#endif
namespace fastann {

/**
 * Unsigned char.
 */
inline void cl2s(const unsigned char *qu, const unsigned char *pnts, unsigned N, unsigned D, unsigned *dsq_out) {
    for (unsigned n = 0; n < N; ++n) {
        dsq_out[n] = 0;
        for (unsigned d = 0; d < D; ++d) {
            dsq_out[n] += ((unsigned)qu[d] - (unsigned)pnts[n * D + d]) * ((unsigned)qu[d] - (unsigned)pnts[n * D + d]);
        }
    }
}

inline void cl2f_1_8(const unsigned char *qu, const unsigned char *pnts, unsigned N, unsigned D, unsigned *dsq_out) {
    for (unsigned n = 0; n < N; ++n) {
        const unsigned char *pnt_n = pnts + n * D;
        dsq_out[n] = 0;
        unsigned d;
        for (d = 0; d < (D & -8); d += 8) {
            dsq_out[n] +=
                ((unsigned)qu[d + 0] - (unsigned)pnt_n[d + 0]) * ((unsigned)qu[d + 0] - (unsigned)pnt_n[d + 0]);
            dsq_out[n] +=
                ((unsigned)qu[d + 1] - (unsigned)pnt_n[d + 1]) * ((unsigned)qu[d + 1] - (unsigned)pnt_n[d + 1]);
            dsq_out[n] +=
                ((unsigned)qu[d + 2] - (unsigned)pnt_n[d + 2]) * ((unsigned)qu[d + 2] - (unsigned)pnt_n[d + 2]);
            dsq_out[n] +=
                ((unsigned)qu[d + 3] - (unsigned)pnt_n[d + 3]) * ((unsigned)qu[d + 3] - (unsigned)pnt_n[d + 3]);
            dsq_out[n] +=
                ((unsigned)qu[d + 4] - (unsigned)pnt_n[d + 4]) * ((unsigned)qu[d + 4] - (unsigned)pnt_n[d + 4]);
            dsq_out[n] +=
                ((unsigned)qu[d + 5] - (unsigned)pnt_n[d + 5]) * ((unsigned)qu[d + 5] - (unsigned)pnt_n[d + 5]);
            dsq_out[n] +=
                ((unsigned)qu[d + 6] - (unsigned)pnt_n[d + 6]) * ((unsigned)qu[d + 6] - (unsigned)pnt_n[d + 6]);
            dsq_out[n] +=
                ((unsigned)qu[d + 7] - (unsigned)pnt_n[d + 7]) * ((unsigned)qu[d + 7] - (unsigned)pnt_n[d + 7]);
        }
        for (; d < D; ++d) {
            dsq_out[n] +=
                ((unsigned)qu[d + 0] - (unsigned)pnt_n[d + 0]) * ((unsigned)qu[d + 0] - (unsigned)pnt_n[d + 0]);
        }
    }
}

inline void cl2v_2_32(const unsigned char *qu, const unsigned char *pnts, unsigned N, unsigned D, unsigned *dsq_out) {
#ifdef _MSC_VER
    __declspec(align(16)) static const unsigned char mask_uc[16] = {
#else
    static const unsigned char mask_uc[16] __attribute__ ((__aligned__(16))) = {
#endif

        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00
    };
    for (unsigned n = 0; n < N; ++n) {
        const unsigned char *pnt_n = pnts + n * D;
        unsigned d = 0;
        __m128i acur1, bcur1, acur2, bcur2;
        __m128i t1, t2, t3;
        __m128i mask = _mm_load_si128((__m128i *)mask_uc);

        __m128i acc1 = {0}; // acc1 <- [0, 0, 0, 0]
        __m128i acc2 = {0}; // acc2 <- [0, 0, 0, 0]

        for (; d < (D & -32); d += 32) {
            acur1 = _mm_loadu_si128((__m128i *)(qu + d));
            bcur1 = _mm_loadu_si128((__m128i *)(pnt_n + d));

            t1 = _mm_adds_epu8(_mm_subs_epu8(acur1, bcur1), _mm_subs_epu8(bcur1, acur1)); // t1 = |a[:8] - b[:8]|_1
            t2 = _mm_and_si128(t1, mask);
            t3 = _mm_and_si128(_mm_srli_si128(t1, 1), mask);

            acc1 = _mm_add_epi32(_mm_add_epi32(acc1, _mm_madd_epi16(t2, t2)), _mm_madd_epi16(t3, t3));

            acur2 = _mm_loadu_si128((__m128i *)(qu + d + 16));
            bcur2 = _mm_loadu_si128((__m128i *)(pnt_n + d + 16));

            t1 = _mm_adds_epu8(_mm_subs_epu8(acur2, bcur2), _mm_subs_epu8(bcur2, acur2)); // t1 = |a[8:16] - b[8:16]|_1
            t2 = _mm_and_si128(t1, mask);
            t3 = _mm_and_si128(_mm_srli_si128(t1, 1), mask);

            acc2 = _mm_add_epi32(_mm_add_epi32(acc2, _mm_madd_epi16(t2, t2)), _mm_madd_epi16(t3, t3));
        }

        // Horizontal add
        acc1 = _mm_add_epi32(acc1, acc2);
        acc2 = _mm_shuffle_epi32(acc1, 0xe);
        acc1 = _mm_add_epi32(acc1, acc2);
        acc2 = _mm_shuffle_epi32(acc1, 0x1);
        acc1 = _mm_add_epi32(acc1, acc2);
        dsq_out[n] = _mm_cvtsi128_si32(acc1);

        for (; d < D; ++d) {
            dsq_out[n] += ((unsigned)qu[d] - (unsigned)pnt_n[d]) * ((unsigned)qu[d] - (unsigned)pnt_n[d]);
        }
    }
}

//
#ifdef FASTANN_HAS_AVX
inline void cl2v_2_64(const unsigned char *qu, const unsigned char *pnts, unsigned N, unsigned D, unsigned *dsq_out) {
#ifdef _MSC_VER
    __declspec(align(32)) static const unsigned char mask_uc[32] = {
#else
    static const unsigned char mask_uc[32] __attribute__ ((__aligned__(32))) = {
#endif
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00,
        0xff,
        0x00
    };
    for (unsigned n = 0; n < N; ++n) {
        const unsigned char *pnt_n = pnts + n * D;
        unsigned d = 0;
        __m256i acur1, bcur1, acur2, bcur2;
        __m256i t1, t2, t3;
        __m256i mask = _mm256_load_si256((__m256i *)mask_uc);

        __m256i acc1 = {0}; // acc1 <- [0, 0, 0, 0]
        __m256i acc2 = {0}; // acc2 <- [0, 0, 0, 0]

        for (; d < (D & -64); d += 64) {
            acur1 = _mm256_loadu_si256((__m256i *)(qu + d));
            bcur1 = _mm256_loadu_si256((__m256i *)(pnt_n + d));

            t1 = _mm256_adds_epu8(_mm256_subs_epu8(acur1, bcur1),
                                  _mm256_subs_epu8(bcur1, acur1)); // t1 = |a[:32] - b[:32]|_1
            t2 = _mm256_and_si256(t1, mask);
            t3 = _mm256_and_si256(_mm256_srli_si256(t1, 1), mask);

            acc1 = _mm256_add_epi32(_mm256_add_epi32(acc1, _mm256_madd_epi16(t2, t2)), _mm256_madd_epi16(t3, t3));

            acur2 = _mm256_loadu_si256((__m256i *)(qu + d + 32));
            bcur2 = _mm256_loadu_si256((__m256i *)(pnt_n + d + 32));

            t1 = _mm256_adds_epu8(_mm256_subs_epu8(acur2, bcur2),
                                  _mm256_subs_epu8(bcur2, acur2)); // t1 = |a[8:16] - b[8:16]|_1
            t2 = _mm256_and_si256(t1, mask);
            t3 = _mm256_and_si256(_mm256_srli_si256(t1, 1), mask);

            acc2 = _mm256_add_epi32(_mm256_add_epi32(acc2, _mm256_madd_epi16(t2, t2)), _mm256_madd_epi16(t3, t3));
        }

        // Horizontal add
        acc1 = _mm256_add_epi32(acc1, acc2);
        acc2 = _mm256_shuffle_epi32(acc1, 0xe);
        acc1 = _mm256_add_epi32(acc1, acc2);
        acc2 = _mm256_shuffle_epi32(acc1, 0x1);
        acc1 = _mm256_add_epi32(acc1, acc2);
#ifndef __clang__
        dsq_out[n] = acc1.m256i_i32[0] + acc1.m256i_i32[4];
#else
        int32_t *pacc1 = (int32_t *)&acc1;
        dsq_out[n] = pacc1[0] + pacc1[4];
#endif
        for (; d < D; ++d) {
            dsq_out[n] += ((unsigned)qu[d] - (unsigned)pnt_n[d]) * ((unsigned)qu[d] - (unsigned)pnt_n[d]);
        }
    }
}
#endif

/**
 * Single-precision.
 */
inline void sl2s(const float *qu, const float *pnts, unsigned N, unsigned D, float *dsq_out) {
    for (unsigned n = 0; n < N; ++n) {
        dsq_out[n] = 0.0f;
        for (unsigned d = 0; d < D; ++d) {
            dsq_out[n] += (qu[d] - pnts[n * D + d]) * (qu[d] - pnts[n * D + d]);
        }
    }
}

inline void sl2f_1_8(const float *qu, const float *pnts, unsigned N, unsigned D, float *dsq_out) {
    for (unsigned n = 0; n < N; ++n) {
        const float *pnt_n = pnts + n * D;
        dsq_out[n] = 0.0f;
        unsigned d;
        for (d = 0; d < (D & -8); d += 8) {
            dsq_out[n] += (qu[d + 0] - pnt_n[d + 0]) * (qu[d + 0] - pnt_n[d + 0]);
            dsq_out[n] += (qu[d + 1] - pnt_n[d + 1]) * (qu[d + 1] - pnt_n[d + 1]);
            dsq_out[n] += (qu[d + 2] - pnt_n[d + 2]) * (qu[d + 2] - pnt_n[d + 2]);
            dsq_out[n] += (qu[d + 3] - pnt_n[d + 3]) * (qu[d + 3] - pnt_n[d + 3]);
            dsq_out[n] += (qu[d + 4] - pnt_n[d + 4]) * (qu[d + 4] - pnt_n[d + 4]);
            dsq_out[n] += (qu[d + 5] - pnt_n[d + 5]) * (qu[d + 5] - pnt_n[d + 5]);
            dsq_out[n] += (qu[d + 6] - pnt_n[d + 6]) * (qu[d + 6] - pnt_n[d + 6]);
            dsq_out[n] += (qu[d + 7] - pnt_n[d + 7]) * (qu[d + 7] - pnt_n[d + 7]);
        }

        for (; d < D; ++d) {
            dsq_out[n] += (qu[d] - pnt_n[d]) * (qu[d] - pnt_n[d]);
        }
    }
}

inline void sl2u_2_8(const float *qu, const float *pnts, unsigned N, unsigned D, float *dsq_out) {
    for (unsigned n = 0; n < N; ++n) {
        const float *pnt_n = pnts + n * D;
        unsigned d = 0;
        __m128 acc1, acc2;                 // Two accumulators
        __m128 acur1, bcur1, acur2, bcur2; // Contain 4 elems each from a and b

        acc1 = _mm_setzero_ps();
        acc2 = _mm_setzero_ps();
        for (; d < (D & -8); d += 8) { // Unroll 8
            acur1 = _mm_loadu_ps(qu + d);
            bcur1 = _mm_loadu_ps(pnt_n + d);
            acur2 = _mm_loadu_ps(qu + d + 4);
            bcur2 = _mm_loadu_ps(pnt_n + d + 4);

            acc1 = _mm_add_ps(acc1, _mm_mul_ps(_mm_sub_ps(acur1, bcur1), _mm_sub_ps(acur1, bcur1)));
            acc2 = _mm_add_ps(acc2, _mm_mul_ps(_mm_sub_ps(acur2, bcur2), _mm_sub_ps(acur2, bcur2)));
        }

        for (; d < D; ++d) { // Finish up
            acur1 = _mm_load_ss(qu + d);
            bcur1 = _mm_load_ss(pnt_n + d);

            acc1 = _mm_add_ss(acc1, _mm_mul_ss(_mm_sub_ss(acur1, bcur1), _mm_sub_ss(acur1, bcur1)));
        }

        // Horizontal add - sigh...
        acc1 = _mm_add_ps(acc1, acc2);          // acc1 = [a1 a2 a3 a4]
        acc2 = _mm_movehl_ps(acc2, acc1);       // acc2 = [a3 a4 a3 a4]
        acc1 = _mm_add_ps(acc1, acc2);          // acc1 = [a1+a3 a2+a4 a3+a3 a4+a4]
        acc2 = _mm_shuffle_ps(acc1, acc1, 0x1); // acc2 = [a2+a4 a1+a3 a1+a3 a1+a3]
        acc1 = _mm_add_ss(acc1, acc2);          // acc1 = [a1+a2+a3+a4 ...]
        _mm_store_ss(&dsq_out[n], acc1);
    }
}

/**
 * Double precision.
 */
inline void dl2s(const double *qu, const double *pnts, unsigned N, unsigned D, double *dsq_out) {
    for (unsigned n = 0; n < N; ++n) {
        dsq_out[n] = 0.0;
        for (unsigned d = 0; d < D; ++d) {
            dsq_out[n] += (qu[d] - pnts[n * D + d]) * (qu[d] - pnts[n * D + d]);
        }
    }
}

inline void dl2f_1_8(const double *qu, const double *pnts, unsigned N, unsigned D, double *dsq_out) {
    for (unsigned n = 0; n < N; ++n) {
        const double *pnt_n = pnts + n * D;
        unsigned d;
        dsq_out[n] = 0.0;
        for (d = 0; d < (D & -8); d += 8) {
            dsq_out[n] += (qu[d + 0] - pnt_n[d + 0]) * (qu[d + 0] - pnt_n[d + 0]);
            dsq_out[n] += (qu[d + 1] - pnt_n[d + 1]) * (qu[d + 1] - pnt_n[d + 1]);
            dsq_out[n] += (qu[d + 2] - pnt_n[d + 2]) * (qu[d + 2] - pnt_n[d + 2]);
            dsq_out[n] += (qu[d + 3] - pnt_n[d + 3]) * (qu[d + 3] - pnt_n[d + 3]);
            dsq_out[n] += (qu[d + 4] - pnt_n[d + 4]) * (qu[d + 4] - pnt_n[d + 4]);
            dsq_out[n] += (qu[d + 5] - pnt_n[d + 5]) * (qu[d + 5] - pnt_n[d + 5]);
            dsq_out[n] += (qu[d + 6] - pnt_n[d + 6]) * (qu[d + 6] - pnt_n[d + 6]);
            dsq_out[n] += (qu[d + 7] - pnt_n[d + 7]) * (qu[d + 7] - pnt_n[d + 7]);
        }
        for (; d < D; ++d) {
            dsq_out[n] += (qu[d + 0] - pnt_n[d + 0]) * (qu[d + 0] - pnt_n[d + 0]);
        }
    }
}

inline void dl2v_2_8(const double *qu, const double *pnts, unsigned N, unsigned D, double *dsq_out) {
    for (unsigned n = 0; n < N; ++n) {
        const double *pnt_n = pnts + n * D;
        unsigned d = 0;

        __m128d acc1, acc2;                 // Two accumulators
        __m128d acur1, bcur1, acur2, bcur2; // Contain 2 elems each from a and b

        acc1 = _mm_setzero_pd();
        acc2 = _mm_setzero_pd();
        for (; d < (D & -8); d += 8) { // Unroll 8
            acur1 = _mm_loadu_pd(qu + d);
            bcur1 = _mm_loadu_pd(pnt_n + d);
            acc1 = _mm_add_pd(acc1, _mm_mul_pd(_mm_sub_pd(acur1, bcur1), _mm_sub_pd(acur1, bcur1)));

            acur2 = _mm_loadu_pd(qu + d + 2);
            bcur2 = _mm_loadu_pd(pnt_n + d + 2);
            acc2 = _mm_add_pd(acc2, _mm_mul_pd(_mm_sub_pd(acur2, bcur2), _mm_sub_pd(acur2, bcur2)));

            acur1 = _mm_loadu_pd(qu + d + 4);
            bcur1 = _mm_loadu_pd(pnt_n + d + 4);
            acc1 = _mm_add_pd(acc1, _mm_mul_pd(_mm_sub_pd(acur1, bcur1), _mm_sub_pd(acur1, bcur1)));

            acur2 = _mm_loadu_pd(qu + d + 6);
            bcur2 = _mm_loadu_pd(pnt_n + d + 6);
            acc2 = _mm_add_pd(acc2, _mm_mul_pd(_mm_sub_pd(acur2, bcur2), _mm_sub_pd(acur2, bcur2)));
        }

        for (; d < D; ++d) { // Finish up
            acur1 = _mm_load_sd(qu + d);
            bcur1 = _mm_load_sd(pnt_n + d);

            acc1 = _mm_add_sd(acc1, _mm_mul_sd(_mm_sub_sd(acur1, bcur1), _mm_sub_sd(acur1, bcur1)));
        }

        // Horizontal add
        acc1 = _mm_add_pd(acc1, acc2);          // acc1 = [a1 a2]
        acc2 = _mm_shuffle_pd(acc1, acc1, 0x1); // acc2 = [a2 a1 a1 a1]
        acc1 = _mm_add_sd(acc1, acc2);          // acc1 = [a1+a2 ...]
        _mm_store_sd(&dsq_out[n], acc1);
    }
}
} // namespace fastann

#ifdef EXPERIMENTAL_ASM
extern "C" void dl2v_2_8_exp(const double *qu, const double *pnts, unsigned N, unsigned D, double *dsq_out);

extern "C" void sl2u_2_16_exp(const float *qu, const float *pnts, unsigned N, unsigned D, float *dsq_out);

// extern "C" void dl2v_2_8_exp2(
//        const double* qu,
//        const double* pnts,
//        unsigned N,
//        unsigned D,
//        double* dsq_out);
#endif

#endif

/*!
 * \file math.cpp
 *
 * \author Han
 * \date 2017/05/08
 *
 * 一些简单的数学函数
 */
#include <base/math.h>

namespace h2o {

size_t n_choose_k(const size_t n, const size_t k) {
    if (k == 0) {
        return 1;
    }

    return (n * n_choose_k(n - 1, k - 1)) / k;
}
} // namespace h2o

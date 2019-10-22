/*!
 * \file math.h
 *
 * \author Han
 * \date 2017/05/08
 *
 * 一些简单的数学函数
 */
#pragma once

#include <base/common.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

namespace h2o {

// Return 1 if number is positive, -1 if negative, and 0 if the number is 0.
template <typename T> int sign_of_number(const T val);

// Check if the given floating point number is a not-a-number (NaN) value.
inline bool is_nan(const float x);
inline bool is_nan(const double x);

// Check if the given floating point array contains a NaN value.
template <typename Derived> inline bool is_nan(const Eigen::MatrixBase<Derived> &x);

// Check if the given floating point number is a infinity.
inline bool is_inf(const float x);
inline bool is_inf(const double x);

// Check if the given floating point array contains infinity.
template <typename Derived> inline bool is_inf(const Eigen::MatrixBase<Derived> &x);

// Clip the given value to a low and maximum value.
template <typename T> inline T clip(const T &value, const T &low, const T &high);

// Convert angle in degree to radians.
inline float deg_to_rad(const float deg);
inline double deg_to_rad(const double deg);

// Convert angle in radians to degree.
inline float rad_to_deg(const float rad);
inline double rad_to_deg(const double rad);

// Determine median value in vector. Returns NaN for empty vectors.
template <typename T> double median(const std::vector<T> &elems);

// Determine mean value in a vector.
template <typename T> double mean(const std::vector<T> &elems);

// Determine sample variance in a vector.
template <typename T> double variance(const std::vector<T> &elems);

// Determine sample standard deviation in a vector.
template <typename T> double stddev(const std::vector<T> &elems);

template <typename T> double rmse(const std::vector<T> &elems);

// Check if any of the values in the vector is less than the given threshold.
template <typename T> bool any_less_than(std::vector<T> elems, T threshold);

// Check if any of the values in the vector is greater than the given threshold.
template <typename T> bool any_greater_than(std::vector<T> elems, T threshold);

// Generate N-choose-K combinations.
//
// Note that elements in range [first, last) must be in sorted order,
// according to `std::less`.
template <class Iterator> bool next_combination(Iterator first, Iterator middle, Iterator last);

// Sigmoid function.
template <typename T> T sigmoid(const T x, const T alpha = 1);

// Scale values according to sigmoid transform.
//
//   x \in [0, 1] -> x \in [-x0, x0] -> sigmoid(x, alpha) -> x \in [0, 1]
//
// @param x        Value to be scaled in the range [0, 1].
// @param x0       Spread that determines the range x is scaled to.
// @param alpha    Exponential sigmoid factor.
//
// @return         The scaled value in the range [0, 1].
template <typename T> T scale_sigmoid(T x, const T alpha = 1, const T x0 = 10);

// Binomial coefficient or all combinations, defined as n! / ((n - k)! k!).
size_t n_choose_k(const size_t n, const size_t k);

// Cast value from one type to another and truncate instead of overflow, if the
// input value is out of range of the output data type.
template <typename T1, typename T2> T2 truncate_cast(const T1 value);

// Compute the n-th percentile in the given sequence.
template <typename T> T percentile(const std::vector<T> &elems, const double p);

template <typename T> bool is_almost_zero(T value);
////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

namespace internal {

template <class Iterator> bool next_combination(Iterator first1, Iterator last1, Iterator first2, Iterator last2) {
    if ((first1 == last1) || (first2 == last2)) {
        return false;
    }
    Iterator m1 = last1;
    Iterator m2 = last2;
    --m2;
    while (--m1 != first1 && *m1 >= *m2) {
    }
    bool result = (m1 == first1) && *first1 >= *m2;
    if (!result) {
        while (first2 != m2 && *m1 >= *first2) {
            ++first2;
        }
        first1 = m1;
        std::iter_swap(first1, first2);
        ++first1;
        ++first2;
    }
    if ((first1 != last1) && (first2 != last2)) {
        m1 = last1;
        m2 = first2;
        while ((m1 != first1) && (m2 != last2)) {
            std::iter_swap(--m1, m2);
            ++m2;
        }
        std::reverse(first1, m1);
        std::reverse(first1, last1);
        std::reverse(m2, last2);
        std::reverse(first2, last2);
    }
    return !result;
}

} // namespace internal

template <typename T> int sign_of_number(const T val) { return (T(0) < val) - (val < T(0)); }

bool is_nan(const float x) { return x != x; }
bool is_nan(const double x) { return x != x; }

template <typename Derived> bool is_nan(const Eigen::MatrixBase<Derived> &x) { return !(x.array() == x.array()).all(); }

bool is_inf(const float x) { return !is_nan(x) && is_nan(x - x); }
bool is_inf(const double x) { return !is_nan(x) && is_nan(x - x); }

template <typename Derived> bool is_inf(const Eigen::MatrixBase<Derived> &x) {
    return !((x - x).array() == (x - x).array()).all();
}

template <typename T> T clip(const T &value, const T &low, const T &high) {
    return std::max(low, std::min(value, high));
}

float deg_to_rad(const float deg) { return deg * 0.0174532925199432954743716805978692718781530857086181640625f; }

double deg_to_rad(const double deg) { return deg * 0.0174532925199432954743716805978692718781530857086181640625; }

// Convert angle in radians to degree.
float rad_to_deg(const float rad) { return rad * 57.29577951308232286464772187173366546630859375f; }

double rad_to_deg(const double rad) { return rad * 57.29577951308232286464772187173366546630859375; }

template <typename T> double median(const std::vector<T> &elems) {
    CHECK(!elems.empty());

    const size_t mid_idx = elems.size() / 2;

    std::vector<T> ordered_elems = elems;
    std::nth_element(ordered_elems.begin(), ordered_elems.begin() + mid_idx, ordered_elems.end());

    if (elems.size() % 2 == 0) {
        const T mid_element1 = ordered_elems[mid_idx];
        const T mid_element2 = *std::max_element(ordered_elems.begin(), ordered_elems.begin() + mid_idx);
        return (mid_element1 + mid_element2) / 2.0;
    } else {
        return ordered_elems[mid_idx];
    }
}

template <typename T> T percentile(const std::vector<T> &elems, const double p) {
    CHECK(!elems.empty());
    CHECK_GE(p, 0);
    CHECK_LE(p, 100);

    const int idx = static_cast<int>(std::round(p / 100 * (elems.size() - 1)));
    const size_t percentile_idx = std::max(0, std::min(static_cast<int>(elems.size() - 1), idx));

    std::vector<T> ordered_elems = elems;
    std::nth_element(ordered_elems.begin(), ordered_elems.begin() + percentile_idx, ordered_elems.end());

    return ordered_elems.at(percentile_idx);
}

template <typename T> bool is_almost_zero(T value) { return std::abs(value - T(0)) < 1e-4f; }

template <typename T> double mean(const std::vector<T> &elems) {
    CHECK(!elems.empty());
    double sum = 0;
    for (const auto el : elems) {
        sum += static_cast<double>(el);
    }
    return sum / elems.size();
}

template <typename T> double variance(const std::vector<T> &elems) {
    const double avg = mean(elems);
    double var = 0;
    for (const auto el : elems) {
        const double diff = el - avg;
        var += diff * diff;
    }
    return var / (elems.size() - 1);
}

template <typename T> double stddev(const std::vector<T> &elems) { return std::sqrt(variance(elems)); }

template <typename T> double rmse(const std::vector<T> &elems) {
    double sum = 0.0;
    // sqrt(sum(i^2)/n)
    for (const auto &t : elems) {
        sum += t * t;
    }
    sum /= elems.size();
    return std::sqrt(sum);
}

template <typename T> bool any_less_than(std::vector<T> elems, T threshold) {
    for (const auto &el : elems) {
        if (el < threshold) {
            return true;
        }
    }
    return false;
}

template <typename T> bool any_greater_than(std::vector<T> elems, T threshold) {
    for (const auto &el : elems) {
        if (el > threshold) {
            return true;
        }
    }
    return false;
}

template <class Iterator> bool next_combination(Iterator first, Iterator middle, Iterator last) {
    return internal::next_combination(first, middle, middle, last);
}

template <typename T> T sigmoid(const T x, const T alpha) { return T(1) / (T(1) + exp(-x * alpha)); }

template <typename T> T scale_sigmoid(T x, const T alpha, const T x0) {
    const T t0 = sigmoid(-x0, alpha);
    const T t1 = sigmoid(x0, alpha);
    x = (sigmoid(2 * x0 * x - x0, alpha) - t0) / (t1 - t0);
    return x;
}

template <typename T1, typename T2> T2 truncate_cast(const T1 value) {
    return std::min(static_cast<T1>(std::numeric_limits<T2>::max()),
                    std::max(static_cast<T1>(std::numeric_limits<T2>::min()), value));
}
} // namespace h2o

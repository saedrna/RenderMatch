/*!
 * \file common.h
 *
 * \author Han
 * \date 2017/03/27
 *
 *
 */
#pragma once

#include <algorithm>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

using VALUE = uint64_t;
using iid_t = uint32_t;
using fid_t = uint32_t;
using oid_t = uint32_t;

const uint32_t UNDEFINED_ID = -1;

template <class T> inline void hash_combine(std::size_t &seed, const T &v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
// 实例化 pair 结构体的 hash 函数，用于作为 unordered_map 的键值
template <typename S, typename T> struct hash<pair<S, T>> {
    inline size_t operator()(const pair<S, T> &v) const {
        size_t seed = 0;
        ::hash_combine(seed, v.first);
        ::hash_combine(seed, v.second);
        return seed;
    }
};

template <typename ContainerT, typename PredicateT> void erase_if(ContainerT &items, const PredicateT &predicate) {
    for (auto it = items.begin(); it != items.end();) {
        if (predicate(*it))
            it = items.erase(it);
        else
            ++it;
    }
};
} // namespace std

// 扩充一些额外的类型定义, uint8_t - b, uint16_t - w, uint32_t - u
namespace Eigen {

/** \defgroup matrixtypedefs Global matrix typedefs
 *
 * \ingroup Core_Module
 *
 * Eigen defines several typedef shortcuts for most common matrix and vector types.
 *
 * The general patterns are the following:
 *
 * \c MatrixSizeType where \c Size can be \c 2,\c 3,\c 4 for fixed size square matrices or \c X for dynamic size,
 * and where \c Type can be \c i for integer, \c f for float, \c d for double, \c cf for complex float, \c cd
 * for complex double.
 *
 * For example, \c Matrix3d is a fixed-size 3x3 matrix type of doubles, and \c MatrixXf is a dynamic-size matrix of
 * floats.
 *
 * There are also \c VectorSizeType and \c RowVectorSizeType which are self-explanatory. For example, \c Vector4cf is
 * a fixed-size vector of 4 complex floats.
 *
 * \sa class Matrix
 */

#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)                                                        \
    /** \ingroup matrixtypedefs */                                                                                     \
    typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix;                                                   \
    /** \ingroup matrixtypedefs */                                                                                     \
    typedef Matrix<Type, Size, 1> Vector##SizeSuffix##TypeSuffix;                                                      \
    /** \ingroup matrixtypedefs */                                                                                     \
    typedef Matrix<Type, 1, Size> RowVector##SizeSuffix##TypeSuffix;

#define EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, Size)                                                              \
    /** \ingroup matrixtypedefs */                                                                                     \
    typedef Matrix<Type, Size, Dynamic> Matrix##Size##X##TypeSuffix;                                                   \
    /** \ingroup matrixtypedefs */                                                                                     \
    typedef Matrix<Type, Dynamic, Size> Matrix##X##Size##TypeSuffix;

#define EIGEN_MAKE_TYPEDEFS_ALL_SIZES(Type, TypeSuffix)                                                                \
    EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 2, 2)                                                                        \
    EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 3, 3)                                                                        \
    EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, 4, 4)                                                                        \
    EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Dynamic, X)                                                                  \
    EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 2)                                                                     \
    EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 3)                                                                     \
    EIGEN_MAKE_FIXED_TYPEDEFS(Type, TypeSuffix, 4)

EIGEN_MAKE_TYPEDEFS_ALL_SIZES(uint8_t, b)
EIGEN_MAKE_TYPEDEFS_ALL_SIZES(uint16_t, w)
EIGEN_MAKE_TYPEDEFS_ALL_SIZES(uint32_t, u)

#undef EIGEN_MAKE_TYPEDEFS_ALL_SIZES
#undef EIGEN_MAKE_TYPEDEFS
#undef EIGEN_MAKE_FIXED_TYPEDEFS
} // namespace Eigen

// 所有记录的都是Utf-8的字符串
namespace h2o {

using namespace Eigen;

// 包含 eigen 的基本结构
typedef Eigen::Matrix<double, 3, 4> Matrix34d;
typedef Eigen::Matrix<double, 2, 3> Matrix23d;

typedef Eigen::AlignedBox2i BoundingBox2i;
typedef Eigen::AlignedBox2d BoundingBox2d;
typedef Eigen::AlignedBox2f BoundingBox2f;
typedef Eigen::AlignedBox3d BoundingBox3d;
typedef Eigen::AlignedBox3f BoundingBox3f;
typedef Eigen::Hyperplane<double, 3> Plane3d;
typedef Eigen::Hyperplane<double, 2> Line2d;
typedef Eigen::Transform<double, 3, Eigen::Affine, Eigen::ColMajor> Transform3d;

// SU 的一些常亮信息
constexpr double DEG2RAD = 0.01745329251994329576923690768489;
constexpr double RAD2DEG = 57.295779513082320876798154814105;
const double METER_2_INCH = 39.37007874015748031496062992126; // 2.54 cm
const double INCH_2_METER = 0.025400000000000000000000000000;
const double RAD_2_DEGREE = 57.295779513082320876798154814105;
const double DEGREE_2_RAD = 0.01745329251994329576923690768489;

const uint32_t INVALID_INDEX = -1;

struct hash_pair {
public:
    template <typename T, typename U> std::size_t operator()(const std::pair<T, U> &x) const {
        return std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
    }
};

typedef uint32_t GLuint;
typedef int GLint;

} // namespace h2o

namespace std {
template <> struct hash<h2o::Vector2i> {
    std::size_t operator()(const h2o::Vector2i &k) const {
        return ((std::hash<int>()(k(0)) ^ (std::hash<int>()(k(1)) << 1)) >> 1);
    }
};
} // namespace std

namespace hash_eigen {
template <typename T> struct hash {
    std::size_t operator()(T const &matrix) const {
        size_t seed = 0;
        for (int i = 0; i < (int)matrix.size(); i++) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
} // namespace hash_eigen

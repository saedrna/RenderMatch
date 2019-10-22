/*
 * @Author: Han Hu
 * @Date: 2018-08-03 16:29:14
 * 封装 MathGeoLib 的 OBB 类
 */
#pragma once
#include <base/common.h>

namespace h2o {
// 和 MathGeoLib 交换的数据结构
struct OBB3d {
    Vector3d center = Vector3d::Constant(DBL_MAX);
    Vector3d r = Vector3d::Constant(-1.0);
    std::array<Vector3d, 3> axis;
};

// 计算点云的obb
OBB3d obb_compute(const std::vector<Vector3d> &points);

// mathgeolib 的 计算 convex hull 的有时候会出错，用这个方法更可靠
OBB3d obb_compute(const std::vector<Vector3d> &points, const Vector3d &dir1, const Vector3d &dir2);

OBB3d obb_compute(const BoundingBox3d &aabb);

// 计算 SU face 的 obb
OBB3d obb_compute(VALUE face, double thickness);

// 绕中心缩放
OBB3d obb_scale(const OBB3d &obb, double scale);
OBB3d obb_scale(const OBB3d &obb, const Vector3d &scale);

// 绕某一点缩放
OBB3d obb_scale(const OBB3d &obb, double scale, const Vector3d &center);
OBB3d obb_scale(const OBB3d &obb, const Vector3d &scale, const Vector3d &center);

double obb_volume(const OBB3d &obb);

// 根据包围盒计算12个三角形（6）个面
MatrixXd obb_triangulate(const OBB3d &obb, MatrixXd *normal = nullptr);

MatrixXd obb_edges(const OBB3d &obb);
} // namespace h2o

/*!
 * \file plane.h
 *
 * \author Han
 * \date 2017/04/24
 *
 * 平面操作，主要包括坐标的三维和二维变换等
 */
#pragma once
#include <base/common.h>
#include <modelpro/obb.h>

namespace h2o {
class Plane {
public:
    /**
     * \brief 采用平面一点和法向量构建平面
     * \param p 平面上的一个点，之所以需要定义这个点，是为了将他设为中心，算出3D-2D之间的变换
     * \param n 平面法向量，需要归一化
     */
    Plane(const Vector3d &p, const Vector3d &n);
    Plane() {}
    ~Plane();

    /**
     * \brief 将三维点，投影到平面上，获取平面的二维坐标
     */
    virtual Vector2d to_2d(const Vector3d &point) const;

    /**
     * \brief 从平面上的二维坐标，反算对应的三维空间坐标
     */
    virtual Vector3d to_3d(const Vector2d &point) const;

    /**
     * \brief 点到平面距离，顾及正负号
     */
    virtual double signed_distance(const Vector3d &point) const;

    /**
     * \brief 点是否在平面上，epsilon 是 sketchup 默认值
     */
    virtual bool on_plane(const Vector3d &point) const;

    Vector3d get_normal() const;

protected:
    Plane3d plane_;
    ///\brief xy = R_ * xyz + T
    Matrix3d R_;
    Vector3d T_;
    Matrix3d Rinv_;
};

class PlaneRotated : public Plane {
public:
    /**
     * \brief 旋转平面，在计算从三维到二维的旋转矩阵的时候，会考虑二维平面，根据三维的 minAreaRect 的轴和X Y 平行
     */
    PlaneRotated(const std::vector<Vector3d> &points3d, const Vector3d &n);
    PlaneRotated() : Plane() {}
    ~PlaneRotated() {}

    void set_gsd(double gsd);
    double get_gsd() const { return 1.0 / io_(0, 0); }

    ///\brief 获取平面上二维bbox的四个角点
    std::vector<Vector2d> get_corners2d() const;
    ///\brief 平面四个bbox在三维空间的坐标
    std::vector<Vector3d> get_corners3d() const;

    ///\brief 平面上的二维坐标的bbox
    const BoundingBox2d &get_bbox() const { return bb_plane2d_; }

    // 必须要设置gsd 大小，在二维平面坐标直接根据 gsd 计算纹理的像素坐标
    Vector2d to_texture(const Vector3d &point) const;
    ///\brief 从纹理 uv 坐标到 三维坐标
    Vector3d from_texture(const Vector2d &point) const;

    // 获取平面的 OBB，因为是一个平面，所以需要设置一个 thickness
    OBB3d get_obb(double thickness);

protected:
    ///\brief 小于 -1 等于 0 大于 1
    int compare_z(const Vector3d &p1, const Vector3d &p2) const;
    void init_plane_bb();

protected:
    BoundingBox2d bb_plane2d_;

    ///\brief texture = io*plane2d
    Matrix23d io_;
    ///\broef plane2d = iob*texture
    Matrix23d iob_;
    ///\brief 所有点的 minrectarea 的 bb 在三维坐标空间中的坐标
    std::vector<Vector3d> bb_3d_;
    ///\brief 所有点的 minrectarea 的 bb 在二维平面上的坐标
    std::vector<Vector2d> bb_2d_;
};

// 计算从影像坐标到直接坐标的单映变换 H, obj = H * image
Matrix3d estimate_homography(const std::vector<Vector2d> &object, const std::vector<Vector2d> &image);

// 计算根据四个点估算平均的地面分辨率 gsd, 1 pixel = 1 gsd
double estimate_gsd(const std::vector<Vector2d> &object, const std::vector<Vector2d> &image);

PlaneRotated plane_from_face(VALUE face);
} // namespace h2o

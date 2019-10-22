/*!
 * \file plane.cpp
 *
 * \author Han
 * \date 2017/06/21
 *
 * 平面操作，主要包括坐标的三维和二维变换等
 */
#include <MathGeoLib/MathGeoLib.h>

#include <modelpro/mesh.h>
#include <modelpro/plane.h>
#include <sketchup/sketchup.h>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace h2o {
Plane::Plane(const Vector3d &p, const Vector3d &n) {
    Vector3d normal = n;
    normal.normalize();
    Vector3d z_axis = {0.0, 0.0, 1.0};

    // 平面二三维变换的实质是一个旋转加平移
    R_ = Quaterniond::FromTwoVectors(normal, z_axis);
    T_ = -R_ * p;
    Rinv_ = R_.inverse();

    plane_ = Plane3d(n, p);
}

Plane::~Plane() {}

Vector2d Plane::to_2d(const Vector3d &point) const {
    Vector3d plane = R_ * point + T_;
    return plane.head(2);
}

Vector3d Plane::to_3d(const Vector2d &point) const {
    Vector3d point3d = {point.x(), point.y(), 0.0};
    Vector3d object3d = Rinv_ * (point3d - T_);
    return object3d;
}

double Plane::signed_distance(const Vector3d &point) const { return plane_.signedDistance(point); }

bool Plane::on_plane(const Vector3d &point) const {
    double distance = signed_distance(point);
    // EPSILON_LENGTH 的精度是 0.001
    if (distance < EPSILON_LENGTH && distance > -EPSILON_LENGTH) {
        return true;
    } else {
        return false;
    }
}

Vector3d Plane::get_normal() const { return plane_.normal(); }

PlaneRotated::PlaneRotated(const std::vector<Vector3d> &points3d, const Vector3d &n) : Plane() {
    CHECK(points3d.size() >= 3);

    Vector3d normal = n;
    normal.normalize();
    Vector3d z_axis(0.0, 0.0, 1.0);
    Vector3d y_axis(0.0, 1.0, 0.0);
    // 平面二三维变换的实质是一个旋转加平移
    R_ = Quaterniond::FromTwoVectors(normal, z_axis);

    T_ = -R_ * points3d[0];
    Rinv_ = R_.inverse();

    plane_ = Plane3d(n, points3d[0]);
    // 将所有点变换到二维，计算 rotated rect
    std::vector<cv::Point2f> plane2d(points3d.size());
    std::transform(begin(points3d), end(points3d), begin(plane2d), [&](const Vector3d &p) {
        Vector3d plane3d = R_ * p + T_;
        CHECK(std::abs(plane3d.z()) < 0.1)
            << "Project to 2d plane failed, the z coord of the point is : " << plane3d.z();
        return cv::Point2f(plane3d.x(), plane3d.y());
    });

    /*
     * OPENCV 的 rotated rect 的四个角点分别为
     * point = center + R * corner
     * R 为 Rotation2d 得到的
     * 0 ----- 3
     * |       |
     * 1 ----- 2
     */
    cv::RotatedRect rect = cv::minAreaRect(plane2d);

    Matrix3d R_z;
    Vector3d T2;
    Vector3d direction_y;

    std::vector<cv::Point2f> points_rect(4);
    std::vector<Vector3d> bb_3d(4);
    rect.points(points_rect.data());

    std::transform(begin(points_rect), end(points_rect), begin(bb_3d), [&](const cv::Point2f &p) {
        Vector3d plane3d(p.x, p.y, 0.0);
        Vector3d object3d = Rinv_ * (plane3d - T_);
        return object3d;
    });

    // 绕z轴再旋转一定量，将二维平面的xy轴和他对其
    // 这个 旋转的大小的计算是两种
    if (std::abs(std::abs(normal.dot(z_axis)) - 1.0) < 1e-6) {
        direction_y = Vector3d(points_rect[0].x - points_rect[1].x, points_rect[0].y - points_rect[1].y, 0.0);
        R_z = Quaterniond::FromTwoVectors(direction_y, y_axis);
        T2 = -R_z * Vector3d(points_rect[1].x, points_rect[1].y, 0.0);
    } else {
        // 需要保证 y 轴是朝上的，将某个朝上的轴，旋转到y轴
        if (compare_z(bb_3d[0], bb_3d[1]) == 1) {
            direction_y = Vector3d(points_rect[0].x - points_rect[1].x, points_rect[0].y - points_rect[1].y, 0.0);
            R_z = Quaterniond::FromTwoVectors(direction_y, y_axis);
            T2 = -R_z * Vector3d(points_rect[1].x, points_rect[1].y, 0.0);
        } else if (compare_z(bb_3d[0], bb_3d[1]) == -1) {
            direction_y = Vector3d(points_rect[1].x - points_rect[0].x, points_rect[1].y - points_rect[0].y, 0.0);
            R_z = Quaterniond::FromTwoVectors(direction_y, y_axis);
            T2 = -R_z * Vector3d(points_rect[3].x, points_rect[3].y, 0.0);
        }
        if (compare_z(bb_3d[0], bb_3d[3]) == 1) {
            direction_y = Vector3d(points_rect[0].x - points_rect[3].x, points_rect[0].y - points_rect[3].y, 0.0);
            R_z = Quaterniond::FromTwoVectors(direction_y, y_axis);
            T2 = -R_z * Vector3d(points_rect[2].x, points_rect[2].y, 0.0);
        } else if (compare_z(bb_3d[0], bb_3d[3]) == -1) {
            direction_y = Vector3d(points_rect[3].x - points_rect[0].x, points_rect[3].y - points_rect[0].y, 0.0);
            R_z = Quaterniond::FromTwoVectors(direction_y, y_axis);
            T2 = -R_z * Vector3d(points_rect[0].x, points_rect[0].y, 0.0);
        }
    }
    // plane3d = R_p + T
    // texture = Rz * plane3d + T2, T2 用于将点 0 的坐标移动到 0
    // texture = Rz * R_ * p + Rz * T + T2
    T_ = R_z * T_ + T2;
    R_ = R_z * R_;
    Rinv_ = R_.inverse();
    bb_3d_ = bb_3d;

    init_plane_bb();
}

void PlaneRotated::set_gsd(double gsd) {
    // 左上角的纹理影像坐标为 0，0，但是他的plane2d的坐标是最大的
    io_(0, 0) = 1.0 / gsd;
    io_(0, 1) = 0.0;
    io_(0, 2) = 0.0;
    io_(1, 0) = 0.0;
    io_(1, 1) = -1.0 / gsd;
    io_(1, 2) = bb_plane2d_.sizes()(1) / gsd;

    Matrix2d A;
    Vector2d b;
    A = io_.block(0, 0, 2, 2);
    b = io_.col(2);

    iob_.block(0, 0, 2, 2) = A.inverse();
    iob_.col(2) = -A.inverse() * b;
}

std::vector<Vector2d> PlaneRotated::get_corners2d() const {
    std::vector<Vector2d> bb_plane2d(4);
    std::transform(begin(bb_3d_), end(bb_3d_), begin(bb_plane2d), [&](const Vector3d &p) { return to_2d(p); });
    return bb_plane2d;
}

std::vector<Vector3d> PlaneRotated::get_corners3d() const { return bb_3d_; }

Vector2d PlaneRotated::to_texture(const Vector3d &point) const {
    Vector2d plane2d = to_2d(point);
    Vector2d tex2d = io_ * plane2d.homogeneous();
    return tex2d;
}

Vector3d PlaneRotated::from_texture(const Vector2d &point) const {
    Vector2d plane2d = iob_ * point.homogeneous();
    Vector3d object3d = to_3d(plane2d);
    return object3d;
}

OBB3d PlaneRotated::get_obb(double thickness) {
    Vector3d normal = plane_.normal();

    std::vector<Vector3d> corners3d = get_corners3d();
    std::vector<Vector3d> points(corners3d.size() * 2);
    for (int i = 0; i < corners3d.size(); ++i) {
        Vector3d point = corners3d[i] + thickness * normal;
        points[2 * i + 0] = point;

        point = corners3d[i] - thickness * normal;
        points[2 * i + 1] = point;
    }

    Vector3d dir1 = (corners3d[1] - corners3d[0]).normalized();
    Vector3d dir2 = (corners3d[3] - corners3d[0]).normalized();

    OBB3d obb = obb_compute(points, dir1, dir2);

    return obb;
}

int PlaneRotated::compare_z(const Vector3d &p1, const Vector3d &p2) const {
    if (p1.z() - p2.z() > EPSILON_LENGTH) {
        return 1;
    } else if (p1.z() - p2.z() < -EPSILON_LENGTH) {
        return -1;
    } else {
        return 0;
    }
    return 0;
}

void PlaneRotated::init_plane_bb() {
    // 把所有的 bb_3d 换算到平面坐标
    for (const auto &corner : bb_3d_) {
        bb_plane2d_.extend(to_2d(corner));
    }
}

Matrix3d estimate_homography(const std::vector<Vector2d> &object, const std::vector<Vector2d> &image) {
    std::vector<cv::Point2f> object2(object.size()), image2(image.size());
    for (int i = 0; i < object.size(); ++i) {
        object2[i] = cv::Point2f(object[i].x(), object[i].y());
        image2[i] = cv::Point2f(image[i].x(), image[i].y());
    }

    cv::Mat mask;
    cv::Mat H2 = cv::findHomography(image2, object2, mask, 0);
    Matrix3d H;
    cv::cv2eigen(H2, H);

    return H;
}

double estimate_gsd(const std::vector<Vector2d> &object, const std::vector<Vector2d> &image) {
    double gsd = 0.0;
    for (int i = 0; i < image.size(); ++i) {

        int j = (i + 1) % image.size();

        Vector2d o1 = object[i];
        Vector2d o2 = object[j];

        Vector2d i1 = image[i];
        Vector2d i2 = image[j];

        gsd += (o1 - o2).norm() / (i1 - i2).norm();
    }
    gsd /= image.size();
    return gsd;
}

PlaneRotated plane_from_face(VALUE face) {
    Vector3d normal = su_face_normal(face);
    std::vector<Vector3d> points = su_entity_vertices(face);
    PlaneRotated plane(points, normal);

    return plane;
}

} // namespace h2o

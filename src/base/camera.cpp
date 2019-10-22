/*
 * @Author: Han
 * @Date: 2018-05-05 20:49:29
 * 相机信息
 */
#include <base/camera.h>

#include <Eigen/Geometry>

namespace h2o {
FrameCamera::FrameCamera() {
    cid_ = INVALID_INDEX;
    iid_ = INVALID_INDEX;
    size_ = Vector2i(-1, -1);
    K = Matrix3d::Zero();
    R = Matrix3d::Zero();
    C = Vector3d::Zero();
    P = Matrix34d::Zero();
}
FrameCamera::FrameCamera(uint32_t cid, uint32_t iid, int width, int height, double delta, double f, double x0,
                         double y0, const Vector3d &r, const Vector3d &X) {
    cid_ = cid;
    iid_ = iid;
    size_(0) = width;
    size_(1) = height;

    K(0, 0) = f / delta;
    K(1, 1) = f / delta;
    double u0 = (width - 1) / 2.0 + x0 / delta;
    double v0 = (height - 1) / 2.0 - y0 / delta;

    K(0, 1) = 0.0;
    K(0, 2) = u0;

    K(1, 0) = 0.0;
    K(1, 2) = v0;

    K(2, 0) = 0.0;
    K(2, 1) = 0.0;
    K(2, 2) = 1.0;

    double angle = r.norm();
    Vector3d axis = r.normalized();
    Matrix3d Rpg = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    Matrix3d R_x = Eigen::AngleAxisd(M_PI, Vector3d::UnitX()).toRotationMatrix();
    R = Rpg * R_x;
    R.transposeInPlace();

    C = X;
    Vector3d t = -R * C;
    P << R, t;
    P = K * P;
}
FrameCamera::~FrameCamera() {}
} // namespace h2o

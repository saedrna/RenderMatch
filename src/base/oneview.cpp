/*
 * @Author: Han
 * @Date: 2018-05-05 20:50:02
 * 影像单视几何操作
 */
#include <base/oneview.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
namespace h2o {
std::tuple<cv::Mat, cv::Mat> get_distortion_mappings(const FrameCamera &camera) {
    // 返回 mapx mapy
    cv::Mat mapx, mapy;
    cv::Mat old_K;
    cv::Mat new_K;
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);

    cv::eigen2cv(camera.K, old_K);
    new_K = old_K.clone();
    ((double *)new_K.data)[2] = (camera.size_(0) - 1) / 2.0;
    ((double *)new_K.data)[5] = (camera.size_(1) - 1) / 2.0;

    cv::initUndistortRectifyMap(old_K, camera.distortion_, R, new_K, {camera.size_(0), camera.size_(1)}, CV_16SC2, mapx,
                                mapy);
    return std::make_tuple(mapx, mapy);
}
Vector2d project(const FrameCamera &camera, const Vector3d &point) {
    return (camera.P * point.homogeneous()).eval().hnormalized();
}
std::vector<Vector2d> project(const FrameCamera &camera, const std::vector<Vector3d> &points) {
    std::vector<Vector2d> pixels(points.size());
    for (int i = 0; i < points.size(); ++i) {
        pixels[i] = project(camera, points[i]);
    }
    return pixels;
}
Vector2d project(const Matrix34d &P, const Vector3d &point) { return (P * point.homogeneous()).eval().hnormalized(); }
std::vector<Vector2d> project(const Matrix34d &P, const std::vector<Vector3d> &points) {
    std::vector<Vector2d> pixels(points.size());
    for (int i = 0; i < points.size(); ++i) {
        pixels[i] = project(P, points[i]);
    }
    return pixels;
}
Vector3d get_direction(const FrameCamera &camera) {
    // XC = R(X - XS)，相机的朝向就是相机坐标系的 0 0 1 在世界坐标系的方向
    Vector3d z_axis = {0.0, 0.0, 1.0};
    Vector3d dir = camera.R.transpose() * z_axis;
    dir.normalize();
    return dir;
}
Vector3d project_to_zplane(const FrameCamera &camera, const Vector2d &point, double z) { return Vector3d(); }
} // namespace h2o

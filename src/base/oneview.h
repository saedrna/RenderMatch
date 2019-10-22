/*
 * @Author: Han
 * @Date: 2018-05-05 20:49:50
 * 影像单视几何操作
 */
#pragma once

#include <base/camera.h>

namespace h2o {

std::tuple<cv::Mat, cv::Mat> get_distortion_mappings(const FrameCamera &camera);
Vector2d project(const FrameCamera &camera, const Vector3d &point);
std::vector<Vector2d> project(const FrameCamera &camera, const std::vector<Vector3d> &points);
Vector2d project(const Matrix34d &P, const Vector3d &point);
std::vector<Vector2d> project(const Matrix34d &P, const std::vector<Vector3d> &points);
Vector3d get_direction(const FrameCamera &camera);
Vector3d project_to_zplane(const FrameCamera &camera, const Vector2d &point, double z);

} // namespace h2o

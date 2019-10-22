/*
 * @Author: Han Hu
 * @Date: 2018-09-01 19:25:04
 * 仅顾及角度和面积的纹理映射
 */

#include <modelpro/tex_simple.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace h2o {

TexInformation TexSimple::compute(iid_t best_view) {
    if (best_view == INVALID_INDEX) {
        // 首先计算最佳影像
        std::unordered_map<uint32_t, std::tuple<double, double>> view_info;

        // view_id => [投影到影像的面积，影像视角和平面夹角]
        std::unordered_map<uint32_t, double> view_area, view_angle;

        view_angle = estimate_angle();
        view_area = estimate_area();

        for (const auto &area : view_area) {
            // view info 只记录在 area 和 angle 都获得了值的影像
            if (view_angle.count(area.first)) {
                view_info[area.first] = std::make_tuple(area.second, view_angle.at(area.first));
            }
        }

        if (!view_info.size()) {
            return TexInformation();
        }

        // 归一化值, 0 - area ; 1- angle
        BoundingBox2d minmax;
        for (const auto &info : view_info) {
            double area, angle;
            std::tie(area, angle) = info.second;
            minmax.extend(Vector2d(area, angle));
        }

        // 计算score
        std::vector<std::pair<uint32_t, double>> score;
        for (const auto &info : view_info) {
            double area, angle;
            std::tie(area, angle) = info.second;
            // area 是越大 socre 越高
            area = (area - minmax.min()(0)) / (minmax.max()(0) - minmax.min()(0));
            // angle 是越小score 越高
            angle = (minmax.max()(1) - angle) / (minmax.max()(1) - minmax.min()(1));

            double score_weighted = (1.0 - weight_angle_) * area + weight_angle_ * angle;
            score.emplace_back(info.first, score_weighted);
        }

        std::sort(begin(score), end(score), [&](const auto &p1, const auto &p2) { return p1.second > p2.second; });
        best_view = score[0].first;
    }

    if (cameras_->count(best_view) == 0 || disk_images_->count(best_view) == 0) {
        return TexInformation();
    }

    // 纹理映射
    {
        FrameCamera *camera = cameras_->at(best_view).get();
        DiskImage *image = disk_images_->at(best_view).get();

        PlaneRotated plane = plane_;

        std::vector<Vector3d> corners3d = plane.get_corners3d();
        std::vector<Vector2d> corners2d = plane.get_corners2d();
        // 把包围和的点投影到影像上
        std::vector<Vector2d> image2d = project(*camera, corners3d);
        double gsd = estimate_gsd(corners2d, image2d);
        plane.set_gsd(gsd);

        // 获取纹理影像坐标和纹理

        std::vector<Vector2d> mat_points;
        cv::Mat mat;
        std::tie(mat_points, mat) = subset_texture_by_plane(plane, camera, image);
        if (mat_points.empty()) {
            return TexInformation();
        }

        // 纹理映射之前，需要把坐标变回局部坐标
        Matrix4d face_xform = xform_.inverse().eval();
        for (Vector3d &point : corners3d) {
            point = (face_xform * point.homogeneous()).hnormalized();
        }

        TexInformation info;
        info.mat = mat;
        info.mat_points = mat_points;
        info.obj_points = corners3d;
        info.view_id_ = best_view;
        return info;
    }
}

std::unordered_map<iid_t, double> TexSimple::estimate_angle() {
    std::unordered_map<uint32_t, double> view_angle;
    Vector3d normal = plane_.get_normal();

    std::vector<Vector3d> object3d = plane_.get_corners3d();
    Vector3d center(0.0, 0.0, 0.0);
    for (const auto &p : object3d) {
        center += p;
    }
    center /= object3d.size();
    normal.normalize();

    for (const auto &view : *cameras_) {
        Vector3d C = view.second->C;
        Vector3d dir = C - center;
        dir.normalize();
        double angle = std::acos(dir.dot(normal)) * RAD_2_DEGREE;
        if (std::abs(angle) > max_angle_) {
            continue;
        }

        view_angle[view.first] = angle;
    }

    return view_angle;
}

std::unordered_map<iid_t, double> TexSimple::estimate_area() {
    std::unordered_map<uint32_t, double> view_area;
    std::vector<Vector3d> object3d = plane_.get_corners3d();

    for (const auto &view : *cameras_) {
        BoundingBox2i bbox_bound;
        const FrameCamera *camera = view.second.get();

        bbox_bound.extend(Vector2i(0 + black_border_, 0 + black_border_));
        bbox_bound.extend(Vector2i(camera->size_(0) - black_border_, camera->size_(1) - black_border_));

        std::vector<Vector2d> image2d = project(*camera, object3d);

        bool is_in_image = std::all_of(begin(image2d), end(image2d),
                                       [&](const auto &pixel) { return bbox_bound.contains(pixel.cast<int>()); });
        if (!is_in_image) {
            continue;
        }

        std::vector<cv::Point2f> cv_points(image2d.size());
        std::transform(begin(image2d), end(image2d), begin(cv_points),
                       [&](const auto &pixel) { return cv::Point2f(pixel.x(), pixel.y()); });

        cv::RotatedRect rotate_rect = cv::minAreaRect(cv_points);
        if (rotate_rect.size.area() <= 0.0) {
            continue;
        }

        // area 是长度的平方，这里求area的平方根，是为了只用他的长度单位计算权重
        view_area[view.first] = sqrt(rotate_rect.size.area());
    }

    return view_area;
}

std::tuple<std::vector<Eigen::Vector2d>, cv::Mat>
TexSimple::subset_texture_by_plane(const PlaneRotated &plane, const FrameCamera *camera, DiskImage *image) {
    auto points3d = plane.get_corners3d();
    auto image2d = project(*camera, points3d);

    BoundingBox2i image_roi;
    BoundingBox2i image_size;

    for (const auto &point : image2d) {
        Vector2i pointi(point.x() + 0.5, point.y() + 0.5);
        image_roi.extend(pointi);
    }
    image_size.extend(Vector2i(0, 0));
    image_size.extend(camera->size_);

    image_roi = image_size.intersection(image_roi);
    if (image_roi.isEmpty() || image_roi.sizes()(0) == 0 || image_roi.sizes()(1) == 0) {
        return std::tuple<std::vector<Vector2d>, cv::Mat>();
    }

    Matrix23d xform;
    cv::Mat mat;

    std::tie(xform, mat) = image->get_patch(image_roi, image_roi.sizes());
    {
        // 逆变换
        Matrix23d inverse;
        inverse.leftCols(2) = xform.leftCols(2).inverse();
        inverse.col(2) = -xform.leftCols(2).inverse() * xform.col(2);
        xform = inverse; // 此时 new = xform * old
    }
    // 把 image2d 变换到新的影像坐标下
    for (Vector2d &point : image2d) {
        point = xform * point.homogeneous();
    }

    // 纹理坐标的像素坐标
    std::vector<Vector2d> mat_points(image2d.size());
    for (int i = 0; i < mat_points.size(); ++i) {
        mat_points[i] = plane.to_texture(points3d[i]);
    }

    double gsd = plane.get_gsd();
    BoundingBox2d bounds = plane.get_bbox();
    int rows = bounds.sizes()(1) / gsd + 0.5;
    int cols = bounds.sizes()(0) / gsd + 0.5;
    if (rows == 0 || cols == 0) {
        return std::tuple<std::vector<Vector2d>, cv::Mat>();
    }

    // 估计仿射变换参数，对原始影像进行仿射变换
    cv::Mat H;
    {
        Matrix3d H2 = estimate_homography(mat_points, image2d);
        cv::eigen2cv(H2, H);
    }
    cv::warpPerspective(mat, mat, H, {cols, rows}, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

    return std::make_tuple(mat_points, mat);
}

} // namespace h2o

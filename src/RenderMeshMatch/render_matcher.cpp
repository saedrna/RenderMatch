/*
 * @Author: Han
 * @Date: 2019-11-15 20:32:43
 * Match aerial and ground images, with rendered images as delegates
 */
#include "render_matcher.h"

#include <match/sift_matcher.h>

#include <Eigen/Jacobi>
#include <fstream>
#include <nlohmann/json.hpp>

RenderMeshMatchConfig load_config(const std::string &path) {
    std::ifstream ifile(path);
    nlohmann::json j;
    ifile >> j;

    RenderMeshMatchConfig config;
    return config;
}

void RenderMatcher::set_ogl_matrices(const Matrix4f &view, const MatrixXf &proj) {
    mvp_inverse_ = (proj * view).inverse();
}

RenderMatchResult RenderMatcher::match(uint32_t iid, const cv::Mat &mat_rgb, const cv::Mat &mat_dep) {
    // load the sift descriptors of the ground image
    std::vector<cv::KeyPoint> keys_ground;
    cv::Mat desc_ground;

    {
        std::string path = block_ground_.photos.at(iid).path;
        std::string name = get_filename_noext(path);
        std::string dir = get_directory(path);
        path = join_paths(dir, name + ".sift");
        sift_read(path, keys_ground, desc_ground);
    }

    // extract sift from rendered images
    std::vector<cv::KeyPoint> keys_render;
    cv::Mat desc_render;
    {
        cv::Mat mat = mat_rgb.clone();
        cv::cvtColor(mat, mat, cv::COLOR_BGRA2GRAY);
        mat = image_percent_scale_8u(mat, mat_rgb != cv::Scalar(255, 255, 255, 0));
        std::tie(keys_render, desc_render) = sift_->detect_and_compute(mat);
    }

    // match sift
    SiftMatcherParam sift_param;
    SiftMatcher sift_matcher;
    sift_matcher.set_match_param(sift_param);
    sift_matcher.set_train_data(keys_ground, desc_ground);
    std::vector<cv::DMatch> matches = sift_matcher.match(keys_render, desc_render);

    // too few matches
    if (matches.size() < 10) {
        return RenderMatchResult();
    }

    // for each match expand to aerial views with patch match
    for (const auto &match : matches) {
        auto key_ground = keys_ground.at(match.trainIdx);
        auto key_render = keys_render.at(match.queryIdx);

        int r = int(key_render.pt.y + 0.5);
        int c = int(key_render.pt.x + 0.5);
    }

    return RenderMatchResult();
}

std::tuple<MatrixXf, Vector3f> RenderMatcher::get_patch_on_rendered_image(uint32_t iid, const Vector2i &pt,
                                                                          const cv::Mat &mat_dep) {

    int patch_size = (param_.ncc_window / 2 + param_.ncc_search / 2) * 2 + 1;
    int patch_half = patch_size / 2;

    int rows = mat_dep.rows;
    int cols = mat_dep.cols;

    BoundingBox2i bounds_image;
    bounds_image.extend(Vector2i(0, 0));
    bounds_image.extend(Vector2i(cols, rows));
    BoundingBox2i bounds_patch;
    bounds_patch.extend(Vector2i(pt.x() - patch_half, pt.y() - patch_half));
    bounds_patch.extend(Vector2i(pt.x() + patch_half + 1, pt.y() + patch_half + 1));

    if (!bounds_image.contains(bounds_patch)) {
        return std::make_tuple(MatrixXf(), Vector3f::Zero());
    }

    // compute the 3d point of the four corners and center
    MatrixXf corners(3, 4);
    std::vector<float> depths(4);
    for (int i = 0; i < 4; ++i) {
        Vector2i corner = bounds_patch.corner(BoundingBox2i::CornerType(i));
        float depth = mat_dep.at<float>(corner.y(), corner.x());
        if (std::abs(depth - 1.0) < 1e-4) {
            // invalid depth value
            return std::make_tuple(MatrixXf(), Vector3f::Zero());
        }

        corners.col(i) = depth_to_xyz(depth, corner);
    }

    // compute the eigen vectors
    Vector3f nor;
    {
        Vector3f mean = corners.rowwise().mean();
        MatrixXf centered = corners.colwise() - mean;
        Eigen::JacobiSVD<MatrixXf> jacobi(centered, Eigen::ComputeFullU | Eigen::ComputeFullV);
        nor = jacobi.matrixU().col(2);
    }

    // reoriente the normal
    {
        float depth = mat_dep.at<float>(pt.y(), pt.x());
        if (std::abs(depth - 1.0) < 1e-4) {
            // invalid depth value
            return std::make_tuple(MatrixXf(), Vector3f::Zero());
        }
        Vector3f point = depth_to_xyz(depth, pt);
        Vector3f C = block_ground_.photos.at(iid).C.cast<float>();

        if (nor.dot(C - point) < 0) {
            nor = -nor;
        }
    }

    return std::tie(corners, nor);
}

std::tuple<cv::Mat, Matrix3f> RenderMatcher::get_patch_on_aerial_image(uint32_t iid_ground, uint32_t iid_aerial,
                                                                       const MatrixXf &corners,
                                                                       const Vector3f &normal) {

    Photo photo_aerial = block_aerial_.photos.at(iid_aerial);
    BoundingBox2i bounds_image;
    bounds_image.extend(Vector2i(0, 0));
    bounds_image.extend(
        Vector2i(block_aerial_.groups.at(photo_aerial.cid).width, block_aerial_.groups.at(photo_aerial.cid).height));

    // project the four corners to the aerial images
    MatrixXf points2d(2, 4);
    BoundingBox2i bounds_patch;

    for (int i = 0; i < 4; ++i) {
        Vector3d point = corners.col(i).cast<double>();
        Vector2d point2d = block_aerial_.project(point, iid_aerial);

        points2d.col(i) = point2d;
        bounds_patch.extend(Vector2i(point2d.x() + 1.5, point2d.y() + 1.5));
        bounds_patch.extend(Vector2i(point2d.x() - 0.5, point2d.y() - 0.5));
    }

    if (!bounds_image.contains(bounds_patch)) {
        return std::tuple<cv::Mat, Matrix3f>();
    }

    images_aerial_.at(iid_aerial)->get_patch(bounds_image, bounds_image.sizes());

    return std::tuple<cv::Mat, Matrix3f>();
}

cv::Mat RenderMatcher::get_patch_on_ground_image(uint32_t iid, const Vector2i &point) {

    int patch_size = (param_.ncc_window / 2 + param_.ncc_search / 2) * 2 + 1;
    int patch_half = patch_size / 2;

    int rows = viewport_.y();
    int cols = viewport_.x();

    BoundingBox2i bounds_image;
    bounds_image.extend(Vector2i(0, 0));
    bounds_image.extend(Vector2i(cols, rows));
    BoundingBox2i bounds_patch;
    bounds_patch.extend(Vector2i(point.x() - patch_half, point.y() - patch_half));
    bounds_patch.extend(Vector2i(point.x() + patch_half + 1, point.y() + patch_half + 1));

    if (!bounds_image.contains(bounds_patch)) {
        return cv::Mat();
    }

    cv::Mat patch;
    Matrix23d H;

    std::tie(H, patch) = images_ground_.at(iid)->get_patch(bounds_patch, bounds_patch.sizes());

    return patch;
}

Vector3f RenderMatcher::depth_to_xyz(float depth, const Vector2i &point) {
    if (std::abs(depth - 1.0) < 1e-4) {
        return Vector3f::Constant(FLT_MAX);
    }

    Vector4f screen;
    screen(0) = (float)(point.x() + 0.5f) / viewport_.x();
    screen(1) = (float)(viewport_.y() - point.y() - 0.5) / viewport_.y();
    screen(2) = depth;
    screen(3) = 1.0;

    screen = screen.array() * 2.0f - 1.0f;
    Vector4f object = mvp_inverse_ * screen;
    Vector3f coord = object.hnormalized();

    return coord;
}

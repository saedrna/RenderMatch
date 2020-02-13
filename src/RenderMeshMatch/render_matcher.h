/*
 * @Author: Han
 * @Date: 2019-11-15 20:32:44
 * Match aerial and ground images, with rendered images as delegates
 */
#pragma once

#include <RenderMatch/block.h>
#include <base/base.h>
#include <match/sift_detector.h>

using namespace h2o;

struct RenderMeshMatchConfig {
    // use mesh for occlusion detection
    bool use_occlusion = false;

    // the angle difference between the patch and aerial view before testing possible match
    double angle_difference = 60.0;

    // ncc window for correlation, should be odd
    int ncc_window = 41;

    // additional search region for ncc, to locate the peak
    // final patch size is (window / 2 + search / 2) * 2 + 1
    int ncc_search = 21;

    // threshold of ncc for valid match
    double ncc_threshold = 0.6;

    // use lsm on the patch
    bool use_lsm = false;

};

RenderMeshMatchConfig load_config(const std::string &path);

struct RenderMatchResult {
    Vector3f xyz;

    uint32_t iid_ground;
    Vector2f pt_ground;

    std::vector<uint32_t> iid_aerial;
    std::vector<Vector2f> pt_aerial;
};
using RenderMatchResults = std::vector<RenderMatchResult>;

class RenderMatcher {

public:
    RenderMatcher();

public:
    void set_param(const RenderMeshMatchConfig &param);
    void set_block(const h2o::Block &aerial, const h2o::Block &ground);
    void set_ogl_matrices(const Matrix4f &view, const MatrixXf &proj);
    void save_match(RenderMatchResults matches_results);
    RenderMatchResults match(uint32_t iid, const cv::Mat &mat_rgb, const cv::Mat &mat_dep);

protected:
    // extract a patch from the rendered image, four corner points and a normal vector
    std::tuple<MatrixXf, Vector3f, Vector3f> get_patch_on_rendered_image(uint32_t iid, const Vector2i &pt,
                                                                         const cv::Mat &mat_dep);

    cv::Mat debug_patch_on_render_image(const cv::Mat &mat_rgb, const Vector2d &point);

    // extract a patch from the aerial image, with a homography matrix to determine the original coordinates
    // aerial = H * patch

    std::tuple<cv::Mat, Matrix3f> get_patch_on_aerial_image(uint32_t iid_ground, uint32_t iid_aerial,
                                                            const MatrixXf &corners, const Vector3f &normal);

    // return a patch on the ground image, which is the template
    // extract a patch from the ground image, assume that the deformation between ground and render image is removed
    // i.e. only translational difference

    cv::Mat get_patch_on_ground_image(uint32_t iid, const Vector2d &point);

    std::vector<uint32_t> search_visible_aerial_images(const MatrixXf &corners, const Vector3f &normal);

    Vector3f depth_to_xyz(float depth, const Vector2i &point);

    // inverse of the model view projection matrix, used to determin xyz from depth
    Matrix4f mvp_inverse_;
    Vector2i viewport_;

    std::map<uint32_t, DiskImagePtr> images_ground_;
    std::map<uint32_t, DiskImagePtr> images_aerial_;
    h2o::Block block_aerial_;
    h2o::Block block_ground_;

    std::shared_ptr<SiftDetector> sift_;

    RenderMeshMatchConfig param_;
};

/*
 * @Author: Han
 * @Date: 2019-11-15 20:32:43
 * Match aerial and ground images, with rendered images as delegates
 */
#include "render_matcher.h"

#include <match/lsm.h>
#include <match/sift_matcher.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

#include <Eigen/Jacobi>
#include <fstream>
#include <istream>
#include <nlohmann/json.hpp>
#include <random>

#include <rapidxml/rapidxml.hpp>
#include <rapidxml/rapidxml_print.hpp>

#include <QFile>
#include <QTextStream>

RenderMeshMatchConfig load_config(const std::string &path) {
    std::ifstream ifile(path);
    nlohmann::json j;
    ifile >> j;

    RenderMeshMatchConfig config;
    config.use_occlusion = j["UseOcclusion"];
    config.angle_difference = j["AngleDifference"];
    config.ncc_window = j["NccWindow"];
    config.ncc_search = j["NccSearch"];
    config.ncc_threshold = j["NccThreshold"];
    config.use_lsm = j["UseLsm"];
    config.num_propagation = j["NumPropagation"];
    return config;
}

RenderMatcher::RenderMatcher() { sift_ = std::make_shared<SiftDetector>(); }

void RenderMatcher::set_param(const RenderMeshMatchConfig &param) { param_ = param; }

void RenderMatcher::set_block(const h2o::Block &aerial, const h2o::Block &ground) {
    block_ground_ = ground;
    block_aerial_ = aerial;

    // initialize images
    for (const auto &pair : ground.photos) {
        DiskImagePtr image = std::make_shared<DiskImage>(pair.second.path, true);
        images_ground_.emplace(pair.first, image);
    }
    for (const auto &pair : aerial.photos) {
        DiskImagePtr image = std::make_shared<DiskImage>(pair.second.path, true);
        images_aerial_.emplace(pair.first, image);
    }
}

void RenderMatcher::set_ogl_matrices(const Matrix4f &view, const MatrixXf &proj) {
    mvp_inverse_ = (proj * view).inverse();
}

void RenderMatcher::save_block(const RenderMatchResults &matches, const std::string &block_merged,
                               const std::string &ofile) {
    h2o::Block merged = load_block_xml(block_merged);

    auto get_name = [](const std::string &path) {
        std::string name = path;
        name = get_filename_noext(name);
        name = string_to_lower(name);
        return name;
    };

    // name of image -> id in merged block, assume the name of image is different
    std::map<std::string, uint32_t> name_to_idx;
    std::map<std::string, std::string> name_to_path;

    for (const auto &pair : block_aerial_.photos) {
        std::string name = get_name(pair.second.path);

        uint32_t idx = INVALID_INDEX;
        std::string path;
        // search merged block
        for (const auto &pair2 : merged.photos) {
            std::string name2 = get_name(pair2.second.path);

            if (name == name2) {
                idx = pair2.second.id;
                path = pair2.second.path;
                break;
            }
        }

        name_to_idx.emplace(name, idx);
        name_to_path.emplace(name, path);
    }

    for (const auto &pair : block_ground_.photos) {
        std::string name = get_name(pair.second.path);

        uint32_t idx = INVALID_INDEX;
        std::string path;
        // search merged block
        for (const auto &pair2 : merged.photos) {
            std::string name2 = get_name(pair2.second.path);

            if (name == name2) {
                idx = pair2.second.id;
                path = pair2.second.path;
                break;
            }
        }

        name_to_idx.emplace(name, idx);
        name_to_path.emplace(name, path);
    }

    // export all survey data
    using document = rapidxml::xml_document<>;
    using node = rapidxml::xml_node<>;
    document doc;
    node *declaration = doc.allocate_node(rapidxml::node_declaration);
    doc.append_node(declaration);
    declaration->append_attribute(doc.allocate_attribute("version", "1.0"));
    declaration->append_attribute(doc.allocate_attribute("encoding", "utf-8"));

    node *survey_data = doc.allocate_node(rapidxml::node_element, "SurveysData");
    doc.append_node(survey_data);
    node *tie_points = doc.allocate_node(rapidxml::node_element, "TiePoints");
    survey_data->append_node(tie_points);

    uint32_t id = 0;
    for (const auto &match : matches) {
        node *tie_point = doc.allocate_node(rapidxml::node_element, "TiePoint");
        tie_points->append_node(tie_point);

        node *nid = doc.allocate_node(rapidxml::node_element, "Id", doc.allocate_string(std::to_string(id).c_str()));
        tie_point->append_node(nid);

        node *nname =
            doc.allocate_node(rapidxml::node_element, "Name", doc.allocate_string(std::to_string(id).c_str()));
        tie_point->append_node(nname);

        node *ntype = doc.allocate_node(rapidxml::node_element, "Type", "User");
        tie_point->append_node(ntype);

        auto add_measurment = [&doc, tie_point, &name_to_idx, &name_to_path](const std::string &name,
                                                                             const Vector2f &point) {
            uint32_t photoid = name_to_idx.at(name);
            std::string path = name_to_path.at(name);

            if (photoid == INVALID_INDEX) {
                return;
            }

            node *nmeasure = doc.allocate_node(rapidxml::node_element, "Measurement");
            tie_point->append_node(nmeasure);

            node *nphotoid = doc.allocate_node(rapidxml::node_element, "PhotoId",
                                               doc.allocate_string(std::to_string(photoid).c_str()));
            nmeasure->append_node(nphotoid);

            node *nimagepath =
                doc.allocate_node(rapidxml::node_element, "ImagePath", doc.allocate_string(path.c_str()));
            nmeasure->append_node(nimagepath);

            node *nx =
                doc.allocate_node(rapidxml::node_element, "x", doc.allocate_string(std::to_string(point.x()).c_str()));
            nmeasure->append_node(nx);

            node *ny =
                doc.allocate_node(rapidxml::node_element, "y", doc.allocate_string(std::to_string(point.y()).c_str()));
            nmeasure->append_node(ny);
        };

        add_measurment(get_name(block_ground_.photos.at(match.iid_ground).path), match.pt_ground);
        for (int i = 0; i < match.iid_grounds.size(); ++i) {
            uint32_t iid = match.iid_grounds.at(i);
            Vector2f pt = match.pt_grounds.at(i);
            add_measurment(get_name(block_ground_.photos.at(iid).path), pt);
        }
        for (int i = 0; i < match.iid_aerials.size(); ++i) {
            uint32_t iid = match.iid_aerials.at(i);
            Vector2f pt = match.pt_aerials.at(i);
            add_measurment(get_name(block_aerial_.photos.at(iid).path), pt);
        }

        id++;
    }

    std::ofstream output(ofile);
    output << doc;
}

void RenderMatcher::merge_block(const RenderMatchResults &matches, const std::string &block_merged,
                                const std::string &ofile) {
    h2o::Block merged = load_block_xml(block_merged);

    auto get_name = [](const std::string &path) {
        std::string name = path;
        name = get_filename_noext(name);
        name = string_to_lower(name);
        return name;
    };

    // name of image -> id in merged block, assume the name of image is different
    std::map<std::string, uint32_t> name_to_idx;
    std::map<std::string, std::string> name_to_path;

    for (const auto &pair : block_aerial_.photos) {
        std::string name = get_name(pair.second.path);

        uint32_t idx = INVALID_INDEX;
        std::string path;
        // search merged block
        for (const auto &pair2 : merged.photos) {
            std::string name2 = get_name(pair2.second.path);

            if (name == name2) {
                idx = pair2.second.id;
                path = pair2.second.path;
                break;
            }
        }

        name_to_idx.emplace(name, idx);
        name_to_path.emplace(name, path);
    }

    for (const auto &pair : block_ground_.photos) {
        std::string name = get_name(pair.second.path);

        uint32_t idx = INVALID_INDEX;
        std::string path;
        // search merged block
        for (const auto &pair2 : merged.photos) {
            std::string name2 = get_name(pair2.second.path);

            if (name == name2) {
                idx = pair2.second.id;
                path = pair2.second.path;
                break;
            }
        }

        name_to_idx.emplace(name, idx);
        name_to_path.emplace(name, path);
    }

    // export all survey data
    using document = rapidxml::xml_document<>;
    using node = rapidxml::xml_node<>;
    document doc;
    std::ifstream fmergeed(block_merged);
    std::vector<char> buffer((std::istreambuf_iterator<char>(fmergeed)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');
    doc.parse<0>(buffer.data());

    node *tie_points = doc.first_node("BlocksExchange")->first_node("Block")->first_node("TiePoints");

    uint32_t id = 0;
    for (const auto &match : matches) {
        node *tie_point = doc.allocate_node(rapidxml::node_element, "TiePoint");
        tie_points->append_node(tie_point);

        // node *nid = doc.allocate_node(rapidxml::node_element, "Id", doc.allocate_string(std::to_string(id).c_str()));
        // tie_point->append_node(nid);

        // node *nname =
        //    doc.allocate_node(rapidxml::node_element, "Name", doc.allocate_string(std::to_string(id).c_str()));
        // tie_point->append_node(nname);

        // node *ntype = doc.allocate_node(rapidxml::node_element, "Type", "User");
        // tie_point->append_node(ntype);

        node *nposition = doc.allocate_node(rapidxml::node_element, "Position");
        tie_point->append_node(nposition);

        node *nx =
            doc.allocate_node(rapidxml::node_element, "x", doc.allocate_string(std::to_string(match.xyz.x()).c_str()));
        node *ny =
            doc.allocate_node(rapidxml::node_element, "y", doc.allocate_string(std::to_string(match.xyz.y()).c_str()));
        node *nz =
            doc.allocate_node(rapidxml::node_element, "z", doc.allocate_string(std::to_string(match.xyz.z()).c_str()));
        nposition->append_node(nx);
        nposition->append_node(ny);
        nposition->append_node(nz);

        auto add_measurment = [&doc, tie_point, &name_to_idx, &name_to_path](const std::string &name,
                                                                             const Vector2f &point) {
            uint32_t photoid = name_to_idx.at(name);
            std::string path = name_to_path.at(name);

            if (photoid == INVALID_INDEX) {
                return;
            }

            node *nmeasure = doc.allocate_node(rapidxml::node_element, "Measurement");
            tie_point->append_node(nmeasure);

            node *nphotoid = doc.allocate_node(rapidxml::node_element, "PhotoId",
                                               doc.allocate_string(std::to_string(photoid).c_str()));
            nmeasure->append_node(nphotoid);

            node *nimagepath =
                doc.allocate_node(rapidxml::node_element, "ImagePath", doc.allocate_string(path.c_str()));
            nmeasure->append_node(nimagepath);

            node *nx =
                doc.allocate_node(rapidxml::node_element, "x", doc.allocate_string(std::to_string(point.x()).c_str()));
            nmeasure->append_node(nx);

            node *ny =
                doc.allocate_node(rapidxml::node_element, "y", doc.allocate_string(std::to_string(point.y()).c_str()));
            nmeasure->append_node(ny);
        };

        add_measurment(get_name(block_ground_.photos.at(match.iid_ground).path), match.pt_ground);
        for (int i = 0; i < match.iid_grounds.size(); ++i) {
            uint32_t iid = match.iid_grounds.at(i);
            Vector2f pt = match.pt_grounds.at(i);
            add_measurment(get_name(block_ground_.photos.at(iid).path), pt);
        }
        for (int i = 0; i < match.iid_aerials.size(); ++i) {
            uint32_t iid = match.iid_aerials.at(i);
            Vector2f pt = match.pt_aerials.at(i);
            add_measurment(get_name(block_aerial_.photos.at(iid).path), pt);
        }

        id++;
    }

    std::ofstream output(ofile);
    output << doc;
}

RenderMatchResults RenderMatcher::match(uint32_t iid, const cv::Mat &mat_rgb, const cv::Mat &mat_dep) {
    // initialize viewpoint
    viewport_.x() = mat_dep.cols;
    viewport_.y() = mat_dep.rows;

    // load the sift descriptors of the ground image
    std::vector<cv::KeyPoint> keys_ground;
    cv::Mat desc_ground;

    {
        std::string path = block_ground_.photos.at(iid).path;
        std::string name = get_filename_noext(path);
        std::string dir = get_directory(path);
        path = join_paths(dir, name + ".sift");
        if (file_exist(path)) {
            sift_read(path, keys_ground, desc_ground);
        } else {
            cv::Mat mat = cv::imread(images_ground_.at(iid)->get_path(), cv::IMREAD_GRAYSCALE);
            mat = image_percent_scale_8u(mat);
            std::tie(keys_ground, desc_ground) = sift_->detect_and_compute(mat);
        }
    }

    // extract sift from rendered images
    std::vector<cv::KeyPoint> keys_render;
    cv::Mat desc_render;
    {
        cv::Mat mat = mat_rgb.clone();
        cv::Mat alpha;
        {
            std::vector<cv::Mat> channels;
            cv::split(mat_rgb, channels);
            alpha = channels[3];
        }

        cv::cvtColor(mat, mat, cv::COLOR_BGRA2GRAY);
        mat = image_percent_scale_8u(mat, alpha != 0);
        std::tie(keys_render, desc_render) = sift_->detect_and_compute(mat);
    }

    // match sift
    SiftMatcherParam sift_param;
    SiftMatcher sift_matcher;
    sift_matcher.set_match_param(sift_param);
    sift_matcher.set_train_data(keys_ground, desc_ground);
    // std::vector<cv::DMatch> matches = sift_matcher.match(keys_render, desc_render);

    // local geometry constraints for outlier removal
    std::vector<cv::DMatch> matches = sift_matcher.match_proposed(keys_render, desc_render, keys_ground, keys_render);

    // too few matches
    if (matches.size() < 10) {
        return RenderMatchResults();
    }

    // reserve only some matches for propagation
    if (matches.size() > param_.num_propagation) {
        std::shuffle(begin(matches), end(matches), std::default_random_engine());
        matches.resize(param_.num_propagation);
    }

    // for each match expand to aerial/ground views with patch match
    RenderMatchResults results;

    for (const auto &match : matches) {
        auto key_ground = keys_ground.at(match.trainIdx);
        auto key_render = keys_render.at(match.queryIdx);

        int r = int(key_render.pt.y + 0.5);
        int c = int(key_render.pt.x + 0.5);

        // extract ground patch
        cv::Mat mat_ground_template = get_patch_on_ground_image(iid, Vector2d(key_ground.pt.x, key_ground.pt.y));
        if (mat_ground_template.empty()) {
            continue;
        }

        // debug
        //         cv::Mat mat_patch_ren = debug_patch_on_render_image(mat_rgb, Vector2d(key_render.pt.x,
        //         key_render.pt.y));

        // extract a planar (a point and normal) on the rendered depth image
        MatrixXf corners_ren;
        MatrixXf point_ren;
        Vector3f normal_ren;
        std::tie(corners_ren, point_ren, normal_ren) = get_patch_on_rendered_image(iid, Vector2i(c, r), mat_dep);
        if (corners_ren.rows() == 0) {
            continue;
        }

        RenderMatchResult result;
        result.xyz = point_ren;
        result.iid_ground = iid;
        result.pt_ground = Vector2f(key_ground.pt.x, key_ground.pt.y);

        const auto &propagate = [&](bool to_aerial) {
            // search visible aerial images
            std::vector<uint32_t> iids_aerial = search_visible_aerial_images(corners_ren, normal_ren, to_aerial);
            if (iids_aerial.empty()) {
                return;
            }

            for (uint32_t iid_aerial : iids_aerial) {

                if (iid_aerial == iid && to_aerial == false) {
                    continue;
                }

                cv::Mat mat_pat;
                Matrix3f H_pat_aer;
                std::tie(mat_pat, H_pat_aer) =
                    get_patch_on_aerial_image(iid, iid_aerial, corners_ren, normal_ren, to_aerial);
                if (mat_pat.empty()) {
                    continue;
                }

                // do template match
                cv::Mat ncc;
                cv::matchTemplate(mat_pat, mat_ground_template, ncc, cv::TM_CCOEFF_NORMED);
                double ncc_max;
                cv::Point pos;
                cv::minMaxLoc(ncc, nullptr, &ncc_max, nullptr, &pos);

                if (ncc_max < param_.ncc_threshold) {
                    continue;
                }
                int ncc_size = mat_pat.rows - mat_ground_template.rows + 1;
                std::vector<double> affine = {
                    1.0, 0.0, (double)pos.x - ncc_size / 2, 0.0, 1.0, (double)pos.y - ncc_size / 2, 1.0, 0.0};

                if (param_.use_lsm) {
                    LSMSummary summary = lsm_match(mat_ground_template, mat_pat, affine);
                    if (!summary.is_converge) {
                        continue;
                    }
                }

                Vector2f point_pat(affine[2] + mat_pat.cols / 2, affine[5] + mat_pat.rows / 2);
                Vector2f point_aer = (H_pat_aer * point_pat.homogeneous()).hnormalized();

                if (to_aerial) {
                    result.iid_aerials.push_back(iid_aerial);
                    result.pt_aerials.push_back(point_aer);
                } else {
                    result.iid_grounds.push_back(iid_aerial);
                    result.pt_grounds.push_back(point_aer);
                }
            }
        };

        propagate(true);
        if (result.iid_aerials.empty()) {
            continue;
        }
        propagate(false);
        if (result.iid_grounds.empty()) {
            continue;
        }

        results.push_back(result);
    }

    return results;
}

cv::Mat RenderMatcher::draw_matches(uint32_t iid_ground, uint32_t iid_aerial, const RenderMatchResults &matches) {
    std::vector<cv::KeyPoint> keys_ground, keys_aerial;
    std::vector<cv::DMatch> dmatches;

    int pos = 0;
    for (const auto &match : matches) {
        uint32_t iid = match.iid_ground;
        if (iid != iid_ground) {
            continue;
        }

        for (int i = 0; i < match.iid_aerials.size(); ++i) {
            uint32_t iid2 = match.iid_aerials[i];
            if (iid2 != iid_aerial) {
                continue;
            }

            Vector2f pt_aerial = match.pt_aerials[i];

            dmatches.push_back(cv::DMatch(pos, pos, 0.0f));
            keys_ground.push_back(cv::KeyPoint(match.pt_ground.x(), match.pt_ground.y(), 32.0f));
            keys_aerial.push_back(cv::KeyPoint(pt_aerial.x(), pt_aerial.y(), 32.0f));
            pos++;
        }
    }

    if (dmatches.size() == 0) {
        return cv::Mat();
    }

    std::string path_ground = block_ground_.photos.at(iid_ground).path;
    std::string path_aerial = block_aerial_.photos.at(iid_aerial).path;

    cv::Mat mat_ground = cv::imread(path_ground, cv::IMREAD_UNCHANGED);
    cv::Mat mat_aerial = cv::imread(path_aerial, cv::IMREAD_UNCHANGED);

    cv::Mat mat;
    cv::drawMatches(mat_ground, keys_ground, mat_aerial, keys_aerial, dmatches, mat);

    return mat;
}

std::tuple<MatrixXf, Vector3f, Vector3f> RenderMatcher::get_patch_on_rendered_image(uint32_t iid, const Vector2i &pt,
                                                                                    const cv::Mat &mat_dep) {

    int patch_size = (param_.ncc_window / 2 + param_.ncc_search / 2) * 2 + 1;
    int patch_half = patch_size / 2;

    int rows = mat_dep.rows;
    int cols = mat_dep.cols;

    // contain in eigen allow overlap
    BoundingBox2i bounds_image;
    bounds_image.extend(Vector2i(0, 0));
    bounds_image.extend(Vector2i(cols - 1, rows - 1));
    BoundingBox2i bounds_patch;
    bounds_patch.extend(Vector2i(pt.x() - patch_half, pt.y() - patch_half));
    bounds_patch.extend(Vector2i(pt.x() + patch_half + 1, pt.y() + patch_half + 1));

    if (!bounds_image.contains(bounds_patch)) {
        return std::make_tuple(MatrixXf(), Vector3f::Zero(), Vector3f::Zero());
    }

    // compute the 3d point of the four corners and center
    MatrixXf corners(3, 4);
    for (int i = 0; i < 4; ++i) {
        Vector2i corner = bounds_patch.corner(BoundingBox2i::CornerType(i));

        float depth = mat_dep.at<float>(corner.y(), corner.x());
        if (std::abs(depth - 1.0) < 1e-4) {
            // invalid depth value
            return std::make_tuple(MatrixXf(), Vector3f::Zero(), Vector3f::Zero());
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
    Vector3f point;
    {
        float depth = mat_dep.at<float>(pt.y(), pt.x());
        if (std::abs(depth - 1.0) < 1e-4) {
            // invalid depth value
            return std::make_tuple(MatrixXf(), Vector3f::Zero(), Vector3f::Zero());
        }
        point = depth_to_xyz(depth, pt);
        Vector3f C = block_ground_.photos.at(iid).C.cast<float>();

        if (nor.dot(C - point) < 0) {
            nor = -nor;
        }
    }

    return std::tie(corners, point, nor);
}

cv::Mat RenderMatcher::debug_patch_on_render_image(const cv::Mat &mat_rgb, const Vector2d &point) {
    int patch_size = (param_.ncc_window / 2) * 2 + 1;
    int patch_half = patch_size / 2;

    int rows = mat_rgb.rows;
    int cols = mat_rgb.cols;

    BoundingBox2i bounds_image;
    bounds_image.extend(Vector2i(0, 0));
    bounds_image.extend(Vector2i(cols, rows));

    BoundingBox2i bounds_patch;
    bounds_patch.extend(Vector2i(point.x() - 1.5 - patch_half, point.y() - 1.5 - patch_half));
    bounds_patch.extend(Vector2i(point.x() + 2.5 + patch_half, point.y() + 2.5 + patch_half));

    if (!bounds_image.contains(bounds_patch)) {
        return cv::Mat();
    }

    cv::Mat sub;
    cv::Mat mat = mat_rgb.clone();
    cv::cvtColor(mat, mat, cv::COLOR_BGRA2GRAY);
    cv::getRectSubPix(mat, cv::Size(patch_size, patch_size), cv::Point2f(point.x(), point.y()), sub);

    return sub;
}

std::tuple<cv::Mat, Matrix3f> RenderMatcher::get_patch_on_aerial_image(uint32_t iid_ground, uint32_t iid_aerial,
                                                                       const MatrixXf &corners, const Vector3f &normal,
                                                                       bool from_aerial) {
    // the corners corresponding to the 3D points of the four 2d corners of the final warped patch
    // we have four coordinate systems
    // - patch (as pat): the final patch warped to the ground view
    // - rendered image (as ren): assumes to have the no deformation with the patch and corners are input
    // - aerial (as aer): the coordinate of the aerial images
    // - aerial subset (as sub): a subset region, which is loaded and used for warp to the final patch
    const h2o::Block &block = from_aerial ? block_aerial_ : block_ground_;
    const auto &images = from_aerial ? images_aerial_ : images_ground_;

    Photo photo_aerial = block.photos.at(iid_aerial);
    BoundingBox2i bounds_aer;
    bounds_aer.extend(Vector2i(0, 0));
    bounds_aer.extend(Vector2i(block.groups.at(photo_aerial.cid).width, block.groups.at(photo_aerial.cid).height));

    // project the four corners to the aerial images
    BoundingBox2i bounds_sub;

    // the two sets of corners are used to estimate the homography for the transformation between patch and original
    // aerial image
    std::vector<Vector2d> corners_pat(4), corners_aer(4);
    for (int i = 0; i < 4; ++i) {
        Vector3d point = corners.col(i).cast<double>();
        Vector2d point2d = block.project(point, iid_aerial);
        corners_aer[i] = point2d;

        bounds_sub.extend(Vector2i(point2d.x() + 1.5, point2d.y() + 1.5));
        bounds_sub.extend(Vector2i(point2d.x() - 0.5, point2d.y() - 0.5));
    }

    if (!bounds_aer.contains(bounds_sub)) {
        return std::tuple<cv::Mat, Matrix3f>();
    }

    cv::Mat mat_sub;
    // reads M * sub = aer
    Matrix23d M_sub_aer;
    std::tie(M_sub_aer, mat_sub) = images.at(iid_aerial)->get_patch(bounds_sub, bounds_sub.sizes());
    if (mat_sub.channels() == 3) {
        cv::cvtColor(mat_sub, mat_sub, cv::COLOR_RGB2GRAY);
    }

    Matrix23d M_aer_sub = revers_affine(M_sub_aer);

    int patch_size = (param_.ncc_window / 2 + param_.ncc_search / 2) * 2 + 1;
    cv::Mat mat_pat = cv::Mat::zeros(patch_size, patch_size, CV_8U);

    BoundingBox2i bounds_pat;
    bounds_pat.extend(Vector2i(0, 0));
    bounds_pat.extend(Vector2i(patch_size, patch_size));
    for (int i = 0; i < 4; ++i) {
        corners_pat[i] = bounds_pat.corner(BoundingBox2i::CornerType(i)).cast<double>();
    }

    Matrix3d H_pat_aer = compute_homography(corners_pat, corners_aer);

    // warp images from the subset patch
    {
        std::vector<Vector2d> corners_sub(4);
        for (int i = 0; i < 4; ++i) {
            corners_sub[i] = M_aer_sub * corners_aer[i].homogeneous();
        }
        Matrix3d H_sub_pat = compute_homography(corners_sub, corners_pat);

        cv::Mat H;
        cv::eigen2cv(H_sub_pat, H);
        cv::warpPerspective(mat_sub, mat_pat, H, mat_pat.size(), cv::INTER_LINEAR);
    }

    return std::make_tuple(mat_pat, H_pat_aer.cast<float>());
}

cv::Mat RenderMatcher::get_patch_on_ground_image(uint32_t iid, const Vector2d &point) {

    int patch_size = (param_.ncc_window / 2) * 2 + 1;
    int patch_half = patch_size / 2;

    uint32_t cid = block_ground_.photos.at(iid).cid;
    int rows = block_ground_.groups.at(cid).height;
    int cols = block_ground_.groups.at(cid).width;

    BoundingBox2i bounds_image;
    bounds_image.extend(Vector2i(0, 0));
    bounds_image.extend(Vector2i(cols, rows));

    BoundingBox2i bounds_patch;
    bounds_patch.extend(Vector2i(point.x() - 1.5 - patch_half, point.y() - 1.5 - patch_half));
    bounds_patch.extend(Vector2i(point.x() + 2.5 + patch_half, point.y() + 2.5 + patch_half));

    if (!bounds_image.contains(bounds_patch)) {
        return cv::Mat();
    }

    Matrix23d M_sub_ima;
    cv::Mat sub;
    std::tie(M_sub_ima, sub) = images_ground_.at(iid)->get_patch(bounds_patch, bounds_patch.sizes());
    if (sub.channels() == 3) {
        cv::cvtColor(sub, sub, cv::COLOR_RGB2GRAY);
    }
    Matrix23d M_ima_sub = revers_affine(M_sub_ima);
    Vector2d center = M_ima_sub * point.homogeneous();

    cv::Mat sub_extract;
    cv::getRectSubPix(sub, cv::Size(patch_size, patch_size), cv::Point2f(center.x(), center.y()), sub_extract);

    return sub_extract;
}

std::vector<uint32_t> RenderMatcher::search_visible_aerial_images(const MatrixXf &corners, const Vector3f &normal,
                                                                  bool from_aerial) {
    std::vector<uint32_t> vis_angles;

    const auto &block = from_aerial ? block_aerial_ : block_ground_;

    Vector3f center = corners.rowwise().mean();
    double threshold = std::cos(param_.angle_difference * DEG2RAD);
    for (const auto &pair : block.photos) {
        Vector3f C = pair.second.C.cast<float>();
        Vector3f dir = (C - center).normalized();
        if (normal.dot(dir) < threshold) {
            continue;
        }

        vis_angles.push_back(pair.first);
    }

    // TODO: do occlusion detection

    return vis_angles;
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

/*
 * @Author: Han
 * @Date: 2019-10-21 19:30:39
 * Store information of block exchange format
 */
#pragma once

#include <base/base.h>

namespace h2o {
struct Photo {
    uint32_t id;
    uint32_t cid;
    std::string path;

    Vector3d C;
    Matrix3d R;

    double znear;
    double zfar;
    double zmed;
};

struct PhotoGroup {
    std::vector<uint32_t> photos;
    uint32_t width;
    uint32_t height;
    double f;
    double u0;
    double v0;
    double k1, k2, k3, p1, p2;
    Matrix3d K;
};
using Photos = std::map<uint32_t, Photo>;
using PhotoGroups = std::map<uint32_t, PhotoGroup>;

struct Block {
    Photos photos;
    PhotoGroups groups;

    Vector2d project(const Vector3d &point, uint32_t iid) const;
};

// load an xml block file
Block load_block_xml(const std::string &path);

// load a json block file
Block load_block(const std::string &path, const Eigen::Vector3d &origin_coord = Eigen::Vector3d(0.0, 0.0, 0.0));

// save a block file in json format
void save_block(const std::string &path, const Block &block);

// remove the distortion of the cameras with a given undistorted image path
Block undistort_block(const Block &block, const std::string &path = "");

FrameCamera to_framecamera(const PhotoGroup &pgroups);
} // namespace h2o

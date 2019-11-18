/*
 * @Author: Han
 * @Date: 2019-10-21 19:50:13
 * Store information of block exchange format
 */

#include "block.h"

#include <fstream>

#include <nlohmann/json.hpp>
#include <xml2json.hpp>

#include <QFile>
#include <QTextStream>

namespace h2o {
Block load_block_xml(const std::string &path) {

    nlohmann::json j;
    {
        QFile file(QString::fromStdString(path));
        file.open(QFile::ReadOnly);
        std::string xml_data = QTextStream(&file).readAll().toStdString();
        std::istringstream ss(xml2json(xml_data.c_str()));
        ss >> j;
    }

    auto block = j["BlocksExchange"]["Block"];
    if (block.empty()) {
        return Block();
    }

    std::vector<nlohmann::json> groups;

    if (block["Photogroups"]["Photogroup"].is_array()) {
        groups = block["Photogroups"]["Photogroup"].get<std::vector<nlohmann::json>>();
    } else {
        groups.push_back(block["Photogroups"]["Photogroup"]);
    }

    PhotoGroups block_photogroups;
    Photos block_photos;

    for (int cid = 0; cid < groups.size(); ++cid) {
        PhotoGroup block_photogroup;

        int width, height;
        double focal_pixel;
        double dx, dy;
        double k1(0.0), k2(0.0), k3(0.0), p1(0.0), p2(0.0);

        nlohmann::json g = groups[cid];
        width = std::stoi((std::string)g["ImageDimensions"]["Width"]);
        height = std::stoi((std::string)g["ImageDimensions"]["Height"]);

        if (g.find("FocalLengthPixels") != g.end()) {
            focal_pixel = std::stod((std::string)g["FocalLengthPixels"]);
        } else {
            double f = std::stod((std::string)g["FocalLength"]);
            double size = std::stod((std::string)g["SensorSize"]);
            double pixel = size / (width > height ? width : height);
            focal_pixel = f / pixel;
        }

        if (g.find("PrincipalPoint") != g.end()) {
            dx = std::stod((std::string)g["PrincipalPoint"]["x"]);
            dy = std::stod((std::string)g["PrincipalPoint"]["y"]);
        } else {
            dx = (width - 1.0) / 2.0;
            dy = (height - 1.0) / 2.0;
        }

        if (g.find("Distortion") != g.end()) {
            k1 = std::stod((std::string)g["Distortion"]["K1"]);
            k2 = std::stod((std::string)g["Distortion"]["K2"]);
            k3 = std::stod((std::string)g["Distortion"]["K3"]);

            // opencv and block exchange has different definition of p1 and p2
            // we use the opencv protocol
            p2 = std::stod((std::string)g["Distortion"]["P1"]);
            p1 = std::stod((std::string)g["Distortion"]["P2"]);
        }

        Matrix3d K = Matrix3d::Identity();
        K(0, 0) = K(1, 1) = focal_pixel;
        K(0, 2) = dx;
        K(1, 2) = dy;
        std::vector<double> distortion;
        distortion.push_back(k1);
        distortion.push_back(k2);
        distortion.push_back(p1);
        distortion.push_back(p2);
        distortion.push_back(k3);

        block_photogroup.f = focal_pixel;
        block_photogroup.width = width;
        block_photogroup.height = height;
        block_photogroup.u0 = dx;
        block_photogroup.v0 = dy;
        block_photogroup.K = K;
        block_photogroup.k1 = k1;
        block_photogroup.k2 = k2;
        block_photogroup.k3 = k3;
        block_photogroup.p1 = p1;
        block_photogroup.p2 = p2;

        Matrix3d O = Matrix3d::Identity();
        if (g.find("CameraOrientation") != g.end()) {
            std::string co = g["CameraOrientation"];
            if (co == "XRightYDown") {
                O << 1, 0, 0, 0, 1, 0, 0, 0, 1;
            } else if (co == "XLeftYDown") {
                O << -1, 0, 0, 0, 1, 0, 0, 0, -1;
            } else if (co == "XLeftYUp") {
                O << -1, 0, 0, 0, -1, 0, 0, 0, 1;
            } else if (co == "XRightYUp") {
                O << 1, 0, 0, 0, -1, 0, 0, 0, -1;
            } else if (co == "XDownYRight") {
                O << 0, 1, 0, 1, 0, 0, 0, 0, -1;
            } else if (co == "XDownYLeft") {
                O << 0, -1, 0, 1, 0, 0, 0, 0, 1;
            } else if (co == "XUpYLeft") {
                O << 0, -1, 0, -1, 0, 0, 0, 0, -1;
            } else if (co == "XUpYRight") {
                O << 0, 1, 0, -1, 0, 0, 0, 0, 1;
            }
        }

        std::vector<nlohmann::json> photos;
        if (g["Photo"].is_array()) {
            photos = g["Photo"].get<std::vector<nlohmann::json>>();
        } else {
            photos.push_back(g["Photo"]);
        }

        std::vector<uint32_t> photo_iids;

        for (const auto &p : photos) {
            iid_t iid = std::stoi((std::string)p["Id"]);
            std::string path = p["ImagePath"];

            Matrix3d R;

            if (p.find("Pose") == p.end() || p["Pose"].find("Rotation") == p["Pose"].end() ||
                p["Pose"].find("Center") == p["Pose"].end()) {
                continue;
            }
            // AT file has many formations and to be simple we only accept rotation matrix
            if (p["Pose"]["Rotation"].find("M_00") == p["Pose"]["Rotation"].end()) {
                continue;
            }

            R(0, 0) = std::stod((std::string)p["Pose"]["Rotation"]["M_00"]);
            R(0, 1) = std::stod((std::string)p["Pose"]["Rotation"]["M_01"]);
            R(0, 2) = std::stod((std::string)p["Pose"]["Rotation"]["M_02"]);
            R(1, 0) = std::stod((std::string)p["Pose"]["Rotation"]["M_10"]);
            R(1, 1) = std::stod((std::string)p["Pose"]["Rotation"]["M_11"]);
            R(1, 2) = std::stod((std::string)p["Pose"]["Rotation"]["M_12"]);
            R(2, 0) = std::stod((std::string)p["Pose"]["Rotation"]["M_20"]);
            R(2, 1) = std::stod((std::string)p["Pose"]["Rotation"]["M_21"]);
            R(2, 2) = std::stod((std::string)p["Pose"]["Rotation"]["M_22"]);

            R = O * R;

            double x = std::stod((std::string)p["Pose"]["Center"]["x"]);
            double y = std::stod((std::string)p["Pose"]["Center"]["y"]);
            double z = std::stod((std::string)p["Pose"]["Center"]["z"]);

            double znear = std::stod((std::string)p["NearDepth"]);
            double zfar = std::stod((std::string)p["FarDepth"]);
            double zmed = std::stod((std::string)p["MedianDepth"]);

            Photo block_photo;
            block_photo.id = iid;
            block_photo.cid = cid;
            block_photo.path = path;
            block_photo.C = Vector3d(x, y, z);
            block_photo.R = R;
            block_photo.znear = znear;
            block_photo.zfar = zfar;
            block_photo.zmed = zmed;

            block_photos.emplace(iid, block_photo);
            photo_iids.push_back(iid);
        }
        block_photogroup.photos = photo_iids;
        block_photogroups.emplace(cid, block_photogroup);
    }

    Block exchange;
    exchange.photos = block_photos;
    exchange.groups = block_photogroups;

    return exchange;
}

Block load_block(const std::string &path) {

    nlohmann::json j;
    {
        QFile file(QString::fromStdString(path));
        file.open(QFile::ReadOnly);
        std::string data = QTextStream(&file).readAll().toStdString();
        std::istringstream ss(data);
        ss >> j;
    }

    auto block = j["BlocksExchange"]["Block"];
    if (block.empty()) {
        return Block();
    }

    std::vector<nlohmann::json> groups;
    if (block["Photogroups"]["Photogroup"].is_array()) {
        groups = block["Photogroups"]["Photogroup"].get<std::vector<nlohmann::json>>();
    } else {
        groups.push_back(block["Photogroups"]["Photogroup"]);
    }

    PhotoGroups block_photogroups;
    Photos block_photos;

    for (int cid = 0; cid < groups.size(); ++cid) {
        PhotoGroup block_photogroup;

        int width, height;
        double focal_pixel;
        double dx, dy;
        double k1(0.0), k2(0.0), k3(0.0), p1(0.0), p2(0.0);

        nlohmann::json g = groups[cid];
        width = g["ImageDimensions"]["Width"];
        height = g["ImageDimensions"]["Height"];
        focal_pixel = g["FocalLengthPixels"];

        if (g.find("PrincipalPoint") != g.end()) {
            dx = g["PrincipalPoint"]["x"];
            dy = g["PrincipalPoint"]["y"];
        } else {
            dx = (width - 1.0) / 2.0;
            dy = (height - 1.0) / 2.0;
        }

        if (g.find("Distortion") != g.end()) {
            k1 = g["Distortion"]["K1"];
            k2 = g["Distortion"]["K2"];
            k3 = g["Distortion"]["K3"];
            p1 = g["Distortion"]["P1"];
            p2 = g["Distortion"]["P2"];
        }

        Matrix3d K = Matrix3d::Identity();
        K(0, 0) = K(1, 1) = focal_pixel;
        K(0, 2) = dx;
        K(1, 2) = dy;
        std::vector<double> distortion;
        distortion.push_back(k1);
        distortion.push_back(k2);
        distortion.push_back(p1);
        distortion.push_back(p1);
        distortion.push_back(k3);

        block_photogroup.f = focal_pixel;
        block_photogroup.width = width;
        block_photogroup.height = height;
        block_photogroup.u0 = dx;
        block_photogroup.v0 = dy;
        block_photogroup.K = K;
        block_photogroup.k1 = k1;
        block_photogroup.k2 = k2;
        block_photogroup.k3 = k3;
        block_photogroup.p1 = p1;
        block_photogroup.p1 = p2;

        Matrix3d O = Matrix3d::Identity();
        if (g.find("CameraOrientation") != g.end()) {
            std::string co = g["CameraOrientation"];
            if (co == "XRightYDown") {
                O << 1, 0, 0, 0, 1, 0, 0, 0, 1;
            } else if (co == "XLeftYDown") {
                O << -1, 0, 0, 0, 1, 0, 0, 0, -1;
            } else if (co == "XLeftYUp") {
                O << -1, 0, 0, 0, -1, 0, 0, 0, 1;
            } else if (co == "XRightYUp") {
                O << 1, 0, 0, 0, -1, 0, 0, 0, -1;
            } else if (co == "XDownYRight") {
                O << 0, 1, 0, 1, 0, 0, 0, 0, -1;
            } else if (co == "XDownYLeft") {
                O << 0, -1, 0, 1, 0, 0, 0, 0, 1;
            } else if (co == "XUpYLeft") {
                O << 0, -1, 0, -1, 0, 0, 0, 0, -1;
            } else if (co == "XUpYRight") {
                O << 0, 1, 0, -1, 0, 0, 0, 0, 1;
            }
        }

        std::vector<nlohmann::json> photos;
        if (g["Photo"].is_array()) {
            photos = g["Photo"].get<std::vector<nlohmann::json>>();
        } else {
            photos.push_back(g["Photo"]);
        }

        std::vector<uint32_t> photo_iids;

        for (const auto &p : photos) {
            iid_t iid = p["Id"];
            std::string path = p["ImagePath"];

            Matrix3d R;

            if (p.find("Pose") == p.end() || p["Pose"].find("Rotation") == p["Pose"].end() ||
                p["Pose"].find("Center") == p["Pose"].end()) {
                continue;
            }
            // AT file has many formations and to be simple we only accept rotation matrix
            if (p["Pose"]["Rotation"].find("M_00") == p["Pose"]["Rotation"].end()) {
                continue;
            }

            R(0, 0) = p["Pose"]["Rotation"]["M_00"];
            R(0, 1) = p["Pose"]["Rotation"]["M_01"];
            R(0, 2) = p["Pose"]["Rotation"]["M_02"];
            R(1, 0) = p["Pose"]["Rotation"]["M_10"];
            R(1, 1) = p["Pose"]["Rotation"]["M_11"];
            R(1, 2) = p["Pose"]["Rotation"]["M_12"];
            R(2, 0) = p["Pose"]["Rotation"]["M_20"];
            R(2, 1) = p["Pose"]["Rotation"]["M_21"];
            R(2, 2) = p["Pose"]["Rotation"]["M_22"];

            R = O * R;

            double x = p["Pose"]["Center"]["x"];
            double y = p["Pose"]["Center"]["y"];
            double z = p["Pose"]["Center"]["z"];

            double znear = p["NearDepth"];
            double zmed = p["MedianDepth"];
            double zfar = p["FarDepth"];

            Photo block_photo;
            block_photo.id = iid;
            block_photo.cid = cid;
            block_photo.path = path;
            block_photo.C = Vector3d(x, y, z);
            block_photo.R = R;
            block_photo.znear = znear;
            block_photo.zmed = zmed;
            block_photo.zfar = zfar;

            block_photos.emplace(iid, block_photo);
            photo_iids.push_back(iid);
        }
        block_photogroup.photos = photo_iids;
        block_photogroups.emplace(cid, block_photogroup);
    }

    Block exchange;
    exchange.photos = block_photos;
    exchange.groups = block_photogroups;

    return exchange;

    return Block();
}

void save_block(const std::string &path, const Block &block) {
    nlohmann::json j;
    j["BlocksExchange"]["Block"] = nlohmann::json();
    std::vector<nlohmann::json> jpgroups;

    for (const auto &pair : block.groups) {
        nlohmann::json jpgroup;
        uint32_t cid = pair.first;

        jpgroup["Name"] = "Photogroup #" + std::to_string(cid);
        jpgroup["ImageDimensions"]["Width"] = pair.second.width;
        jpgroup["ImageDimensions"]["Height"] = pair.second.height;
        jpgroup["FocalLengthPixels"] = pair.second.f;
        jpgroup["CameraOrientation"] = "XRightYDown";
        jpgroup["PrincipalPoint"]["x"] = pair.second.u0;
        jpgroup["PrincipalPoint"]["y"] = pair.second.v0;
        jpgroup["AspectRatio"] = 1;
        jpgroup["Skew"] = 0.0;
        jpgroup["Distortion"]["K1"] = pair.second.k1;
        jpgroup["Distortion"]["K2"] = pair.second.k2;
        jpgroup["Distortion"]["K3"] = pair.second.k3;
        jpgroup["Distortion"]["P1"] = pair.second.p1;
        jpgroup["Distortion"]["P2"] = pair.second.p2;

        std::vector<nlohmann::json> jphotos;
        for (uint32_t iid : pair.second.photos) {
            nlohmann::json jphoto;
            Photo photo = block.photos.at(iid);
            jphoto["Id"] = iid;
            jphoto["ImagePath"] = photo.path;
            jphoto["Pose"]["Rotation"]["M_00"] = photo.R(0, 0);
            jphoto["Pose"]["Rotation"]["M_01"] = photo.R(0, 1);
            jphoto["Pose"]["Rotation"]["M_02"] = photo.R(0, 2);
            jphoto["Pose"]["Rotation"]["M_10"] = photo.R(1, 0);
            jphoto["Pose"]["Rotation"]["M_11"] = photo.R(1, 1);
            jphoto["Pose"]["Rotation"]["M_12"] = photo.R(1, 2);
            jphoto["Pose"]["Rotation"]["M_20"] = photo.R(2, 0);
            jphoto["Pose"]["Rotation"]["M_21"] = photo.R(2, 1);
            jphoto["Pose"]["Rotation"]["M_22"] = photo.R(2, 2);
            jphoto["Pose"]["Center"]["x"] = photo.C(0);
            jphoto["Pose"]["Center"]["y"] = photo.C(1);
            jphoto["Pose"]["Center"]["z"] = photo.C(2);
            jphoto["NearDepth"] = photo.znear;
            jphoto["MedianDepth"] = photo.zmed;
            jphoto["FarDepth"] = photo.zfar;

            jphotos.push_back(jphoto);
        }
        jpgroup["Photo"] = jphotos;
        jpgroups.push_back(jpgroup);
    }
    j["BlocksExchange"]["Block"]["Photogroups"]["Photogroup"] = jpgroups;

    std::ofstream ofile(string_utf8_to_fstream(path));
    ofile << j.dump(4);
    ofile.close();
}

Block undistort_block(const Block &block, const std::string &path) {
    Block rectified = block;
    for (auto &pair : rectified.groups) {
        auto &camera = pair.second;
        camera.u0 = (camera.width - 1.0) / 2.0;
        camera.v0 = (camera.height - 1.0) / 2.0;
        camera.K(0, 2) = camera.u0;
        camera.K(1, 2) = camera.v0;
        camera.k1 = 0.0;
        camera.k2 = 0.0;
        camera.k3 = 0.0;
        camera.p1 = 0.0;
        camera.p2 = 0.0;
    }

    if (path.empty()) {
        return rectified;
    }

    for (auto &pair : rectified.photos) {
        auto &photo = pair.second;
        std::string name = get_filename_noext(photo.path);
        name = join_paths(path, name + ".jpg");
        photo.path = name;
    }

    return rectified;
}

FrameCamera to_framecamera(const PhotoGroup &pgroup) {
    FrameCamera camera;
    camera.distortion_.push_back(pgroup.k1);
    camera.distortion_.push_back(pgroup.k2);
    camera.distortion_.push_back(pgroup.k3);
    camera.distortion_.push_back(pgroup.p1);
    camera.distortion_.push_back(pgroup.p2);
    camera.K = pgroup.K;
    camera.size_(0) = pgroup.width;
    camera.size_(1) = pgroup.height;

    return camera;
}
Vector2d Block::project(const Vector3d &point, uint32_t iid) const {

    uint32_t cid = photos.at(iid).cid;
    double f = groups.at(cid).f;

    Vector3d XC = point - photos.at(iid).C;
    XC = photos.at(iid).R * XC;

    Vector2d xc = XC.hnormalized();
    double r2 = xc.squaredNorm();

    double k1 = groups.at(cid).k1;
    double k2 = groups.at(cid).k2;
    double k3 = groups.at(cid).k3;
    double p1 = groups.at(cid).p1;
    double p2 = groups.at(cid).p2;
    double kd = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;

    Vector2d xd;
    xd.x() = xc.x() * kd + 2 * p1 * xc.x() * xc.y() + p2 * (r2 + 2 * xc.x() * xc.x());
    xd.y() = xc.y() * kd + p1 * (r2 + 2 * xc.y() * xc.y()) + 2 * p2 * xc.x() * xc.y();

    double u0 = groups.at(cid).u0;
    double v0 = groups.at(cid).v0;
    Vector2d x = f * xd + Vector2d(u0, v0);

    return x;
}
} // namespace h2o

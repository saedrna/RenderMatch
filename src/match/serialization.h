/*
 * @Author: Han Hu
 * @Date: 2018-05-31 11:18:40
 * 写入磁盘
 */
#include <base/common.h>
#include <match/recon_track.h>

#include <cereal/archives/json.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/unordered_map.hpp>

namespace cereal {

template <class Archive> inline void save(Archive &ar, const Eigen::VectorXd &m) {
    int n = m.rows();
    ar(CEREAL_NVP(n));
    for (int i = 0; i < n; ++i) {
        ar(m(i));
    }
}

template <class Archive> inline void load(Archive &ar, Eigen::VectorXd &m) {
    int n;
    ar(CEREAL_NVP(n));
    m.resize(n);
    for (int i = 0; i < n; ++i) {
        ar(m(i));
    }
}
template <class Archive> inline void save(Archive &ar, const Eigen::Vector3d &m) {
    ar(make_nvp("x", m(0)));
    ar(make_nvp("y", m(1)));
    ar(make_nvp("z", m(2)));
}
template <class Archive> inline void load(Archive &ar, Eigen::Vector3d &m) {
    ar(make_nvp("x", m(0)));
    ar(make_nvp("y", m(1)));
    ar(make_nvp("z", m(2)));
}
template <class Archive> inline void serialize(Archive &ar, Eigen::Vector2d &m) {
    ar(make_nvp("x", m(0)));
    ar(make_nvp("y", m(1)));
}
template <class Archive> inline void serialize(Archive &ar, Eigen::Vector2f &m) {
    ar(make_nvp("x", m(0)));
    ar(make_nvp("y", m(1)));
}

template <class Archive> inline void serialize(Archive &ar, h2o::Matrix23d &m) {
    ar(make_nvp("m00", m(0, 0)));
    ar(make_nvp("m01", m(0, 1)));
    ar(make_nvp("m02", m(0, 2)));
    ar(make_nvp("m10", m(1, 0)));
    ar(make_nvp("m11", m(1, 1)));
    ar(make_nvp("m12", m(1, 2)));
}

template <class Archive> inline void serialize(Archive &ar, h2o::Track &track) {
    ar(CEREAL_NVP(track.X));
    ar(CEREAL_NVP(track.obs));
}

template <class Archive> inline void serialize(Archive &ar, h2o::Observation &obs) {
    ar(CEREAL_NVP(obs.x));
    ar(CEREAL_NVP(obs.fid));
}
} // namespace cereal

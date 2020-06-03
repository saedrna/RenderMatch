#include "render_matcher.h"

#include <cereal/archives/portable_binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>

using namespace h2o;

namespace cereal {

template <class Archive> inline void serialize(Archive &ar, Eigen::Vector3f &m) {
    ar(make_nvp("x", m(0)));
    ar(make_nvp("y", m(1)));
    ar(make_nvp("z", m(2)));
}
template <class Archive> inline void serialize(Archive &ar, Eigen::Vector2f &m) {
    ar(make_nvp("x", m(0)));
    ar(make_nvp("y", m(1)));
}

template <class Archive> inline void serialize(Archive &ar, RenderMatchResult &m) {
    ar(make_nvp("Position", m.xyz));
    ar(make_nvp("GroundId", m.iid_ground));
    ar(make_nvp("GroundPt", m.pt_ground));
    ar(make_nvp("AerialIds", m.iid_aerials));
    ar(make_nvp("AerialPts", m.pt_aerials));
    ar(make_nvp("GroundIds", m.iid_grounds));
    ar(make_nvp("GroundPts", m.pt_grounds));
}

} // namespace cereal

void save_matches(const RenderMatchResults &match, const std::string &path) {
    std::ofstream ofile(path, std::ios::binary);

    cereal::PortableBinaryOutputArchive ar(ofile);
    ar(match);
}

RenderMatchResults load_matches(const std::string &path) {
    std::ifstream ifile(path, std::ios::binary);
    cereal::PortableBinaryInputArchive ar(ifile);
    RenderMatchResults m;
    ar(m);
    return m;
}

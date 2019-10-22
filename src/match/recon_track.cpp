/*!
 * \file recon_track.cpp
 *
 * \author Han
 * \date 2017/05/08
 *
 * 一个 track 包含多个影像的匹配点
 */
#include <fstream>

#include <match/recon_feature.h>
#include <match/recon_match.h>
#include <match/recon_track.h>
#include <match/serialization.h>
#include <base/base.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

/************************************************************************/
/*                实例化 Track 的 serialization 的类                    */
/************************************************************************/
namespace cereal {} // namespace cereal

namespace h2o {
TrackStatistics track_statisics(const Tracks &tracks) {
    TrackStatistics stats;
    std::unordered_map<int, uint32_t> track_num;

    for (const auto &track : tracks) {
        int num = track.second.obs.size();
        if (track_num.count(num)) {
            track_num[num]++;
        } else {
            track_num[num] = 1;
        }
    }
    stats.set_track_num(track_num);
    return stats;
}
void save_tracks(const std::string &path, const Tracks &tracks, bool binary) {
    if (binary) {
        std::ofstream ofile(string_utf8_to_fstream(path), std::ios::binary);
        CHECK(ofile.is_open());
        cereal::PortableBinaryOutputArchive archive(ofile);
        archive(tracks);
    } else {
        std::ofstream ofile(string_utf8_to_fstream(path));
        CHECK(ofile.is_open());
        cereal::JSONOutputArchive archive(ofile);
        archive(tracks);
    }
}

Tracks load_tracks(const std::string &path, bool binary) {
    if (binary) {
        std::ifstream ifile(string_utf8_to_fstream(path), std::ios::binary);
        CHECK(ifile.is_open());
        cereal::PortableBinaryInputArchive archive(ifile);
        Tracks tracks;
        archive(tracks);
        return tracks;
    } else {
        std::ifstream ifile(string_utf8_to_fstream(path));
        CHECK(ifile.is_open());
        cereal::JSONInputArchive archive(ifile);
        Tracks tracks;
        archive(tracks);
        return tracks;
    }
}

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> graph_t;
typedef std::pair<iid_t, fid_t> vertex_key_t;
typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_t;

Tracks connect_matches(std::shared_ptr<FeaturePointsContainer> features_container,
                       std::shared_ptr<PairwiseMatchContainer> matches_container) {

    // 判断有多少个独立的影像点
    std::unordered_map<vertex_key_t, vertex_t, hash_pair> vertex_indices;
    vertex_t vid = 0;
    for (const auto &pair : matches_container->get_pairs()) {
        int iid_i, iid_j;
        iid_i = pair.first;
        iid_j = pair.second;

        IndexMatches matches = matches_container->get_matches(pair);

        for (const auto &match : matches) {
            vertex_key_t vertex_key_i = {iid_i, match.i};
            vertex_key_t vertex_key_j = {iid_j, match.j};

            if (vertex_indices.count(vertex_key_i) == 0) {
                vertex_indices.insert({vertex_key_i, vid++});
            }
            if (vertex_indices.count(vertex_key_j) == 0) {
                vertex_indices.insert({vertex_key_j, vid++});
            }
        }
    }

    // 再次便利匹配信息，将每个匹配点加入到图中
    int num_vertices = vertex_indices.size();
    graph_t g(num_vertices);

    for (const auto &pair : matches_container->get_pairs()) {
        int iid_i, iid_j;
        iid_i = pair.first;
        iid_j = pair.second;

        IndexMatches matches = matches_container->get_matches(pair);

        for (const auto &match : matches) {
            vertex_key_t vertex_key_i = {iid_i, match.i};
            vertex_key_t vertex_key_j = {iid_j, match.j};
            vertex_t vid_i, vid_j;
            vid_i = vertex_indices[vertex_key_i];
            vid_j = vertex_indices[vertex_key_j];

            if (vid_i < num_vertices && vid_j < num_vertices) {
                boost::add_edge(vid_i, vid_j, g);
            }
        }
    }

    // 根据 component 初始化 tracks
    std::vector<int> component(num_vertices);
    int num_componet = boost::connected_components(g, component.data());
    Tracks tracks;
    for (const auto &vertex_index : vertex_indices) {
        iid_t iid = vertex_index.first.first;
        iid_t fid = vertex_index.first.second;
        vertex_t v = vertex_index.second;
        oid_t oid = component[v];

        Observation observation;
        observation.fid = fid;
        tracks[oid].obs.insert({iid, observation});
    }

    // 遍历所有的 features 设置 tracks 的坐标
    auto image_indices = features_container->get_image_indices();
    for (iid_t image_idx : image_indices) {
        FeaturePoints features = features_container->get_points(image_idx);

        std::vector<Observation *> observations;
        for (auto &track : tracks) {
            if (track.second.obs.count(image_idx)) {
                observations.push_back(&track.second.obs.at(image_idx));
            }
        }

        for (Observation *observation : observations) {
            observation->x = features[observation->fid];
        }
    }

    return tracks;
}

Tracks merge_tracks_only_oid(const Tracks &base, const Tracks &other) {
    Tracks merged;
    oid_t oid = 0;
    for (const auto &t : base) {
        merged.emplace(oid, t.second);
        oid++;
    }
    for (const auto &t : other) {
        merged.emplace(oid, t.second);
        oid++;
    }

    return merged;
}

Tracks merge_tracks_with_fid(const std::vector<const Tracks *> vec_tracks) {
    // 判断有多少个独立的key
    std::unordered_map<vertex_key_t, vertex_t, hash_pair> vertex_indices;
    vertex_t vid = 0;
    for (const Tracks *tracks : vec_tracks) {
        for (const auto &track : *tracks) {
            for (const auto &obs : track.second.obs) {
                vertex_key_t vertex_key = {obs.first, obs.second.fid};
                if (vertex_indices.count(vertex_key) == 0) {
                    vertex_indices.emplace(vertex_key, vid);
                    vid++;
                }
            }
        }
    }

    // 再次遍历 track 信息，将每个匹配点加入到图中
    int num_vertices = vertex_indices.size();
    graph_t g(num_vertices);

    for (const Tracks *tracks : vec_tracks) {
        for (const auto &track : *tracks) {
            int first = true;
            vertex_key_t key_i;
            for (const auto &obs : track.second.obs) {
                vertex_key_t key_j;
                if (first) {
                    key_i = {obs.first, obs.second.fid};
                    first = false;
                } else {
                    key_j = {obs.first, obs.second.fid};
                    vertex_t vid_i, vid_j;
                    vid_i = vertex_indices[key_i];
                    vid_j = vertex_indices[key_j];
                    boost::add_edge(vid_i, vid_j, g);
                }
            }
        }
    }

    std::vector<int> component(num_vertices);
    int num_componet = boost::connected_components(g, component.data());

    // 再次遍历所有的 tracks， 创建新的 tracks 信息
    Tracks merged;
    for (const Tracks *tracks : vec_tracks) {
        for (const auto &track : *tracks) {
            for (const auto &obs : track.second.obs) {
                vertex_key_t vertex_key = {obs.first, obs.second.fid};
                vertex_t vid = vertex_indices[vertex_key];
                oid_t oid = component[vid];
                merged[oid].obs.insert(obs);
            }
        }
    }
    return merged;
}

std::unordered_map<int, uint32_t> TrackStatistics::get_track_num() const { return track_num; }

void TrackStatistics::set_track_num(const std::unordered_map<int, uint32_t> &num) { track_num = num; }

std::string TrackStatistics::get_description() const {
    std::string description = "=======================================================\n";
    std::vector<int> keys(track_num.size());
    std::transform(begin(track_num), end(track_num), begin(keys), [](const auto &pair) { return pair.first; });
    std::sort(begin(keys), end(keys));

    for (int key : keys) {
        std::string desc = string_printf("  -- The number of points for %3d is : %8d\n", key, track_num.at(key));
        description += desc;
    }
    description += "=======================================================\n";
    return description;
}

} // namespace h2o

/*!
 * \file track.h
 *
 * \author Han
 * \date 2017/05/08
 *
 * 一个 track 包含多个影像的匹配点
 */
#pragma once
#include <base/common.h>

namespace h2o {

class PairwiseMatchContainer;
class FeaturePointsContainer;

///\brief 一个 Observation 记录特定影像上的特征点的序号和坐标值
struct Observation {
    Observation() : fid(UNDEFINED_ID) {}
    Observation(const Vector2f &_x, fid_t _fid) : fid(_fid), x(_x) {}

    ///\brief 影像二维坐标
    Vector2f x;
    ///\brief 在影像中的点序号
    fid_t fid;
};

///\brief image id -> feature
typedef std::unordered_map<iid_t, Observation> Observations;
struct Track {
    ///\brief 物方三维坐标
    Vector3d X;

    ///\brief 包含所有影像上的特征点信息
    Observations obs;
};

typedef std::unordered_map<oid_t, Track> Tracks;

class TrackStatistics {
public:
    std::unordered_map<int, uint32_t> get_track_num() const;
    void set_track_num(const std::unordered_map<int, uint32_t> &num);
    ///\brief format 统计信息
    std::string get_description() const;

private:
    ///\brief 重叠度->点数
    std::unordered_map<int, uint32_t> track_num;
};

///\brief 获取 track 的每个重叠度的统计信息
TrackStatistics track_statisics(const Tracks &tracks);
/**
 * \brief 保存 tracks 到文件
 * \param path 如果是二进制，后缀需要是 .bin；如果是文本，后缀是 .json
 * \param tracks
 * \param binary 是否保存成二进制文件
 */
void save_tracks(const std::string &path, const Tracks &tracks, bool binary = true);

/**
 * \brief 读取 tracks 信息
 * \param path 如果是二进制，后缀需要是 .bin；如果是文本，后缀是 .json
 * \param binary 是否保存成二进制文件
 */
Tracks load_tracks(const std::string &path, bool binary = true);

/**
 * \brief 根据特征点和匹配信息获取tracks
 * \param features
 * \param matches
 */
Tracks connect_matches(std::shared_ptr<FeaturePointsContainer> features,
                       std::shared_ptr<PairwiseMatchContainer> matches);

/**
 * \brief 只是重新组织 oid 的合并不同的tracks，输出的 tracks 只是把两者的 oid 叠加
 *
 * 这种合并适用于两者来自不同的特征点和特征描述符，因此在观测值中记录的 fid 的信息是无效的，不能用来判断是否是 connected
 * componet
 */
Tracks merge_tracks_only_oid(const Tracks &base, const Tracks &other);

/**
 * \brief 采用 connected componets 算法对 (iid, fid) 所确定的一个特征点进行重新连接
 *
 * 这种试用于不同的 tracks 来自相同的特征点于描述符，因此 (iid, fid) 所确定的是同一个点，可以用来作为 key 值进行连接
 */
Tracks merge_tracks_with_fid(const Tracks &base, const Tracks &other);
} // namespace h2o

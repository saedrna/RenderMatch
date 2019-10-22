/*!
 * \file recon_feature.h
 *
 * \author Han
 * \date 2017/05/08
 *
 * 特征点信息
 */
#pragma once
#include <base/common.h>
#include <memory>
#include <unordered_map>

namespace h2o {
typedef Vector2f FeaturePoint;
///\brief 一张影像上的所有特征点的坐标
typedef std::vector<FeaturePoint> FeaturePoints;
///\brief 所有影像的特征点坐标
typedef std::unordered_map<iid_t, FeaturePoints> FeaturePointsImageMap;
///\brief 所有影像存储特征点的路径
typedef std::unordered_map<iid_t, std::string> FeaturePointsPath;
class FeaturePointsContainer {
public:
    FeaturePointsContainer(){};
    ~FeaturePointsContainer(){};

    ///\brief 读取特征点信息
    virtual void load(const FeaturePointsPath &paths) = 0;

    ///\brief 获取某个影像 id 的所有特征点
    virtual FeaturePoints get_points(iid_t iid) = 0;
    ///\brief 获取工程中的所有影像 id
    virtual std::vector<iid_t> get_image_indices() = 0;

    ///\brief 从磁盘中读取特征点
    static FeaturePoints read_points(const std::string &path);

    ///\brief 将特征点保存到磁盘
    static void save_points(const std::string &path, const FeaturePoints &points);

    ///\brief 从磁盘中读取特征点和描述符
    static std::tuple<FeaturePoints, cv::Mat> read_features(const std::string &path);

    ///\brief 将特征点和描述符存储到磁盘
    static void save_features(const std::string &path, const FeaturePoints &points, const cv::Mat &desc);
};

///\brief 从 opencv 格式特征点转换成二维点
FeaturePoints features_from_keypoints(const std::vector<cv::KeyPoint> &keys);

///\brief 创建一个特征点的 container，其中所有点的坐标信息都在内存之中
std::shared_ptr<FeaturePointsContainer> create_feature_container_all_in_memory(const FeaturePointsPath &paths);

///\brief 将 points 封装成一个通用的 container
std::shared_ptr<FeaturePointsContainer>
create_feature_container_all_in_memory(const std::shared_ptr<FeaturePointsImageMap> &points);

///\brief 创建一个 lru cache，内存中只保留 ncache 数目的影像的特征点，其他的从磁盘动态创建
std::shared_ptr<FeaturePointsContainer> create_feature_container_lru_cache(const FeaturePointsPath &paths, int ncache);
} // namespace h2o

/*!
 * \file sift_detector.h
 *
 * \author Han
 * \date 2017/05/04
 *
 *
 */
#pragma once
#include <base/common.h>
#include <match/recon_feature.h>

class SiftGPU;
namespace h2o {

/**
 * \brief 保留 response 最大的 N 个点（对SIFT其实用的是 size ）
 * \param keys 特征点
 * \param desc 特征描述符
 * \param nfeatures 保留数目
 */
void retain_best_features(std::vector<cv::KeyPoint> &keys, cv::Mat &desc, int nfeatures);

///\brief 保留 response 最大的 N 个点，会根据保留的数目确定格网划分的大小
void retain_best_features_gridded(std::vector<cv::KeyPoint> &keys, cv::Mat &desc, int nfeatures);

void filter_features_by_mask(std::vector<cv::KeyPoint> &keys, cv::Mat &desc, const cv::Mat &mask);

///\brief 将特征点和描述符保存到磁盘
void sift_save(const std::string &path, const std::vector<cv::KeyPoint> &kpts, const cv::Mat &desc);

///\brief 将 sift 点和描述符从磁盘中读出
void sift_read(const std::string &path, std::vector<cv::KeyPoint> &kpts, cv::Mat &desc);

///\brief 将匹配信息保存到磁盘
void match_save(const std::string &path, const std::vector<cv::DMatch> &matches);

///\brief 从磁盘中读出匹配信息
void match_read(const std::string &path, std::vector<cv::DMatch> &matches);

///\brief fast 特征点提取的接口，最低的 response 设置为 10
std::vector<cv::KeyPoint> detect_fast(const cv::Mat &mat, int nfeature = 20000);

class SiftDetector {
public:
    /**
     * \brief 构造函数
     * \param opt 特征提取的参数
     * 0 - sparse，feature 的 response 比较大，点少
     * 1 - normal，正常的 response
     * 2 - 会升采样影像
     * \param nfeat 保留特征点的数目
     */
    SiftDetector(int opt = 1, int nfeat = 20000);
    ~SiftDetector();

    std::tuple<std::vector<cv::KeyPoint>, cv::Mat> detect_and_compute(const cv::Mat &img);
    void detect_and_compute(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, cv::Mat &desc);
    cv::Mat compute(const cv::Mat &img, const std::vector<cv::KeyPoint> &keys, bool ignore_scale = true,
                    bool ignore_oriente = true);
    ///\brief 采用这个接口，不能顾及尺度和旋转信息
    cv::Mat compute(const cv::Mat &img, const FeaturePoints &features);

protected:
    /**
     * \brief 采用 root sum 的方式归一化特征描述符到 uint8_t
     *
     * Arandjelovic, R., Zisserman, A., 2012. Three things everyone should know to improve object
     * retrieval. CVPR2012.
     */
    void root_normalize(cv::Mat &desc);

    ///\brief 直接归一化到 uint8_t
    void normalize(cv::Mat &desc);

protected:
    int nfeat_;
    std::shared_ptr<SiftGPU> sift_;
};
} // namespace h2o

/*!
 * \file sift_matcher.h
 *
 * \author Han
 * \date 2017/05/04
 *
 *
 */
#pragma once
#include <base/common.h>
#include <match/recon_feature.h>
#include <match/recon_match.h>

// forward declaration
class SiftMatchGPU;

// forward declaration fastann 的索引
namespace fastann {
template <class Float> class nn_obj;
}

namespace h2o {
struct SiftMatcherParam {
    ///\brief nn ration 是距离的平方之比，因此0.7大概代表距离之比为0.85
    double nn_ratio = 0.8;
    double sac_threshold = 5.0; // pixels

    int ntree = 4;
    int nchecks = 128;

    // 0 - homography; 1 - fundemental, 2 - affine, 3 - translate， -1 - no ransac
    int model = 1;

    int max_matches = 4000;

    // 对 CPU 的无效
    int cross_check = 1;

    // spatial matching
    double buffer = 50.0;
    double sift_threshold = 256; // 8bit norm 到 512 了，大概相当于 0.5
};

class SiftMatcher {
public:
    SiftMatcher();
    virtual void set_train_data(const std::vector<cv::KeyPoint> &keys, const cv::Mat &desc);
    virtual void set_train_data(const FeaturePoints &features, const cv::Mat &desc);
    virtual void set_match_param(const SiftMatcherParam &param) { param_ = param; }
    virtual std::vector<cv::DMatch> match(const std::vector<cv::KeyPoint> &qkeys, const cv::Mat &qdesc);
    virtual std::vector<cv::DMatch> SiftMatcher::match_proposed(const std::vector<cv::KeyPoint> &qkeys,
                                                                const cv::Mat &qdesc,
                                                                std::vector<cv::KeyPoint> keys_ground,
                                                                std::vector<cv::KeyPoint> keys_render);
    virtual IndexMatches match(const FeaturePoints &features, const cv::Mat &desc);
    static cv::Mat draw_matches(const cv::Mat &mat_i, const cv::Mat &mat_j, const FeaturePoints &feat_i,
                                const FeaturePoints &feat_j, const IndexMatches &matches);

protected:
    IndexMatches acransac(const FeaturePoints &i_feat, const FeaturePoints &j_feat, const IndexMatches &initial);
    std::vector<cv::DMatch> retain_best_matches(const std::vector<cv::KeyPoint> &qkeys,
                                                const std::vector<cv::DMatch> &matches);

    SiftMatcherParam param_;
    int constexpr static SIFT_DIM = 128;
    std::vector<cv::KeyPoint> keys_;
    cv::Mat desc_;
    std::shared_ptr<fastann::nn_obj<uint8_t>> index_;
};

class SiftMatcherGpu {
public:
    SiftMatcherGpu();

    void set_match_param(const SiftMatcherParam &param);
    void set_train_data(const FeaturePoints &features, const cv::Mat &desc);
    IndexMatches match(const FeaturePoints &features, const cv::Mat &desc);

protected:
    IndexMatches acransac(const IndexMatches &initial, const FeaturePoints &feat1, const FeaturePoints &feat2);
    IndexMatches prosac(const IndexMatches &initial, const FeaturePoints &feat1, const FeaturePoints &feat2);

protected:
    FeaturePoints features_train_;

    std::shared_ptr<SiftMatchGPU> sift_matcher_;
    SiftMatcherParam param_;
};
} // namespace h2o

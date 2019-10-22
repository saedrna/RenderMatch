/*!
 * \file sift_matcher_cuda.h
 *
 * \author Han
 * \date 2017/07/07
 *
 * 采用 cuda 进行 sift match，这个比 siftgpu 的接口要方便
 */
#pragma once

#ifdef H2O_BUILD_CUDA
#include <cuda_runtime.h>

#include <base/common.h>
#include <base/recon_feature.h>
#include <match/sift_matcher.h>

namespace h2o {
class SiftMatcherCuda : public SiftMatcher {
public:
    SiftMatcherCuda();
    ~SiftMatcherCuda();
    virtual void set_train_data(const FeaturePoints &features, const cv::Mat &desc) override;
    virtual IndexMatches match(const FeaturePoints &features, const cv::Mat &desc) override;

protected:
    std::tuple<uint8_t *, cudaTextureObject_t> create_cuda_texture(const cv::Mat &desc);
    void destroy_cuda_texture(uint8_t *&d_data, cudaTextureObject_t &tex);

    SiftMatcherParam param_;
    FeaturePoints train_;

    ///\brief 创建 texture object
    cudaTextureObject_t tex_train_;
    uint8_t *d_train_;
};
} // namespace h2o

#endif
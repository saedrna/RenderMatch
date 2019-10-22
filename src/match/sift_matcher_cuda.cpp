/*!
 * \file sift_matcher_cuda.cpp
 *
 * \author Han
 * \date 2017/07/07
 *
 * 采用 cuda 进行 sift match，这个比 siftgpu 的接口要方便
 */

#ifdef H2O_BUILD_CUDA
#include <cuda_runtime.h>
#include <match/sift_matcher_cuda.h>

#include <cudaknn/CUDAKfNN_packed.h>

namespace h2o {
SiftMatcherCuda::SiftMatcherCuda() : SiftMatcher() {
    cudaDeviceSetCacheConfig(cudaFuncCachePreferL1);
    cudaDeviceSetSharedMemConfig(cudaSharedMemBankSizeEightByte);
}
SiftMatcherCuda::~SiftMatcherCuda() {
    if (tex_train_ != NULL) {
        cudaDestroyTextureObject(tex_train_);
        tex_train_ = NULL;
    }
    if (d_train_ != NULL) {
        cudaFree(d_train_);
        d_train_ = NULL;
    }
}
void SiftMatcherCuda::set_train_data(const FeaturePoints &features, const cv::Mat &desc) {
    train_ = features;
    std::tie(d_train_, tex_train_) = create_cuda_texture(desc);
}

IndexMatches SiftMatcherCuda::match(const FeaturePoints &features, const cv::Mat &desc) {
    int num_query = desc.rows;
    uint8_t *d_query;
    cudaTextureObject_t tex_query;
    std::tie(d_query, tex_query) = create_cuda_texture(desc);

    std::vector<uint32_t> h_matches(num_query);
    uint32_t *d_matches;
    cudaMalloc(&d_matches, num_query * sizeof(uint32_t));
    CUDAKfNN_packed(tex_train_, train_.size(), tex_query, num_query, d_matches, param_.nn_ratio);
    cudaMemcpy(h_matches.data(), d_matches, sizeof(uint32_t) * num_query, cudaMemcpyDeviceToHost);

    destroy_cuda_texture(d_query, tex_query);
    cudaFree(d_matches);

    IndexMatches matches;
    matches.reserve(num_query);
    for (int i = 0; i < num_query; ++i) {
        if (h_matches[i] != -1) {
            matches.emplace_back(h_matches[i], i);
        }
    }

    matches = acransac(train_, features, matches);

    if (matches.size() > param_.max_matches) {
        std::random_shuffle(begin(matches), end(matches));
        matches.resize(param_.max_matches);
    }
    std::sort(begin(matches), end(matches), [](const IndexMatch &m1, const IndexMatch &m2) { return m1.i < m2.i; });
    return matches;
}

std::tuple<uint8_t *, cudaTextureObject_t> SiftMatcherCuda::create_cuda_texture(const cv::Mat &desc) {
    int num_feature = desc.rows;
    uint8_t *d_desc;
    cudaMalloc(&d_desc, desc.total());
    cudaMemcpy(d_desc, desc.data, desc.total(), cudaMemcpyHostToDevice);
    cudaResourceDesc resDesc;
    memset(&resDesc, 0, sizeof(resDesc));
    resDesc.resType = cudaResourceTypeLinear;
    resDesc.res.linear.devPtr = d_desc;
    resDesc.res.linear.desc.f = cudaChannelFormatKindUnsigned;
    resDesc.res.linear.desc.x = resDesc.res.linear.desc.y = resDesc.res.linear.desc.z = resDesc.res.linear.desc.w = 32;
    resDesc.res.linear.sizeInBytes = 128 * num_feature;
    struct cudaTextureDesc texDesc;
    memset(&texDesc, 0, sizeof(texDesc));
    texDesc.addressMode[0] = texDesc.addressMode[1] = texDesc.addressMode[2] = texDesc.addressMode[3] =
        cudaAddressModeBorder;
    texDesc.filterMode = cudaFilterModePoint;
    texDesc.readMode = cudaReadModeElementType;
    texDesc.normalizedCoords = 0;
    cudaTextureObject_t tex_q = 0;
    cudaCreateTextureObject(&tex_q, &resDesc, &texDesc, nullptr);
    return std::make_tuple(d_desc, tex_q);
}

void SiftMatcherCuda::destroy_cuda_texture(uint8_t *&d_data, cudaTextureObject_t &tex) {
    if (d_data != NULL) {
        cudaFree(d_data);
        d_data = NULL;
    }
    if (tex != NULL) {
        cudaDestroyTextureObject(tex);
        tex = NULL;
    }
}
} // namespace h2o

#endif // CEL_HAS_CUDA

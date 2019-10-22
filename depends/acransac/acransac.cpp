#include "acransac.h"
#include "numeric.h"
#include "robust_estimator_ACRansac.hpp"
#include "robust_estimator_ACRansacKernelAdaptator.hpp"
#include "solver_fundamental_kernel.hpp"
#include "solver_homography_kernel.hpp"
#include "solver_translation_kernel.hpp"
using namespace std;
namespace acransac {

using namespace openMVG;
typedef struct ImageSize_ {
    int w1, h1, w2, h2;
    ImageSize_() { w1 = h1 = w2 = h2 = -1; }
} ImageSize;
ImageSize matches2points(const vector<cv::DMatch> &matches, const vector<cv::KeyPoint> &kpts_train,
                         const vector<cv::KeyPoint> &kpts_query, openMVG::Mat &pts_train, openMVG::Mat &pts_query) {

    int nmatches = matches.size();
    pts_train = openMVG::Mat(2, nmatches);
    pts_query = openMVG::Mat(2, nmatches);

    ImageSize size;

    for (size_t i = 0; i < matches.size(); i++) {
        const cv::DMatch &match = matches[i];
        pts_train(0, i) = kpts_train[match.trainIdx].pt.x;
        pts_train(1, i) = kpts_train[match.trainIdx].pt.y;

        pts_query(0, i) = kpts_query[match.queryIdx].pt.x;
        pts_query(1, i) = kpts_query[match.queryIdx].pt.y;

        size.w1 = std::max(size.w1, (int)ceil(pts_train(0, i)));
        size.h1 = std::max(size.h1, (int)ceil(pts_train(1, i)));
        size.w2 = std::max(size.w2, (int)ceil(pts_query(0, i)));
        size.h2 = std::max(size.h2, (int)ceil(pts_query(1, i)));
    }
    return size;
}

void acransac_homography(const vector<cv::DMatch> &matches, const vector<cv::KeyPoint> &kpts_train,
                         const vector<cv::KeyPoint> &kpts_query, vector<cv::DMatch> &matchout) {
    srand(1314);
    openMVG::Mat train_2f, query_2f;
    ImageSize size = matches2points(matches, kpts_train, kpts_query, train_2f, query_2f);
    typedef openMVG::robust::ACKernelAdaptor<openMVG::homography::kernel::FourPointSolver,
                                             openMVG::homography::kernel::AsymmetricError, openMVG::UnnormalizerI,
                                             openMVG::Mat3>
        KernelType;
    KernelType kernel(train_2f, size.w1, size.h1, query_2f, size.w2, size.h2, false);
    vector<size_t> inliners;

    openMVG::robust::ACRANSAC(kernel, inliners);
    vector<cv::DMatch> matches_filter(inliners.size());
    for (int i = 0; i < inliners.size(); ++i) {
        matches_filter[i] = matches[inliners[i]];
    }
    matchout = matches_filter;
}

std::vector<cv::DMatch> acransac_homography(const std::vector<cv::DMatch> &matches,
                                            const std::vector<cv::KeyPoint> &kpts_train,
                                            const std::vector<cv::KeyPoint> &kpts_query, double threshold) {
    srand(1314);
    openMVG::Mat train_2f, query_2f;
    ImageSize size = matches2points(matches, kpts_train, kpts_query, train_2f, query_2f);
    typedef openMVG::robust::ACKernelAdaptor<openMVG::homography::kernel::FourPointSolver,
                                             openMVG::homography::kernel::AsymmetricError, openMVG::UnnormalizerI,
                                             openMVG::Mat3>
        KernelType;
    KernelType kernel(train_2f, size.w1, size.h1, query_2f, size.w2, size.h2, false);
    vector<size_t> inliners;

    openMVG::robust::ACRANSAC(kernel, inliners, 1024, NULL, threshold * threshold, false);
    vector<cv::DMatch> matches_filter(inliners.size());
    for (int i = 0; i < inliners.size(); ++i) {
        matches_filter[i] = matches[inliners[i]];
    }
    return matches_filter;
}

void acransac_fundamental(const vector<cv::DMatch> &matches, const vector<cv::KeyPoint> &kpts_train,
                          const vector<cv::KeyPoint> &kpts_query, vector<cv::DMatch> &matchout) {
    srand(1314);
    openMVG::Mat train_2f, query_2f;
    ImageSize size = matches2points(matches, kpts_train, kpts_query, train_2f, query_2f);
    typedef robust::ACKernelAdaptor<fundamental::kernel::SevenPointSolver,
                                    fundamental::kernel::SymmetricEpipolarDistanceError, UnnormalizerT, Mat3>
        KernelType;
    KernelType kernel(train_2f, size.w1, size.h1, query_2f, size.w2, size.h2, false);

    vector<size_t> inliners;

    openMVG::robust::ACRANSAC(kernel, inliners);
    vector<cv::DMatch> matches_filter(inliners.size());
    for (int i = 0; i < inliners.size(); ++i) {
        matches_filter[i] = matches[inliners[i]];
    }
    matchout = matches_filter;
}
std::vector<cv::DMatch> acransac_fundamental(const std::vector<cv::DMatch> &matches,
                                             const std::vector<cv::KeyPoint> &kpts_train,
                                             const std::vector<cv::KeyPoint> &kpts_query, double threshold) {
    srand(1314);
    openMVG::Mat train_2f, query_2f;
    ImageSize size = matches2points(matches, kpts_train, kpts_query, train_2f, query_2f);
    typedef robust::ACKernelAdaptor<fundamental::kernel::SevenPointSolver,
                                    fundamental::kernel::SymmetricEpipolarDistanceError, UnnormalizerT, Mat3>
        KernelType;
    KernelType kernel(train_2f, size.w1, size.h1, query_2f, size.w2, size.h2, true);

    vector<size_t> inliners;

    openMVG::robust::ACRANSAC(kernel, inliners, 1024, NULL, threshold * threshold, false);
    vector<cv::DMatch> matches_filter(inliners.size());
    for (int i = 0; i < inliners.size(); ++i) {
        matches_filter[i] = matches[inliners[i]];
    }
    return matches_filter;
}
std::vector<cv::DMatch> acransac_translation(const std::vector<cv::DMatch> &matches,
                                             const std::vector<cv::KeyPoint> &kpts_train,
                                             const std::vector<cv::KeyPoint> &kpts_query, double threshold) {
    srand(1314);
    openMVG::Mat train_2f, query_2f;
    ImageSize size = matches2points(matches, kpts_train, kpts_query, train_2f, query_2f);
    typedef robust::ACKernelAdaptor<translation::kernel::Translation2dSolver, translation::kernel::AsymmetricError,
                                    UnnormalizerT, Mat3>
        KernelType;

    KernelType kernel(train_2f, size.w1, size.h1, query_2f, size.w2, size.h2, false);
    vector<size_t> inliners;

    Mat3 model;
    openMVG::robust::ACRANSAC(kernel, inliners, 1024, &model, threshold * threshold, true);
    vector<cv::DMatch> matches_filter(inliners.size());
    for (int i = 0; i < inliners.size(); ++i) {
        matches_filter[i] = matches[inliners[i]];
    }
    return matches_filter;
}
} // namespace acransac

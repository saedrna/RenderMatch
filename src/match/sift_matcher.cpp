/*!
 * \file sift_matcher.cpp
 *
 * \author Han
 * \date 2017/05/04
 *
 * Sift 采用 ann 匹配
 */
#include <match/sift_matcher.h>

#include <acransac/acransac.h>
#include <fastann/fastann.hpp>
#include <siftgpu/SiftGPU.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/flann/kdtree_index.h>
#include <opencv2/flann/matrix.h>

#include <iostream>
#include <fstream>

namespace h2o {
SiftMatcher::SiftMatcher() {}
void SiftMatcher::set_train_data(const std::vector<cv::KeyPoint> &keys, const cv::Mat &desc) {
    keys_ = keys;
    desc_ = desc.clone();
    index_.reset(fastann::nn_obj_build_kdtree<uint8_t>(desc_.data, desc_.rows, SIFT_DIM, param_.ntree, param_.nchecks));
}

void SiftMatcher::set_train_data(const FeaturePoints &features, const cv::Mat &desc) {
    std::vector<cv::KeyPoint> keys(features.size());
    std::transform(begin(features), end(features), begin(keys),
                   [](const Vector2f &feat) { return cv::KeyPoint(feat.x(), feat.y(), 9.0); });
    set_train_data(keys, desc);
}

//line intersection check
bool lineIntersection(cv::DMatch match1, cv::DMatch match2, std::vector<cv::KeyPoint> queryKeys,
                      std::vector<cv::KeyPoint> trainKeys) {
    // l1->[dist,x1,y1,x2,y2],l2->[dist,x3,y3,x4,y4]
    float x1 = queryKeys[match1.queryIdx].pt.x, y1 = queryKeys[match1.queryIdx].pt.y,
          x2 = trainKeys[match1.trainIdx].pt.x, y2 = trainKeys[match1.trainIdx].pt.y;
    float x3 = queryKeys[match2.queryIdx].pt.x, y3 = queryKeys[match2.queryIdx].pt.y,
          x4 = trainKeys[match2.trainIdx].pt.x, y4 = trainKeys[match2.trainIdx].pt.y;
    float k1, k2, b1, b2, xi, yi;
    if (x1 == x2) {
        if (x3 > x1 && x4 > x1) {
            return false;
        }
    } else if (x3 == x4) {
        if (x1 > x3 && x2 > x3) {
            return false;
        }
    } else {
        k1 = (y2 - y1) / (x2 - x1);
        k2 = (y4 - y3) / (x4 - x3);
        b1 = y1 - k1 * x1;
        b2 = y3 - k2 * x3;
        xi = -(b1 - b2) / (k1 - k2);
        yi = k1 * xi + b1;
        // std::cout << xi<<"  " << yi << std::endl;
        if ((xi - x1) * (xi - x2) > 0 || (xi - x3) * (xi - x4) > 0) {
            return false;
        } else {
            return true;
        }
    }
}

std::vector<cv::DMatch> SiftMatcher::match_ransac(const std::vector<cv::KeyPoint> &qkeys, const cv::Mat &qdesc) {

    CHECK(qkeys.size() == qdesc.rows);
    const int K = 2;
    const int N = qdesc.rows;
    std::vector<uint32_t> argmins(K * N, (uint32_t)-1), mins(K * N, (uint32_t)-1);
    index_->search_knn(qdesc.data, qdesc.rows, 2, argmins.data(), mins.data());

    std::vector<cv::DMatch> matches;
    matches.reserve(N);

    for (int i = 0; i < N; ++i) {
        if ((float)mins[i * K] / mins[i * K + 1] < param_.nn_ratio) {
            matches.emplace_back(i, argmins[i * K], (float)mins[i * K]);
        }
    }

    if (param_.model == 0) {
        matches = acransac::acransac_homography(matches, keys_, qkeys, param_.sac_threshold);
    } else if (param_.model == 1) {
        matches = acransac::acransac_fundamental(matches, keys_, qkeys, param_.sac_threshold);
    } else if (param_.model == 3) {
        matches = acransac::acransac_translation(matches, keys_, qkeys, param_.sac_threshold);
    }
    return retain_best_matches(qkeys, matches);
}

//  local geometry constraints for outlier removal
std::vector<cv::DMatch> SiftMatcher::match_proposed(const std::vector<cv::KeyPoint> &qkeys, const cv::Mat &qdesc,
                                                   std::vector<cv::KeyPoint> keys_ground,
                                                   std::vector<cv::KeyPoint> keys_render) {

    std::vector<cv::DMatch> lenMatches, matchesOut;
    CHECK(qkeys.size() == qdesc.rows);
    const int K = 2;
    const int N = qdesc.rows;
    std::vector<uint32_t> argmins(K * N, (uint32_t)-1), mins(K * N, (uint32_t)-1);
    index_->search_knn(qdesc.data, qdesc.rows, 2, argmins.data(), mins.data());

    std::vector<cv::DMatch> matches;
    matches.reserve(N);

    for (int i = 0; i < N; ++i) {
        if ((float)mins[i * K] / mins[i * K + 1] < param_.nn_ratio) {
            matches.emplace_back(i, argmins[i * K], (float)mins[i * K]);
        }
    }

    std::vector<std::vector<float>> dists;
    std::vector<float> distance;
    std::vector<int> frequency;
    std::vector<float> orients;

    /*Consistency1. Scale*/
    {
        std::vector<cv::DMatch> tmatches;
        for (int i = 0; i < matches.size(); i++) {
            auto match = matches[i];
            auto key_ground = keys_ground.at(match.trainIdx);
            auto key_render = keys_render.at(match.queryIdx);
            float keyDist = std::sqrt(std::powf((key_ground.pt.x - key_render.pt.x), 2) +
                                      std::powf((key_ground.pt.y - key_render.pt.y), 2));
            distance.push_back(keyDist);
        }

        auto itc = matches.begin();
        auto itcd = distance.begin();
        for (int i = 0; i < matches.size(); i++) {
            if (distance[i] < 120) {
                tmatches.push_back(matches[i]);
            }
        }
        matches = tmatches;
    }

    {
        using namespace cv;
        std::vector<cv::DMatch> tmatches;
        orients.resize(matches.size());
        // build index
        int iTargetNum = matches.size();
        cvflann::Matrix<float> data(new float[iTargetNum * 2], iTargetNum, 2);
        for (int i = 0; i < matches.size(); i++) {
            data[i][0] = keys_ground.at(matches[i].trainIdx).pt.x;
            data[i][1] = keys_ground.at(matches[i].trainIdx).pt.y;
        }
        cvflann::Index<flann::L2_Simple<float>> index(data, cvflann::KDTreeSingleIndexParams(15));
        index.buildIndex();

        // search knn
        int knn;
        if (matches.size() > 10) {
            knn = 5;
        } else
            knn = matches.size() / 2;

        cvflann::Matrix<float> p(new float[iTargetNum * 2], iTargetNum, 2);
        for (int i = 0; i < iTargetNum; i++) {
            p[i][0] = keys_ground.at(matches[i].trainIdx).pt.x;
            p[i][1] = keys_ground.at(matches[i].trainIdx).pt.y;
        }
        cvflann::Matrix<int> indices(new int[iTargetNum * knn], iTargetNum, knn);
        cvflann::Matrix<float> keyDists(new float[iTargetNum * knn], iTargetNum, knn);
        std::vector<float> vIndices, vDists;
        vIndices.resize(iTargetNum), vDists.resize(iTargetNum);
        index.knnSearch(p, indices, keyDists, knn, cvflann::SearchParams()); // indices:n个邻近点索引下标, keyDist:对应的距离

		/*Consistency2. Frequency*/
		{
            std::vector<cv::DMatch> tmatches;
            frequency.resize(matches.size());
            int countNum = 0;
            for (int i = 0; i < matches.size(); i++) {
                for (int j = 0; j < indices.cols; j++) {
                    if (lineIntersection(matches[indices[i][j]], matches[j], keys_render, keys_ground)) {
                        frequency[i]++;
                        frequency[indices[i][j]]++;
                    }
                }
            }
            auto itc = matches.begin();
            auto itcf = frequency.begin();
            for (int i = 0; i < matches.size(); i++) {
                if (frequency[i] ==0) {
                    tmatches.push_back(matches[i]);
                }
            }
            matches = tmatches;
		}

		/*Consistency3. Orientation*/
        for (int i = 0; i < matches.size(); i++) {
            float x1 = keys_render[matches[i].queryIdx].pt.x, y1 = keys_render[matches[i].queryIdx].pt.y,
                  x2 = keys_ground[matches[i].trainIdx].pt.x, y2 = keys_ground[matches[i].trainIdx].pt.y;

            for (int j = 0; j < indices.cols; j++) {
                float x3 = keys_render[matches[indices[i][j]].queryIdx].pt.x,
                      y3 = keys_render[matches[indices[i][j]].queryIdx].pt.y,
                      x4 = keys_ground[matches[indices[i][j]].trainIdx].pt.x,
                      y4 = keys_ground[matches[indices[i][j]].trainIdx].pt.y;
                orients[i] += (x2 - x1) * (x4 - x3) + (y2 - y1) * (y4 - y3); //取邻近的5对匹配对算与主方向的偏差值
            }
        }
        data.free(), indices.free(), keyDists.free();

        for (int i = 0; i < matches.size(); i++) {
            if (orients[i] > 0) {
                tmatches.push_back(matches[i]);
            }
        }
        matches = tmatches;
    }

    if (param_.model == 0) {
        matches = acransac::acransac_homography(matches, keys_, qkeys, param_.sac_threshold);
    } else if (param_.model == 1) {
        matches = acransac::acransac_fundamental(matches, keys_, qkeys, param_.sac_threshold);
    } else if (param_.model == 3) {
        matches = acransac::acransac_translation(matches, keys_, qkeys, param_.sac_threshold);
    } else if (param_.model == 2) {
        matches = acransac::acransac_fundamental(matches, keys_, qkeys, param_.sac_threshold);
    }
    return retain_best_matches(qkeys, matches);
}

IndexMatches SiftMatcher::match(const FeaturePoints &features, const cv::Mat &desc) {
    std::vector<cv::KeyPoint> keys(features.size());
    std::transform(begin(features), end(features), begin(keys),
                   [](const Vector2f &feat) { return cv::KeyPoint(feat.x(), feat.y(), 9.0); });
    std::vector<cv::DMatch> matches = match_ransac(keys, desc);
    IndexMatches matches_out(matches.size());
    std::transform(begin(matches), end(matches), begin(matches_out),
                   [](const cv::DMatch &dmatch) { return IndexMatch(dmatch.trainIdx, dmatch.queryIdx); });
    return matches_out;
}

cv::Mat SiftMatcher::draw_matches(const cv::Mat &mat_i, const cv::Mat &mat_j, const FeaturePoints &feat_i,
                                  const FeaturePoints &feat_j, const IndexMatches &matches) {
    std::vector<cv::KeyPoint> keys_i(feat_i.size()), keys_j(feat_j.size());
    for (int i = 0; i < feat_i.size(); ++i) {
        keys_i[i].pt.x = feat_i[i].x();
        keys_i[i].pt.y = feat_i[i].y();
    }
    for (int i = 0; i < feat_j.size(); ++i) {
        keys_j[i].pt.x = feat_j[i].x();
        keys_j[i].pt.y = feat_j[i].y();
    }
    std::vector<cv::DMatch> dmatches(matches.size());
    for (int i = 0; i < matches.size(); ++i) {
        dmatches[i].queryIdx = matches[i].i;
        dmatches[i].trainIdx = matches[i].j;
    }

    if (mat_i.channels() == 1) {
        cv::Mat mat_i_rgb;
        cv::Mat mat_j_rgb;
        cv::cvtColor(mat_i, mat_i_rgb, cv::COLOR_GRAY2BGR);
        cv::cvtColor(mat_j, mat_j_rgb, cv::COLOR_GRAY2BGR);
        cv::Mat output;
        cv::drawMatches(mat_i_rgb, keys_i, mat_j_rgb, keys_j, dmatches, output, cv::Scalar(0, 255, 255),
                        cv::Scalar(255, 0, 0), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        return output;
    }
    if (mat_i.channels() == 3) {
        cv::Mat output;
        cv::drawMatches(mat_i, keys_i, mat_j, keys_j, dmatches, output, cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0),
                        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        return output;
    }

    return cv::Mat();
}

IndexMatches SiftMatcher::acransac(const FeaturePoints &i_feat, const FeaturePoints &j_feat,
                                   const IndexMatches &initial) {
    std::vector<cv::KeyPoint> i_keys(i_feat.size()), j_keys(j_feat.size());
    std::vector<cv::DMatch> matches(initial.size());

    std::transform(begin(i_feat), end(i_feat), begin(i_keys),
                   [](const Vector2f &feat) { return cv::KeyPoint(feat.x(), feat.y(), 1.0); });
    std::transform(begin(j_feat), end(j_feat), begin(j_keys),
                   [](const Vector2f &feat) { return cv::KeyPoint(feat.x(), feat.y(), 1.0); });
    std::transform(begin(initial), end(initial), begin(matches),
                   [](const IndexMatch &index) { return cv::DMatch(index.j, index.i, 1.0f); });

    if (param_.model == 0) {
        matches = acransac::acransac_homography(matches, i_keys, j_keys, param_.sac_threshold);
    } else if (param_.model == 1) {
        matches = acransac::acransac_fundamental(matches, i_keys, j_keys, param_.sac_threshold);
    } else if (param_.model == 3) {
        matches = acransac::acransac_translation(matches, i_keys, j_keys, param_.sac_threshold);
    }

    IndexMatches out_matches(matches.size());
    std::transform(begin(matches), end(matches), begin(out_matches),
                   [](const cv::DMatch &match) { return IndexMatch(match.trainIdx, match.queryIdx); });
    if (out_matches.size() < 15) {
        return IndexMatches();
    }
    return out_matches;
}

std::vector<cv::DMatch> SiftMatcher::retain_best_matches(const std::vector<cv::KeyPoint> &qkeys,
                                                         const std::vector<cv::DMatch> &matches) {

    if (matches.size() > param_.max_matches) {
        std::vector<cv::DMatch> filtered = matches;
        std::partial_sort(begin(filtered), begin(filtered) + param_.max_matches, end(filtered),
                          [&](const cv::DMatch &m1, const cv::DMatch &m2) {
                              double response1 = 0.0;
                              int idx_train = m1.trainIdx;
                              int idx_query = m1.queryIdx;
                              response1 = keys_[idx_train].response + qkeys[idx_query].response;

                              double response2 = 0.0;
                              idx_train = m2.trainIdx;
                              idx_query = m2.queryIdx;
                              response2 = keys_[idx_train].response + qkeys[idx_query].response;

                              return response1 > response2;
                          });
        filtered.resize(param_.max_matches);
        return filtered;
    } else {
        return matches;
    }
}

SiftMatcherGpu::SiftMatcherGpu() {
    sift_matcher_ = std::make_shared<SiftMatchGPU>();
    sift_matcher_->SetLanguage(SiftMatchGPU::SIFTMATCH_GLSL);
    CHECK(sift_matcher_->CreateContextGL() == SiftGPU::SIFTGPU_FULL_SUPPORTED);
    CHECK(sift_matcher_->VerifyContextGL());
}

void SiftMatcherGpu::set_match_param(const SiftMatcherParam &param) {
    param_ = param;
    sift_matcher_->SetMaxSift(param.max_matches);
}

void SiftMatcherGpu::set_train_data(const FeaturePoints &features, const cv::Mat &desc) {
    sift_matcher_->SetDescriptors(0, desc.rows, (const uint8_t *)desc.data);
    features_train_ = features;
}

IndexMatches SiftMatcherGpu::match(const FeaturePoints &features, const cv::Mat &desc) {
    sift_matcher_->SetDescriptors(1, desc.rows, (const uint8_t *)desc.data);

    IndexMatches matches(param_.max_matches);

    int num_matches = sift_matcher_->GetSiftMatch(param_.max_matches, (uint32_t(*)[2])matches.data(), 0.7f,
                                                  param_.nn_ratio, param_.cross_check);
    CHECK(num_matches >= 0);
    matches.resize(num_matches);
    return prosac(matches, features_train_, features);
}

IndexMatches SiftMatcherGpu::acransac(const IndexMatches &initial, const FeaturePoints &feat1,
                                      const FeaturePoints &feat2) {
    std::vector<cv::KeyPoint> keys1(feat1.size()), keys2(feat2.size());
    std::vector<cv::DMatch> matches(initial.size());

    std::transform(begin(feat1), end(feat1), begin(keys1),
                   [](const Vector2f &feat) { return cv::KeyPoint(feat.x(), feat.y(), 1.0); });
    std::transform(begin(feat2), end(feat2), begin(keys2),
                   [](const Vector2f &feat) { return cv::KeyPoint(feat.x(), feat.y(), 1.0); });
    std::transform(begin(initial), end(initial), begin(matches),
                   [](const IndexMatch &index) { return cv::DMatch(index.j, index.i, 1.0f); });

    if (param_.model == 0) {
        matches = acransac::acransac_homography(matches, keys1, keys2, param_.sac_threshold);
    } else if (param_.model == 1) {
        matches = acransac::acransac_fundamental(matches, keys1, keys2, param_.sac_threshold);
    } else if (param_.model == 3) {
        matches = acransac::acransac_translation(matches, keys1, keys2, param_.sac_threshold);
    }

    IndexMatches out_matches(matches.size());
    std::transform(begin(matches), end(matches), begin(out_matches),
                   [](const cv::DMatch &match) { return IndexMatch(match.trainIdx, match.queryIdx); });
    if (out_matches.size() < 15) {
        return IndexMatches();
    }
    return out_matches;
}
IndexMatches SiftMatcherGpu::prosac(const IndexMatches &initial, const FeaturePoints &feat1,
                                    const FeaturePoints &feat2) {
    std::vector<cv::Point2f> points1(initial.size()), points2(initial.size());
    for (int i = 0; i < initial.size(); ++i) {
        points1[i] = cv::Point2f(feat1[initial[i].i].x(), feat1[initial[i].i].y());
        points2[i] = cv::Point2f(feat2[initial[i].j].x(), feat2[initial[i].j].y());
    }
    std::vector<uint8_t> mask(initial.size(), false);
    if (param_.model == 0) {
        cv::findHomography(points1, points2, cv::RANSAC, param_.sac_threshold, mask);
    } else if (param_.model == 1) {
        cv::findFundamentalMat(points1, points2, mask, cv::RANSAC, param_.sac_threshold);
    }

    IndexMatches matches_out;
    matches_out.reserve(mask.size());
    for (int i = 0; i < mask.size(); ++i) {
        if (mask[i]) {
            matches_out.push_back(initial[i]);
        }
    }

    if (matches_out.size() < 15) {
        return IndexMatches();
    }
    return matches_out;
}
} // namespace h2o

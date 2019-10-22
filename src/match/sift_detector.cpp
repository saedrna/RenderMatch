/*!
 * \file sift_detector.cpp
 *
 * \author Han
 * \date 2017/05/04
 *
 *
 */
#include <fstream>

#include <base/base.h>
#include <match/sift_detector.h>

#include <GL/glew.h>

#include <siftgpu/SiftGPU.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
namespace h2o {

/**
 * \brief       retainBestFeatures 保留feature response最大的n个点
 * \param[in,out]   std::vector<cv::KeyPoint> & keys 特征点
 * \param[in,out]   cv::Mat & desc 特征描述符
 * \param[in]   int nfeatures 特征点数目
 */
void retain_best_features(std::vector<cv::KeyPoint> &keys, cv::Mat &desc, int nfeatures) {
    bool has_descritor = !desc.empty();
    if (nfeatures >= 0 && keys.size() > nfeatures) {
        if (nfeatures == 0) {
            keys.clear();
            desc = cv::Mat();
            return;
        }
        vector<int> idx(keys.size());
        for (int i = 0; i < keys.size(); ++i)
            idx[i] = i;

        //这里先把点的序号分成两部分，前面N个ID对应的特征点的response更大
        std::nth_element(begin(idx), begin(idx) + nfeatures, end(idx),
                         [&keys](const int &i1, const int &i2) { return keys[i1].response > keys[i2].response; });

        float boundary = keys[idx[nfeatures - 1]].response;

        auto new_end = std::partition(begin(idx) + nfeatures, end(idx),
                                      [&boundary, &keys](const int &i) { return keys[i].response >= boundary; });
        idx.resize(new_end - begin(idx));

        vector<cv::KeyPoint> new_keys(idx.size());
        cv::Mat new_desc(idx.size(), desc.cols, desc.type());

        for (int i = 0; i < idx.size(); ++i) {
            new_keys[i] = keys[idx[i]];
            if (has_descritor) {
                desc.row(idx[i]).copyTo(new_desc.row(i));
            }
        }
        keys = new_keys;
        if (has_descritor) {
            desc = new_desc;
        }
    }
}

void retain_best_features_gridded(std::vector<cv::KeyPoint> &keys, cv::Mat &desc, int nfeatures) {
    bool has_descriptor = !desc.empty();
    if (nfeatures >= 0 && keys.size() > nfeatures) {
        // 找到最大格网
        BoundingBox2d bbox;

        for (const auto &kp : keys) {
            bbox.extend(Vector2d(kp.pt.x, kp.pt.y));
        }

        int num_pixels = bbox.sizes()(0) * bbox.sizes()(1);
        double num_pixels_per_feature = sqrt(double(num_pixels) / nfeatures) * 0.75;

        if (num_pixels_per_feature < 5.0) {
            num_pixels_per_feature = 5.0;
        }

        int num_rows = int(bbox.sizes()(1) / num_pixels_per_feature) + 1;
        int num_cols = int(bbox.sizes()(0) / num_pixels_per_feature) + 1;

        vector<vector<int>> grid_indices(num_rows * num_cols);
        int feat = 0;
        for (const auto &kp : keys) {
            int row, col;
            row = int((kp.pt.y - bbox.min()(1)) / num_pixels_per_feature);
            col = int((kp.pt.x - bbox.min()(0)) / num_pixels_per_feature);
            row = clip(row, 0, num_rows - 1);
            col = clip(col, 0, num_cols - 1);

            int grid = row * num_cols + col;

            grid_indices[grid].push_back(feat++);
        }

        // 对每个grid，保留一个最大的feature
        vector<int> idx;
        idx.reserve(num_rows * num_cols);
        for (auto indices : grid_indices) {
            if (indices.empty())
                continue;
            if (indices.size() >= 2) {
                std::sort(begin(indices), end(indices),
                          [&](int idx1, int idx2) { return keys[idx1].response > keys[idx2].response; });
            }
            idx.push_back(indices[0]);
        }

        std::sort(begin(idx), end(idx), [&](int id1, int id2) { return keys[id1].response > keys[id2].response; });

        vector<cv::KeyPoint> new_keys(idx.size());
        cv::Mat new_desc(idx.size(), desc.cols, desc.type());

        for (int i = 0; i < idx.size(); ++i) {
            new_keys[i] = keys[idx[i]];
            if (has_descriptor) {
                desc.row(idx[i]).copyTo(new_desc.row(i));
            }
        }
        keys = new_keys;
        if (has_descriptor) {
            desc = new_desc;
        }
    }
}

void filter_features_by_mask(std::vector<cv::KeyPoint> &keys, cv::Mat &desc, const cv::Mat &mask) {
    bool has_descriptor = !desc.empty();

    std::vector<cv::KeyPoint> new_keys;
    cv::Mat new_desc;

    new_keys.reserve(keys.size());

    int rows = mask.rows;
    int cols = mask.cols;

    std::vector<int> keep;
    keep.reserve(keys.size());

    for (int i = 0; i < keys.size(); ++i) {
        cv::KeyPoint key = keys[i];
        int r = lround(key.pt.y);
        int c = lround(key.pt.x);

        if (r < 0 || r >= rows || c < 0 || c >= cols) {
            continue;
        }
        if (mask.at<uint8_t>(r, c) == 0) {
            continue;
        }

        keep.push_back(i);
    }

    new_keys.resize(keep.size());
    if (has_descriptor) {
        new_desc = cv::Mat::zeros(keep.size(), desc.cols, desc.type());
    }

    for (int k = 0; k < keep.size(); ++k) {
        new_keys[k] = keys[keep[k]];
        if (has_descriptor) {
            desc.row(keep[k]).copyTo(new_desc.row(k));
        }
    }

    if (has_descriptor) {
        desc = new_desc;
    }
    keys = new_keys;
}

void sift_save(const string &path, const std::vector<cv::KeyPoint> &kpts, const cv::Mat &desc) {
    CHECK(kpts.size() == desc.rows) << "keypoint and descriptor should have the same size";
    CHECK(desc.cols == 128) << "descriptor should have 128 dims";
    int dim = 128;
    int num_keys = kpts.size();
    std::ofstream keypoint_file(string_utf8_to_fstream(path), std::ios::binary);
    int keys_size = num_keys * sizeof(cv::KeyPoint);
    int desc_size = dim * num_keys;

    keypoint_file.write((const char *)&num_keys, sizeof(int));
    keypoint_file.write((const char *)kpts.data(), keys_size);
    keypoint_file.write((const char *)desc.data, desc_size);
    keypoint_file.close();
}

void sift_read(const string &path, std::vector<cv::KeyPoint> &kpts, cv::Mat &desc) {
    std::ifstream keypoint_file(string_utf8_to_fstream(path), std::ios::binary);
    int dim = 128;
    int num_keys = 0;
    keypoint_file.read((char *)&num_keys, sizeof(int));

    kpts.resize(num_keys);
    desc = cv::Mat::zeros(num_keys, dim, CV_8U);

    int keys_size = num_keys * sizeof(cv::KeyPoint);
    int desc_size = dim * num_keys;

    keypoint_file.read((char *)kpts.data(), keys_size);
    keypoint_file.read((char *)desc.data, desc_size);
    keypoint_file.close();
}

void match_save(const std::string &path, const std::vector<cv::DMatch> &matches) {
    int nmatch = matches.size();

    std::ofstream ofile(string_utf8_to_fstream(path), std::ios::binary);
    int match_size = nmatch * sizeof(cv::DMatch);
    ofile.write((const char *)&nmatch, sizeof(int));
    ofile.write((const char *)matches.data(), match_size);
    ofile.close();
}

void match_read(const std::string &path, std::vector<cv::DMatch> &matches) {
    int nmatch;
    std::ifstream ifile(string_utf8_to_fstream(path), std::ios::binary);
    ifile.read((char *)&nmatch, sizeof(int));
    matches.resize(nmatch);
    int match_size = nmatch * sizeof(cv::DMatch);
    ifile.read((char *)matches.data(), match_size);
    ifile.close();
}

SiftDetector::SiftDetector(int opt, int nfeat) {
    sift_ = std::make_shared<SiftGPU>();
    float tdog;
    int fo = 0;

    if (opt == 0) {
        tdog = 0.04f;
        fo = 0;
    } else if (opt == 1) {
        tdog = 0.02f / 3;
        fo = 0;
    } else if (opt == 2) {
        tdog = 0.02f / 3;
        fo = -1;
    }
    string stdog = string_printf("%.6f", tdog);
    string sfo = string_printf("%d", fo);
    string sfeat = string_printf("%d", nfeat);

    nfeat_ = nfeat;
    const char *argv[] = {"-loweo", "-v",
                          "0",      "-ofix-not",
                          "-tc2",   (char *)sfeat.c_str(),
                          "-fo",    (char *)sfo.c_str(),
                          "-t",     (char *)stdog.c_str()};
    int argc = sizeof(argv) / sizeof(char *);

    sift_->ParseParam(argc, (char **)argv);

    CHECK(sift_->CreateContextGL() == SiftGPU::SIFTGPU_FULL_SUPPORTED);
    CHECK(sift_->VerifyContextGL());
}

SiftDetector::~SiftDetector() {}

std::tuple<std::vector<cv::KeyPoint>, cv::Mat> SiftDetector::detect_and_compute(const cv::Mat &img) {
    std::vector<cv::KeyPoint> keys;
    cv::Mat desc;
    detect_and_compute(img, keys, desc);
    return std::make_tuple(keys, desc);
}

void SiftDetector::detect_and_compute(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, cv::Mat &desc) {
    kpts.resize(0);
    desc = cv::Mat();
    int grid_row, grid_col;
    int rows(img.rows), cols(img.cols);

    float cell_size = 2048.0f;
    grid_row = std::ceil((float)rows / cell_size);
    grid_col = std::ceil((float)cols / cell_size);

    const float RAD2DEG = 180.0f / 3.141592653589f;
    int nfeat_per_grid = (1.5 * nfeat_) / (grid_row * grid_col);
    for (int i = 0; i < grid_row * grid_col; ++i) {
        int celly = i / grid_col;
        int cellx = i - celly * grid_col;

        cv::Range row_range((celly * img.rows) / grid_row, ((celly + 1) * img.rows) / grid_row);
        cv::Range col_range((cellx * img.cols) / grid_col, ((cellx + 1) * img.cols) / grid_col);

        cv::Mat sub_image = img(row_range, col_range).clone();
        int channel = sub_image.channels();
        vector<cv::KeyPoint> sub_keypoints;
        vector<float> vsub_descriptor;
        cv::Mat sub_descriptror;
        CHECK(sift_->CreateContextGL() == SiftGPU::SIFTGPU_FULL_SUPPORTED);
        if (sift_->RunSIFT(sub_image.cols, sub_image.rows, sub_image.ptr(), GL_LUMINANCE, GL_UNSIGNED_BYTE)) {
            vector<SiftGPU::SiftKeypoint> keys;
            int num1 = sift_->GetFeatureNum();
            keys.resize(num1);
            vsub_descriptor.resize(128 * num1);
            sift_->GetFeatureVector(&keys[0], &vsub_descriptor[0]);
            sub_keypoints.reserve(num1);
            for_each(begin(keys), end(keys), [&](const SiftGPU::SiftKeypoint &key) {
                //根据siftpp的文档，实际size是4*m*scale，gpusift的默认m是3
                //对SIFT可以把scale 当成response，因为尺度越大的越稳定
                sub_keypoints.push_back(cv::KeyPoint(key.x, key.y, key.s * 3 * 4, key.o * RAD2DEG, key.s));
            });
            sub_descriptror = cv::Mat(num1, 128, CV_32F, &vsub_descriptor[0]);
            for (auto &key : sub_keypoints) {
                key.pt.x += col_range.start;
                key.pt.y += row_range.start;
            }

            if (sub_keypoints.size() > 0) {
                retain_best_features_gridded(sub_keypoints, sub_descriptror, nfeat_per_grid);

                kpts.insert(kpts.end(), sub_keypoints.begin(), sub_keypoints.end());
                desc.push_back(sub_descriptror.rowRange(0, sub_keypoints.size()));
            }
        }
    }

    // RootSift
    root_normalize(desc);

    retain_best_features_gridded(kpts, desc, nfeat_);
}

cv::Mat SiftDetector::compute(const cv::Mat &img, const std::vector<cv::KeyPoint> &keys, bool ignore_scale,
                              bool ignore_orient) {
    cv::Mat desc = cv::Mat::zeros(keys.size(), 128, CV_32F);
    int grid_row, grid_col;
    int rows(img.rows), cols(img.cols);

    float cell_size = 2048.0f;
    grid_row = std::ceil((float)rows / cell_size);
    grid_col = std::ceil((float)cols / cell_size);

    const float RAD2DEG = 180.0f / 3.141592653589f;
    for (int i = 0; i < grid_row * grid_col; ++i) {
        int celly = i / grid_col;
        int cellx = i - celly * grid_col;

        cv::Range row_range((celly * img.rows) / grid_row, ((celly + 1) * img.rows) / grid_row);
        cv::Range col_range((cellx * img.cols) / grid_col, ((cellx + 1) * img.cols) / grid_col);

        cv::Mat sub_image = img(row_range, col_range).clone();
        CHECK(sub_image.channels() == 1);

        // 记录落入当前 grid 的点的序号
        vector<fid_t> key_indices;
        // 记录落入当前 grid 的点的信息，需要减去偏移量，并且转换 scale 和 方向信息，然后上传给 sift 用于计算描述符
        vector<SiftGPU::SiftKeypoint> siftkeys;

        siftkeys.reserve(keys.size() / grid_row / grid_col);
        key_indices.reserve(keys.size() / grid_row / grid_col);

        // 便利所有的点，获取当前 grid 内的，并转换为 siftgpu 所需要的格式
        for (int k = 0; k < keys.size(); ++k) {
            const cv::KeyPoint &cvkey = keys[k];
            if (cvkey.pt.x >= col_range.start && cvkey.pt.x < col_range.end && cvkey.pt.y >= row_range.start &&
                cvkey.pt.y < row_range.end) {
                key_indices.push_back(k);

                SiftGPU::SiftKeypoint siftkey;
                siftkey.x = cvkey.pt.x - col_range.start;
                siftkey.y = cvkey.pt.y - row_range.start;

                if (ignore_scale) {
                    // 用大一点的区域要稳定一些
                    siftkey.s = 2.0;
                } else {
                    //根据siftpp的文档，实际size是4*m*scale，gpusift的默认m是3
                    siftkey.s = cvkey.size / 3.0 / 4.0;
                    if (siftkey.s < 1.0)
                        siftkey.s = 1.0f;
                }
                if (ignore_orient) {
                    siftkey.o = 0.0;
                } else {
                    siftkey.o = cvkey.angle / RAD2DEG;
                }
                siftkeys.push_back(siftkey);
            }
        }
        if (ignore_orient) {
            sift_->SetKeypointList(siftkeys.size(), siftkeys.data(), 0);
        } else {
            sift_->SetKeypointList(siftkeys.size(), siftkeys.data(), 1);
        }
        int num_subkeys = siftkeys.size();
        if (num_subkeys == 0) {
            continue;
        }

        vector<float> vsub_descriptor;
        cv::Mat sub_descriptror;
        if (sift_->RunSIFT(sub_image.cols, sub_image.rows, sub_image.ptr(), GL_LUMINANCE, GL_UNSIGNED_BYTE)) {
            int num1 = sift_->GetFeatureNum();
            CHECK_EQ(num1, num_subkeys);
            vsub_descriptor.resize(128 * num1);
            sift_->GetFeatureVector(NULL, &vsub_descriptor[0]);

            sub_descriptror = cv::Mat(num1, 128, CV_32F, &vsub_descriptor[0]);

            // 根据索引信息把当前获取的 descriptor 拷贝到整体影像的 descriptor 中
            for (int k = 0; k < key_indices.size(); ++k) {
                sub_descriptror.row(k).copyTo(desc.row(key_indices[k]));
            }
        }
    }

    // RootSift
    root_normalize(desc);
    return desc;
}

cv::Mat SiftDetector::compute(const cv::Mat &img, const FeaturePoints &features) {
    cv::Mat desc = cv::Mat::zeros(features.size(), 128, CV_32F);
    int grid_row, grid_col;
    int rows(img.rows), cols(img.cols);

    float cell_size = 2048.0f;
    grid_row = std::ceil((float)rows / cell_size);
    grid_col = std::ceil((float)cols / cell_size);

    const float RAD2DEG = 180.0f / 3.141592653589f;
    for (int i = 0; i < grid_row * grid_col; ++i) {
        int celly = i / grid_col;
        int cellx = i - celly * grid_col;

        cv::Range row_range((celly * img.rows) / grid_row, ((celly + 1) * img.rows) / grid_row);
        cv::Range col_range((cellx * img.cols) / grid_col, ((cellx + 1) * img.cols) / grid_col);

        cv::Mat sub_image = img(row_range, col_range).clone();
        CHECK(sub_image.channels() == 1);

        // 记录落入当前 grid 的点的序号
        vector<fid_t> key_indices;
        // 记录落入当前 grid 的点的信息，需要减去偏移量，并且转换 scale 和 方向信息，然后上传给 sift 用于计算描述符
        vector<SiftGPU::SiftKeypoint> siftkeys;

        siftkeys.reserve(features.size() / grid_row / grid_col);
        key_indices.reserve(features.size() / grid_row / grid_col);

        // 便利所有的点，获取当前 grid 内的，并转换为 siftgpu 所需要的格式
        for (int k = 0; k < features.size(); ++k) {
            const Vector2f &feature = features[k];
            if (feature(0) >= col_range.start && feature(0) < col_range.end && feature(1) >= row_range.start &&
                feature(1) < row_range.end) {
                key_indices.push_back(k);

                SiftGPU::SiftKeypoint siftkey;
                siftkey.x = feature(0) - col_range.start;
                siftkey.y = feature(1) - row_range.start;
                siftkey.s = 2.0;
                siftkey.o = 0.0;

                siftkeys.push_back(siftkey);
            }
        }

        int num_subkeys = siftkeys.size();
        if (num_subkeys == 0) {
            continue;
        }
        sift_->SetKeypointList(siftkeys.size(), siftkeys.data(), 0);

        vector<float> vsub_descriptor;
        cv::Mat sub_descriptror;
        if (sift_->RunSIFT(sub_image.cols, sub_image.rows, sub_image.ptr(), GL_LUMINANCE, GL_UNSIGNED_BYTE)) {
            int num1 = sift_->GetFeatureNum();
            CHECK_EQ(num1, num_subkeys);
            vsub_descriptor.resize(128 * num1);
            sift_->GetFeatureVector(NULL, &vsub_descriptor[0]);

            sub_descriptror = cv::Mat(num1, 128, CV_32F, &vsub_descriptor[0]);

            // 根据索引信息把当前获取的 descriptor 拷贝到整体影像的 descriptor 中
            for (int k = 0; k < key_indices.size(); ++k) {
                sub_descriptror.row(k).copyTo(desc.row(key_indices[k]));
            }
        }
    }

    // RootSift
    root_normalize(desc);
    return desc;
}

void SiftDetector::root_normalize(cv::Mat &desc) {
    Eigen::MatrixXf desc32f;
    cv::cv2eigen(desc, desc32f);

    Eigen::VectorXf sum = desc32f.rowwise().sum();
    Eigen::VectorXf sqsum = desc32f.rowwise().squaredNorm();
    Eigen::MatrixXf rootdesc = (desc32f.array().colwise() / sum.array()).sqrt();

    cv::eigen2cv(rootdesc, desc);

    // opencv 会顾及 overflow 的问题
    desc.convertTo(desc, CV_8U, 512.f, 0.0);
}

void SiftDetector::normalize(cv::Mat &desc) {
    // opencv 会顾及 overflow 的问题
    desc.convertTo(desc, CV_8U, 512.0f);
}

std::vector<cv::KeyPoint> detect_fast(const cv::Mat &mat, int nfeature) {
    CHECK_EQ(mat.type(), CV_8U);

    std::vector<cv::KeyPoint> keys;
    int min_response = 10;

    cv::FAST(mat, keys, min_response, true);

    // 保留指定数目的特征点
    cv::Mat empty_desc;
    retain_best_features_gridded(keys, empty_desc, nfeature);

    // 由于 fast 内部不做子像素定位，所以这里进行子像素定位
    std::vector<cv::Point2f> points(keys.size());
    for (int i = 0; i < points.size(); ++i) {
        points[i] = keys[i].pt;
    }
    cv::TermCriteria criteria;
    criteria.type = cv::TermCriteria::COUNT + cv::TermCriteria::EPS;
    criteria.maxCount = 2;
    criteria.epsilon = 0.1;
    // half size
    int corner_region = 5;
    cv::cornerSubPix(mat, points, cv::Size(corner_region, corner_region), cv::Size(-1, -1), criteria);
    for (int i = 0; i < points.size(); ++i) {
        keys[i].pt = points[i];
    }

    return keys;
}
} // namespace h2o

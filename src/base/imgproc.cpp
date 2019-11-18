/*
 * @Author: Han
 * @Date: 2017-12-16 15:19:16
 * 一些简单的影像处理函数
 */
#include <base/imgproc.h>

#include <ippi.h>
#include <opencv2/core/eigen.hpp>

namespace h2o {

template <typename T>
cv::Mat convert_to_8bit_t(const cv::Mat &mat, const cv::Mat &mask, Vector4f min_v, Vector4f max_v) {

    constexpr int ROWS = T::RowsAtCompileTime;

    using Vector8 = Eigen::Matrix<uint8_t, ROWS, 1>;
    using Vectorf = Eigen::Matrix<float, ROWS, 1>;
    using Vec8 = cv::Vec<uint8_t, ROWS>;

    bool has_mask = !mask.empty();

    cv::Mat mat8u;

    int channels = mat.channels();
    if (channels == 1) {
        mat8u = cv::Mat::zeros(mat.rows, mat.cols, CV_8U);
    } else if (channels == 3) {
        mat8u = cv::Mat::zeros(mat.rows, mat.cols, CV_8UC3);
    } else if (channels == 4) {
        mat8u = cv::Mat::zeros(mat.rows, mat.cols, CV_8UC4);
    }

    int rows = mat.rows;
    int cols = mat.cols;
    Vector4f scale = Vector4f::Constant(254.0).array() / (max_v - min_v).array();

#pragma omp parallel for
    for (int r = 0; r < rows; ++r) {
        Vector8 *mat8u_data = mat8u.ptr<Vector8>(r);
        T const *mat_data = mat.ptr<T>(r);
        for (int c = 0; c < cols; ++c) {
            Vectorf mat_value = mat_data[c].template cast<float>();
            mat_value =
                (mat_value - min_v.head(ROWS)).array() * scale.head(ROWS).array() + Vectorf::Constant(1).array();
            for (int v = 0; v < ROWS; ++v) {
                if (mat_value(v) < 1.0f) {
                    mat_value(v) = 1.0f;
                }
                if (mat_value(v) > 255.0f) {
                    mat_value(v) = 255.0f;
                }
            }
            mat8u_data[c] = mat_value.template cast<uint8_t>();
        }
    }

    if (has_mask) {
        mat8u.setTo(Vec8::zeros(), mask == 0);
    }

    return mat8u;
}

cv::Mat convert_to_8bit(const cv::Mat &mat, const cv::Mat &mask, double clip) {
    if (mat.depth() == CV_8U) {
        return mat;
    }

    cv::Mat mat8u;

    Vector4f min_v = Vector4f::Zero();
    Vector4f max_v = Vector4f::Zero();

    if (clip == 0.0) {
        std::tie(min_v, max_v) = image_minmax_channel(mat, mask);
    } else {
        std::tie(min_v, max_v) = image_minmax_channel_clip(mat, clip, mask);
    }

    if (mat.type() == CV_16UC3) {
        return convert_to_8bit_t<Vector3w>(mat, mask, min_v, max_v);
    } else if (mat.type() == CV_16UC4) {
        return convert_to_8bit_t<Vector4w>(mat, mask, min_v, max_v);
    } else if (mat.type() == CV_32FC3) {
        return convert_to_8bit_t<Vector3f>(mat, mask, min_v, max_v);
    } else if (mat.type() == CV_32FC4) {
        return convert_to_8bit_t<Vector4f>(mat, mask, min_v, max_v);
    }

    return mat8u;
}

template <typename T> Vector2d image_minmax_t(const cv::Mat &mat, const cv::Mat &mask) {
    bool has_mask = !mask.empty();
    Vector2d minmax(DBL_MAX, -DBL_MAX);

    for (int r = 0; r < mat.rows; ++r) {
        const T *data = mat.ptr<T>(r);
        const uint8_t *mask_data = has_mask ? mask.ptr<uint8_t>(r) : NULL;
        for (int c = 0; c < mat.cols; ++c) {
            if (has_mask && !mask_data[c]) {
                continue;
            }

            const T &value = data[c];
            double min_v = value.minCoeff();
            double max_v = value.maxCoeff();
            if (max_v > minmax(1)) {
                minmax(1) = max_v;
            }
            if (min_v < minmax(0)) {
                minmax(0) = min_v;
            }
        }
    }
    return minmax;
}

Vector2d image_minmax(const cv::Mat &mat, const cv::Mat &mask) {
    Vector2d minmax;
    if (mat.channels() == 1) {
        cv::minMaxIdx(mat, &minmax(0), &minmax(1), NULL, NULL, mask);
        return minmax;
    }

    if (mat.type() == CV_8UC3) {
        return image_minmax_t<Vector3b>(mat, mask);
    } else if (mat.type() == CV_8UC4) {
        return image_minmax_t<Vector4b>(mat, mask);
    } else if (mat.type() == CV_16UC3) {
        return image_minmax_t<Vector3w>(mat, mask);
    } else if (mat.type() == CV_16UC4) {
        return image_minmax_t<Vector4w>(mat, mask);
    } else if (mat.type() == CV_32FC3) {
        return image_minmax_t<Vector3f>(mat, mask);
    } else if (mat.type() == CV_32FC4) {
        return image_minmax_t<Vector4f>(mat, mask);
    }

    return Vector2d(DBL_MAX, -DBL_MAX);
}

template <typename T> std::tuple<Vector4f, Vector4f> image_minmax_channel_t(const cv::Mat &mat, const cv::Mat &mask) {
    constexpr int ROWS = T::RowsAtCompileTime;

    bool has_mask = !mask.empty();
    Vector4f min_v = Vector4f::Constant(FLT_MAX);
    Vector4f max_v = Vector4f::Constant(-FLT_MAX);

    for (int r = 0; r < mat.rows; ++r) {
        const T *data = mat.ptr<T>(r);
        const uint8_t *mask_data = has_mask ? mask.ptr<uint8_t>(r) : NULL;
        for (int c = 0; c < mat.cols; ++c) {
            if (has_mask && !mask_data[c]) {
                continue;
            }

            const T &value = data[c];

            for (int v = 0; v < ROWS; ++v) {
                if (value(v) > max_v(v)) {
                    max_v(v) = value(v);
                }
                if (value(v) < min_v(v)) {
                    min_v(v) = value(v);
                }
            }
        }
    }

    return std::make_tuple(min_v, max_v);
}

std::tuple<Vector4f, Vector4f> image_minmax_channel(const cv::Mat &mat, const cv::Mat &mask) {
    Vector4f min_v = Vector4f::Constant(FLT_MAX), max_v = Vector4f::Constant(-FLT_MAX);
    if (mat.channels() == 1) {
        double min_value, max_value;
        cv::minMaxIdx(mat, &min_value, &max_value, NULL, NULL, mask);
        min_v(0) = min_value;
        max_v(0) = max_value;
        return std::make_tuple(min_v, max_v);
    }

    if (mat.type() == CV_8UC3) {
        return image_minmax_channel_t<Vector3b>(mat, mask);
    } else if (mat.type() == CV_8UC4) {
        return image_minmax_channel_t<Vector4b>(mat, mask);
    } else if (mat.type() == CV_16UC3) {
        return image_minmax_channel_t<Vector3w>(mat, mask);
    } else if (mat.type() == CV_16UC4) {
        return image_minmax_channel_t<Vector4w>(mat, mask);
    } else if (mat.type() == CV_32FC3) {
        return image_minmax_channel_t<Vector3f>(mat, mask);
    } else if (mat.type() == CV_32FC4) {
        return image_minmax_channel_t<Vector4f>(mat, mask);
    }

    return std::make_tuple(min_v, max_v);
}

std::tuple<float, float> image_minmax_clip(const cv::Mat &mat, double clip, const cv::Mat &mask) {
    // 先计算直方图 统计 cdf
    // 计算 histogram，并且计算 cdf，然后判断 clip 的最大和最小值
    double min_max[2];
    cv::minMaxIdx(mat, &min_max[0], &min_max[1], NULL, NULL, mask);
    /// Establish the number of bins
    int hist_size = 256;
    float step = (min_max[1] - min_max[0]) / hist_size;

    /// Set the ranges ( for R,G,B) )
    float range[] = {(float)min_max[0], (float)min_max[1] + step * 0.5f};
    const float *hist_range = {range};

    cv::Mat hists;
    calcHist(&mat, 1, 0, mask, hists, 1, &hist_size, &hist_range);
    std::vector<float> hist = hists;
    CHECK(hist.size() == hist_size);

    double total;
    if (mask.empty()) {
        total = mat.rows * mat.cols;
    } else {
        total = cv::countNonZero(mask);
    }

    std::vector<double> cdf(hist.size());
    int i_min_max[2] = {0, hist_size - 1};
    for (int i = 0; i < hist.size(); ++i) {
        cdf[i] = hist[i] / total;
        if (i != 0) {
            cdf[i] += cdf[i - 1];
        }
        if (cdf[i] * 100.0 > clip && i_min_max[0] == 0) {
            i_min_max[0] = i;
        }
        if (cdf[i] * 100.0 > 100 - clip && i_min_max[1] == hist_size - 1) {
            i_min_max[1] = i;
        }
    }
    min_max[0] = range[0] + step * i_min_max[0];
    min_max[1] = range[0] + step * i_min_max[1];

    return std::make_tuple(min_max[0], min_max[1]);
}

std::tuple<Vector4f, Vector4f> image_minmax_channel_clip(const cv::Mat &mat, double clip, const cv::Mat &mask) {
    Vector4f min_v = Vector4f::Constant(FLT_MAX);
    Vector4f max_v = Vector4f::Constant(-FLT_MAX);

    int channels = mat.channels();
    for (int i = 0; i < channels; ++i) {
        float minmax[2];
        cv::Mat channel = image_extract_channel(mat, i);
        std::tie(minmax[0], minmax[1]) = image_minmax_clip(channel, clip, mask);
        min_v(i) = minmax[0];
        max_v(i) = minmax[1];
    }

    return std::make_tuple(min_v, max_v);
}

// src 必须是单波段，dst 是多波段，且两者类型一致
template <typename T> void image_copy_channel_t(const cv::Mat &src, cv::Mat &dst, int channel, const cv::Mat &mask) {
    using Scalar = typename T::Scalar;

    bool has_mask = !mask.empty();

#pragma omp parallel for
    for (int r = 0; r < src.rows; ++r) {
        const Scalar *src_data = src.ptr<Scalar>(r);
        T *dst_data = dst.ptr<T>(r);
        const uint8_t *mask_data = has_mask ? mask.ptr<uint8_t>(r) : NULL;
        for (int c = 0; c < src.cols; ++c) {
            if (has_mask && !mask_data[c]) {
                continue;
            }
            dst_data[c](channel) = src_data[c];
        }
    }
}

void image_copy_channel(const cv::Mat &src, cv::Mat &dst, int channel, const cv::Mat &mask) {
    CHECK(src.depth() == dst.depth());
    CHECK(src.rows == dst.rows && src.cols == dst.cols);

    if (dst.channels() == 1) {
        src.copyTo(dst, mask);
        return;
    }

    if (dst.type() == CV_8UC3) {
        image_copy_channel_t<Vector3b>(src, dst, channel, mask);
    } else if (dst.type() == CV_8UC4) {
        image_copy_channel_t<Vector4b>(src, dst, channel, mask);
    } else if (dst.type() == CV_16UC3) {
        image_copy_channel_t<Vector3w>(src, dst, channel, mask);
    } else if (dst.type() == CV_16UC4) {
        image_copy_channel_t<Vector4w>(src, dst, channel, mask);
    }
}

template <typename T> cv::Mat image_extract_channel_t(const cv::Mat &src, int v) {
    using Scalar = typename T::Scalar;
    cv::Mat channel = cv::Mat::zeros(src.rows, src.cols, src.depth());

    for (int r = 0; r < src.rows; ++r) {
        const T *src_data = src.ptr<T>(r);
        Scalar *channel_data = channel.ptr<Scalar>(r);
        for (int c = 0; c < src.cols; ++c) {
            channel_data[c] = src_data[c](v);
        }
    }
    return channel;
}
cv::Mat image_extract_channel(const cv::Mat &src, int channel) {
    if (src.channels() == 1) {
        return src;
    }

    if (src.type() == CV_8UC3) {
        return image_extract_channel_t<Vector3b>(src, channel);
    } else if (src.type() == CV_8UC4) {
        return image_extract_channel_t<Vector4b>(src, channel);
    } else if (src.type() == CV_16UC3) {
        return image_extract_channel_t<Vector3w>(src, channel);
    } else if (src.type() == CV_16UC4) {
        return image_extract_channel_t<Vector4w>(src, channel);
    } else if (src.type() == CV_32FC3) {
        return image_extract_channel_t<Vector3f>(src, channel);
    } else if (src.type() == CV_32FC4) {
        return image_extract_channel_t<Vector4f>(src, channel);
    }

    return cv::Mat();
}
cv::Mat smooth_resize(const cv::Mat &mat, const Vector2i &dsize) {
    int image_size = std::max(mat.rows, mat.cols);
    int dst_size = std::max(dsize(0), dsize(1));

    double scale = (double)image_size / dst_size;
    cv::Mat dst = mat;
    if (scale > 1.0) {
        int ksize = scale + 0.5;
        cv::blur(dst, dst, {ksize, ksize});
    }
    cv::resize(dst, dst, {dsize(0), dsize(1)}, 0.0, 0.0, cv::INTER_NEAREST);

    return dst;
}

cv::Mat image_percent_scale_8u(const cv::Mat &image, cv::Mat mask, double percent) {
    CHECK(image.channels() == 1);
    if (mask.empty()) {
        mask = cv::Mat(image.rows, image.cols, CV_8U, cv::Scalar(255));
    }

    cv::Mat mat;

    double min_max[2];
    cv::minMaxIdx(image, &min_max[0], &min_max[1], NULL, NULL, mask);
    if (percent != 0) {
        // 计算 histogram，并且计算 cdf，然后判断 clip 的最大和最小值
        std::vector<cv::Mat> mats = {image};
        int dims = 2;
        int nimage = 1;
        std::vector<int> hist_size = {256};
        std::vector<int> channels = {0};
        float step = (min_max[1] - min_max[0]) / hist_size[0];
        std::vector<float> ranges = {(float)min_max[0], (float)min_max[1] + step * 0.5f};

        cv::Mat hists;
        cv::calcHist(mats, channels, mask, hists, hist_size, ranges);
        std::vector<float> hist = hists;
        CHECK(hist.size() == hist_size[0]);

        double total = cv::countNonZero(mask);
        std::vector<double> cdf(hist.size());
        int i_min_max[2] = {0, hist_size[0] - 1};
        for (int i = 0; i < hist.size(); ++i) {
            cdf[i] = hist[i] / total;
            if (i != 0) {
                cdf[i] += cdf[i - 1];
            }
            if (cdf[i] * 100.0 > percent && i_min_max[0] == 0) {
                i_min_max[0] = i;
            }
            if (cdf[i] * 100.0 > 100 - percent && i_min_max[1] == hist_size[0] - 1) {
                i_min_max[1] = i;
            }
        }
        min_max[0] = ranges[0] + step * i_min_max[0];
        min_max[1] = ranges[0] + step * i_min_max[1];
    }

    image.convertTo(mat, CV_32F);
    mat = (mat - min_max[0]) / (min_max[1] - min_max[0]) * 254.0f + 1.0f;
    mat.setTo(cv::Scalar(1.0f), mat < 1.0f);
    mat.setTo(cv::Scalar(255.0f), mat > 255.0f);
    mat.convertTo(mat, CV_8U);
    mat.setTo(cv::Scalar(0), mask == 0);

    return mat;
}

cv::Mat remap_ipp(const cv::Mat &src, const cv::Mat &mapx, const cv::Mat &mapy, int _interpolation) {
    CHECK(src.type() == CV_8U || src.type() == CV_8UC3 || src.type() == CV_8UC4 || src.type() == CV_16U ||
          src.type() == CV_16UC3 || src.type() == CV_16UC4 || src.type() == CV_32F || src.type() == CV_32FC3 ||
          src.type() == CV_32FC4);
    CHECK(mapx.type() == CV_32F && mapy.type() == CV_32F);
    CHECK(mapx.rows == mapy.rows && mapx.cols == mapy.cols);

    cv::Mat dst = cv::Mat::zeros(mapx.rows, mapx.cols, src.type());

    // ipp 所需要的所有参数
    int type = src.type();
    IppiSize src_size = {src.cols, src.rows};
    int src_step = src.step[0];
    IppiRect src_roi = {0, 0, src.cols, src.rows};
    const Ipp32f *mapx_data = (Ipp32f *)mapx.data;
    int mapx_step = mapx.step[0];
    const Ipp32f *mapy_data = (Ipp32f *)mapy.data;
    int mapy_step = mapy.step[0];
    int dst_step = dst.step[0];
    IppiSize dst_size = {dst.cols, dst.rows};
    int interpolation = IPPI_INTER_LINEAR;
    if (_interpolation == cv::INTER_LINEAR) {
        interpolation = IPPI_INTER_LINEAR;
    } else if (_interpolation == cv::INTER_NEAREST) {
        interpolation = IPPI_INTER_NN;
    } else if (_interpolation == cv::INTER_CUBIC) {
        interpolation = IPPI_INTER_CUBIC;
    }

    switch (type) {
    case CV_8U:
        ippiRemap_8u_C1R((const Ipp8u *)src.data, src_size, src_step, src_roi, mapx_data, mapx_step, mapy_data,
                         mapy_step, (Ipp8u *)dst.data, dst_step, dst_size, interpolation);
        break;
    case CV_8UC3:
        ippiRemap_8u_C3R((const Ipp8u *)src.data, src_size, src_step, src_roi, mapx_data, mapx_step, mapy_data,
                         mapy_step, (Ipp8u *)dst.data, dst_step, dst_size, interpolation);
        break;
    case CV_8UC4:
        ippiRemap_8u_C4R((const Ipp8u *)src.data, src_size, src_step, src_roi, mapx_data, mapx_step, mapy_data,
                         mapy_step, (Ipp8u *)dst.data, dst_step, dst_size, interpolation);
        break;
    case CV_16U:
        ippiRemap_16u_C1R((const Ipp16u *)src.data, src_size, src_step, src_roi, mapx_data, mapx_step, mapy_data,
                          mapy_step, (Ipp16u *)dst.data, dst_step, dst_size, interpolation);
        break;
    case CV_16UC3:
        ippiRemap_16u_C3R((const Ipp16u *)src.data, src_size, src_step, src_roi, mapx_data, mapx_step, mapy_data,
                          mapy_step, (Ipp16u *)dst.data, dst_step, dst_size, interpolation);
        break;
    case CV_16UC4:
        ippiRemap_16u_C4R((const Ipp16u *)src.data, src_size, src_step, src_roi, mapx_data, mapx_step, mapy_data,
                          mapy_step, (Ipp16u *)dst.data, dst_step, dst_size, interpolation);
        break;
    case CV_32F:
        ippiRemap_32f_C1R((const Ipp32f *)src.data, src_size, src_step, src_roi, mapx_data, mapx_step, mapy_data,
                          mapy_step, (Ipp32f *)dst.data, dst_step, dst_size, interpolation);
        break;
    case CV_32FC3:
        ippiRemap_32f_C3R((const Ipp32f *)src.data, src_size, src_step, src_roi, mapx_data, mapx_step, mapy_data,
                          mapy_step, (Ipp32f *)dst.data, dst_step, dst_size, interpolation);
        break;
    case CV_32FC4:
        ippiRemap_32f_C4R((const Ipp32f *)src.data, src_size, src_step, src_roi, mapx_data, mapx_step, mapy_data,
                          mapy_step, (Ipp32f *)dst.data, dst_step, dst_size, interpolation);
        break;

    default:
        break;
    }

    return dst;
}

Matrix23d compute_affine(const std::vector<Vector2d> &source, const std::vector<Vector2d> &dest) {
    std::vector<cv::Point2f> cv_source(source.size()), cv_dest(dest.size());
    for (int i = 0; i < source.size(); ++i) {
        cv_source[i] = cv::Point2f(source[i].x(), source[i].y());
        cv_dest[i] = cv::Point2f(dest[i].x(), dest[i].y());
    }

    // affine 只能接收 32f
    cv::Mat M = cv::getAffineTransform(cv_source, cv_dest);
    Matrix23d A;
    cv::cv2eigen(M, A);
    return A;
}

Matrix23d revers_affine(const Matrix23d &M) {
    Matrix2d A = M.leftCols(2).inverse();
    Vector2d b = -A * M.col(2);

    Matrix23d M2;
    M2.leftCols(2) = A;
    M2.col(2) = b;

    return M2;
}

Matrix3d compute_homography(const std::vector<Vector2d> &source, const std::vector<Vector2d> &dest) {
    std::vector<cv::Point2f> cv_source(source.size()), cv_dest(dest.size());
    for (int i = 0; i < source.size(); ++i) {
        cv_source[i] = cv::Point2f(source[i].x(), source[i].y());
        cv_dest[i] = cv::Point2f(dest[i].x(), dest[i].y());
    }

    // 只能接收 32f
    cv::Mat M = cv::getPerspectiveTransform(cv_source, cv_dest);
    Matrix3d A;
    cv::cv2eigen(M, A);
    return A;
}

} // namespace h2o

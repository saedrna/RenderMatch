/*
 * @Author: Han
 * @Date: 2017-12-11 19:52:59
 * DSM 和 DEM 之差
 */
#include <base/ndsm.h>

namespace h2o {
NDsm::NDsm(const GeoImagePtr &dsm, const GeoImagePtr &dem, const cv::Mat &mask, const cv::Vec4b &background)
    : nodata_mask_(mask) {
    background_ = background;
    dsm_ = dsm;
    dem_ = dem;

    colors_.resize(6);
    colors_[0] = Vector3f{153.0f, 102.0f, 255.0f} / 255.0f;
    colors_[1] = Vector3f{0.0f, 0.0f, 255.0f} / 255.0f;
    colors_[2] = Vector3f{0.0f, 255.0f, 0.0f} / 255.0f;
    colors_[3] = Vector3f{255.0f, 255.0f, 0.0f} / 255.0f;
    colors_[4] = Vector3f{255.0f, 102.0f, 0.0f} / 255.0f;
    colors_[5] = Vector3f{255.0f, 0.0f, 0.0f} / 255.0f;

    minmax_ = {FLT_MAX, -FLT_MAX};

    int rows = dsm->get_image().rows;
    int cols = dsm->get_image().cols;

    ndsm_ = std::make_shared<GeoImage>();
    ndsm_color_ = std::make_shared<GeoImage>();
    ndsm_->init_image(rows, cols, CV_32F, dsm->get_xform());
    ndsm_color_->init_image(rows, cols, CV_8UC4, dsm->get_xform());

    step_ = 10.0f;
}
NDsm::~NDsm() {}

void NDsm::compute(BoundingBox2i rect /*= BoundingBox2i()*/) {

    int rows = dsm_->get_image().rows;
    int cols = dsm_->get_image().cols;
    // 最小值是包含，最大值是不包含
    CHECK(rect.min()(0) >= 0 && rect.min()(1) >= 0 && rect.max()(0) <= cols && rect.max()(1) <= rows);

    if (rect.isEmpty()) {
        rect.extend(Vector2i(0, 0));
        rect.extend(Vector2i(cols, rows));
    }

    cv::Rect region(rect.min()(0), rect.min()(1), rect.sizes()(0), rect.sizes()(1));

    cv::Mat dsm = dsm_->get_image()(region);
    cv::Mat dem = dem_->get_image()(region);

    // 确保 DEM 和 DSM 在外部都已经填充了无效值
    cv::Mat ndsm = dsm - dem;
    cv::Mat mask = nodata_mask_(region);

    // 更新最大最小高差
    double minmax[2];
    cv::minMaxIdx(ndsm, &minmax[0], &minmax[1], NULL, NULL, mask);
    Vector2f new_minmax = compute_min_max(minmax[0], minmax[1]);
    // 如果新的 最大最小值超出了原始最大最小值的范围，则更新所有区域
    if (new_minmax[0] < minmax_[0] || new_minmax[1] > minmax_[1]) {
        minmax_[0] = std::min(minmax_[0], new_minmax[0]);
        minmax_[1] = std::max(minmax_[1], new_minmax[1]);

        cv::Mat ndsm_all = dsm_->get_image() - dem_->get_image();
        ndsm_all.copyTo(ndsm_->get_image());
        cv::Mat color = colorize_ndsm(ndsm_all, nodata_mask_);
        color.copyTo(ndsm_color_->get_image());
    } else {
        // 最大最小值没变，只更新局部区域的
        cv::Mat color = colorize_ndsm(ndsm, mask);
        ndsm.copyTo(ndsm_->get_image()(region));
        color.copyTo(ndsm_color_->get_image()(region));
    }
}
std::tuple<Vector2f, std::vector<Vector3f>> NDsm::get_lengend() const {
    std::vector<Vector3f> colors = colors_;
    for (Vector3f &color : colors) {
        color *= 255.0f; // 变到0-255
    }
    return std::make_tuple(minmax_, colors);
}
Vector2f NDsm::compute_min_max(float min_v, float max_v) {
    min_v = std::min(min_v, minmax_(0));
    max_v = std::max(max_v, minmax_(1));

    Vector2f minmax_v(min_v, max_v);

    float length = minmax_v.norm();
    float nstep, part;
    part = std::modf(length / step_, &nstep);
    if (part != 0.0f) {
        nstep += 1.0f;
    } else if (part == 0.0f && nstep == 0.0f) {
        nstep += 1.0f;
    }

    // 将最大最小值扩充一点，防止是0大小
    float new_length = nstep * step_;
    float extend = (new_length - length) / 2.0f;
    minmax_v(0) -= extend;
    minmax_v(1) += extend;

    // ndsm 最大最小值要对称
    float max_absolute = std::max(std::abs(minmax_v(0)), std::abs(minmax_v(1)));
    minmax_v(0) = -max_absolute;
    minmax_v(1) = max_absolute;

    CHECK(!std::isnan(minmax_v(0)) && !std::isnan(minmax_v(1)));

    return minmax_v;
}

cv::Mat NDsm::colorize_ndsm(const cv::Mat &mat, const cv::Mat &mask) {
    float minz = minmax_(0);
    float maxz = minmax_(1);
    cv::Mat color_image(mat.rows, mat.cols, CV_8UC4);
    {
#pragma omp parallel for
        for (int r = 0; r < mat.rows; ++r) {
            cv::Vec4b *color_image_data = color_image.ptr<cv::Vec4b>(r);
            const float *mat_data = mat.ptr<float>(r);
            const uint8_t *mask_data = mask.ptr<uint8_t>(r);
            for (int c = 0; c < mat.cols; ++c) {
                if (mask_data[c]) {
                    float z = mat_data[c];
                    float iv;
                    iv = ((z - minz) / (maxz - minz)) * (colors_.size() - 1);

                    int lower = int(round(floor(iv)));
                    int upper = int(round(ceil(iv)));
                    if (lower < 0)
                        lower = 0;
                    if (upper > colors_.size() - 1)
                        upper = colors_.size() - 1;

                    Vector3f color32f = colors_[lower] + ((colors_[upper] - colors_[lower]) * (iv - lower));
                    Vector3b color8u = (color32f * 255.0).cast<uint8_t>();
                    // RGB
                    color_image_data[c] = {color8u[0], color8u[1], color8u[2], 255};
                } else {
                    color_image_data[c] = background_;
                }
            }
        }
    }
    return color_image;
}
} // namespace h2o

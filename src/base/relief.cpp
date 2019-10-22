/*!
 * \file relief.cpp
 *
 * \author Han
 * \date 2016/12/29
 *
 * DSM/DEM 生成 shaded relief 图
 */

#include <base/relief.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

constexpr int ValidDataDem = 255;
namespace h2o {

ReliefShader::ReliefShader(const GeoImagePtr &dem, const cv::Mat &mask, const cv::Vec4b &background, float z_scale,
                           float azimuth, float altitude)
    : nodata_mask_(mask.clone()), background_(background) {
    z_scale_ = z_scale;
    azimuth_ = azimuth;
    altitude_ = altitude;
    dem_ = dem;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {3, 3});
    cv::erode(nodata_mask_, nodata_mask_, kernel);

    colors_ = {Vector3f(200, 215, 133) / 255.0f, Vector3f(171, 217, 177) / 255.0f, Vector3f(124, 196, 120) / 255.0f,
               Vector3f(117, 193, 120) / 255.0f, Vector3f(175, 204, 166) / 255.0f, Vector3f(219, 208, 78) / 255.0f,
               Vector3f(241, 207, 14) / 255.0f,  Vector3f(242, 167, 0) / 255.0f,   Vector3f(192, 159, 13) / 255.0f,
               Vector3f(211, 192, 112) / 255.0f, Vector3f(240, 219, 161) / 255.0f, Vector3f(252, 236, 192) / 255.0f,
               Vector3f(248, 250, 234) / 255.0f, Vector3f(229, 254, 250) / 255.0f, Vector3f(219, 255, 253) / 255.0f,
               Vector3f(214, 251, 252) / 255.0f, Vector3f(183, 244, 247) / 255.0f, Vector3f(115, 224, 241) / 255.0f,
               Vector3f(29, 188, 239) / 255.0f,  Vector3f(0, 137, 245) / 255.0f,   Vector3f(0, 80, 250) / 255.0f};

    minmax_ = {FLT_MAX, -FLT_MAX};
    step_ = 10.0f;
    hillshade_ = std::make_shared<GeoImage>();
    hillshade_->init_image(dem->get_image().rows, dem->get_image().cols, CV_8UC4, dem->get_xform());
}

ReliefShader::~ReliefShader() {}

void ReliefShader::compute(BoundingBox2i rect /*= BoundingBox2i()*/) {

    bool updata_full = false;

    float *pscale = &scale_;
    int rows = dem_->get_image().rows;
    int cols = dem_->get_image().cols;
    // 最小值是包含，最大值是不包含
    CHECK(rect.min()(0) >= 0 && rect.min()(1) >= 0 && rect.max()(0) <= cols && rect.max()(1) <= rows);
    if (rect.isEmpty()) {
        rect.extend(Vector2i(0, 0));
        rect.extend(Vector2i(cols, rows));
        pscale = NULL; // 重新计算 scale
        updata_full = true;
    }

    if (rect.sizes()(0) < 2 || rect.sizes()(1) < 2) {
        // 因为涉及到梯度，所以四边不更新
        return;
    }

    cv::Rect region(rect.min()(0), rect.min()(1), rect.sizes()(0), rect.sizes()(1));

    // 确保 DEM在外部都已经填充了无效值
    cv::Mat dem = dem_->get_image()(region);
    cv::Mat mask = nodata_mask_(region);

    // 更新最大最小高差

    if (minmax_[0] == FLT_MAX || minmax_[1] == -FLT_MAX) {
        double minmax[2];
        cv::minMaxIdx(dem, &minmax[0], &minmax[1], NULL, NULL, mask);
        minmax_ = compute_minmax(minmax[0], minmax[1]);
    }
    CHECK(!std::isnan(minmax_(0)) && !std::isnan(minmax_(1)));

    // 最大最小值没变，只更新局部区域的
    // 计算 hillshade 并更新
    cv::Mat hillshade = compute_hillshade(dem, mask, pscale);

    // 四周缩小一像素，防止梯度计算的错误导致的问题
    if (!updata_full) {
        hillshade = hillshade(cv::Rect(1, 1, hillshade.cols - 2, hillshade.rows - 2));
        region = cv::Rect(rect.min()(0) + 1, rect.min()(1) + 1, rect.sizes()(0) - 2, rect.sizes()(1) - 2);
    }
    hillshade.copyTo(hillshade_->get_image()(region));
}

std::tuple<Eigen::Vector2f, std::vector<Eigen::Vector3f>> ReliefShader::get_legend() const {
    std::vector<Vector3f> colors = colors_;
    for (Vector3f &color : colors) {
        color *= 255.0f; // 变到0-255
    }
    return std::make_tuple(minmax_, colors);
}

Eigen::Vector2f ReliefShader::compute_minmax(float min_v, float max_v) {
    min_v = std::min(min_v, minmax_(0));
    max_v = std::max(max_v, minmax_(1));

    Vector2f minmax_v(min_v, max_v);

    float length = minmax_v.norm();
    if (length == 0.0) {
        float nstep, part;
        part = std::modf(length / step_, &nstep);
        if (part != 0.0f) {
            nstep += 1.0f;
        } else if (part == 0.0f && nstep == 0.0f) {
            nstep += 1.0f;
        }

        float new_length = nstep * step_;
        float extend = (new_length - length) / 2.0f;

        minmax_v(0) -= extend;
        minmax_v(1) += extend;
    }

    return minmax_v;
}

cv::Mat ReliefShader::compute_hillshade(const cv::Mat &mat, const cv::Mat &mask, float *pscale) {
    const float RAD2DEG = 180.0f / 3.14159f;
    const float DEG2RAD = 3.14159f / 180.0f;

    cv::Mat color_image(mat.rows, mat.cols, CV_8UC3);
    {
        float minz = minmax_(0);
        float maxz = minmax_(1);

#pragma omp parallel for
        for (int r = 0; r < mat.rows; ++r) {
            for (int c = 0; c < mat.cols; ++c) {
                const uint8_t *mask_data = mask.ptr<uint8_t>(r);
                cv::Vec3b *color_image_data = color_image.ptr<cv::Vec3b>(r);
                const float *mat_data = mat.ptr<float>(r);
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
                    color_image_data[c] = {color8u[0], color8u[1], color8u[2]};
                } else {
                    color_image_data[c] = {0, 0, 0};
                }
            }
        }
    }
    cv::Mat hillshade(mat.rows, mat.cols, CV_32F);
    {
        ///\ sa.
        /// http://edndoc.esri.com/arcobjects/9.2/net/shared/geoprocessing/spatial_analyst_tools/how_hillshade_works.htm
        float zenith = 90.0f - altitude_;
        float azimuth = 360.0 - azimuth_ + 90.0;
        if (azimuth >= 360.0f) {
            azimuth -= 360.0f;
        }
        float cellx = dem_->get_xform()(0, 0);
        float celly = std::abs(dem_->get_xform()(1, 1));

        const float COS_ZENITH = cos(zenith * DEG2RAD);
        const float SIN_ZENITH = sin(zenith * DEG2RAD);
        const float COS_AZIMUTH = cos(azimuth * DEG2RAD);
        const float SIN_AZIMUTH = sin(azimuth * DEG2RAD);

        auto hillshade_func = [&](float slope, float aspect) -> float {
            float value = (COS_ZENITH * cos(slope * DEG2RAD) +
                           SIN_ZENITH * sin(slope * DEG2RAD) * cos((azimuth - aspect) * DEG2RAD));
            return value;
        };
        auto aspect_func = [&](float sx, float sy) -> float {
            float aspect = 0.0f;
            if (sx != 0.0f) {
                aspect = atan2(sy, -sx) * RAD2DEG;
                if (aspect < 0.0f) {
                    aspect += 360.0f;
                }
                return aspect;
            } else {
                // sx == 0
                if (sy >= 0.0f) {
                    aspect = 90.0f;
                } else {
                    aspect = 270.0f;
                }
                return aspect;
            }
            return aspect;
        };
        cv::Mat slope_x, slope_y;
        cv::Sobel(mat, slope_x, CV_32F, 1, 0, 3, 1.0 / 8.0 / cellx, 0.0, cv::BORDER_DEFAULT);
        cv::Sobel(mat, slope_y, CV_32F, 0, 1, 3, 1.0 / 8.0 / celly, 0.0, cv::BORDER_DEFAULT);

#pragma omp parallel for
        for (int r = 0; r < mat.rows; ++r) {
            const float *slope_x_data = slope_x.ptr<float>(r);
            const float *slope_y_data = slope_y.ptr<float>(r);
            const uint8_t *mask_data = mask.ptr<uint8_t>(r);
            float *hillshade_data = hillshade.ptr<float>(r);

            for (int c = 0; c < mat.cols; ++c) {
                if (mask_data[c] == ValidDataDem) {
                    float sx = slope_x_data[c];
                    float sy = slope_y_data[c];
                    float slope = atan(z_scale_ * sqrt(sx * sx + sy * sy)) * RAD2DEG;
                    float aspect = aspect_func(sx, sy);

                    float hillshade_value = hillshade_func(slope, aspect);

                    if (hillshade_value < 0.0f) {
                        hillshade_value = 0.0f;
                    }
                    if (hillshade_value > 1.0f) {
                        hillshade_value = 1.0f;
                    }
                    hillshade_data[c] = hillshade_value;
                } else {
                    hillshade_data[c] = 0.0f;
                }
            }
        }
    }

    float scale;
    if (pscale == NULL) {
        float mean = cv::mean(hillshade, mask)[0];
        scale = 1.0f / mean;
        scale_ = scale;
    } else {
        scale = *pscale;
    }

    cv::Mat color_hillshade(mat.rows, mat.cols, CV_8UC4);
    cv::Mat color_image_32f;
    color_image.convertTo(color_image_32f, CV_32FC3, 1.0 / 255.0);
    cv::cvtColor(color_image_32f, color_image_32f, cv::COLOR_RGB2Lab);

#pragma omp parallel for
    for (int r = 0; r < mat.rows; ++r) {
        cv::Vec3f *color_image_32f_data = color_image_32f.ptr<cv::Vec3f>(r);
        const float *hillshade_data = hillshade.ptr<float>(r);
        for (int c = 0; c < mat.cols; ++c) {
            cv::Vec3f &lab = color_image_32f_data[c];
            float muliplier = hillshade_data[c] * scale;
            lab[0] *= muliplier;
            if (lab[0] > 100.0f)
                lab[0] = 100.0f;
        }
    }

    cv::cvtColor(color_image_32f, color_image_32f, cv::COLOR_Lab2RGB);

#pragma omp parallel for
    for (int r = 0; r < mat.rows; ++r) {
        const uint8_t *mask_data = mask.ptr<uint8_t>(r);
        cv::Vec4b *color_hillshade_data = color_hillshade.ptr<cv::Vec4b>(r);
        const cv::Vec3f *color_image_32f_data = color_image_32f.ptr<cv::Vec3f>(r);
        for (int c = 0; c < mat.cols; ++c) {
            if (mask_data[c]) {
                cv::Vec3f color = color_image_32f_data[c];
                color *= 255.0f;
                color_hillshade_data[c] = cv::Vec4b(color[0], color[1], color[2], 255);
            } else {
                color_hillshade_data[c] = background_;
            }
        }
    }

    return color_hillshade;
}

} // namespace h2o

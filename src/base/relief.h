/*!
 * \file relief.h
 *
 * \author Han
 * \date 2016/12/29
 *
 *
 */
#pragma once
#include <base/common.h>
#include <base/geo_image.h>

namespace h2o {

class ReliefShader {
public:
    ReliefShader(const GeoImagePtr &dem, const cv::Mat &mask, const cv::Vec4b &background, float z_scale = 1.0f,
                 float azimuth = -45.0f, float altitude = 45.0f);
    ~ReliefShader();
    // 计算或者更新，如果 bb 是空的则是根据 DSM DEM 重新计算，如果非空则更新
    void compute(BoundingBox2i rect = BoundingBox2i());
    GeoImagePtr get_hillshade() { return hillshade_; }

    std::tuple<Vector2f, std::vector<Vector3f>> get_legend() const;
    void set_minmax(const Vector2f &minmax) { minmax_ = minmax; }

protected:
    Vector2f compute_minmax(float min_v, float max_v);
    cv::Mat compute_hillshade(const cv::Mat &mat, const cv::Mat &mask = cv::Mat(), float *pscale = NULL);

protected:
    float z_scale_;
    float azimuth_;
    float altitude_;

    cv::Vec4b background_;
    cv::Mat nodata_mask_;

    std::vector<Vector3f> colors_;

    GeoImagePtr dem_;
    GeoImagePtr hillshade_;

    Vector2f minmax_;
    float step_;

    // 最终的颜色根据这个尺度调整，在LAB色彩空间, 这个是 hill shade 坡度的均值
    float scale_;
};
} // namespace h2o

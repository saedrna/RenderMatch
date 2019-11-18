/*
 * @Author: Han
 * @Date: 2017-12-16 15:19:00
 * 一些简单的影像处理函数
 */
#pragma once
#include <base/common.h>

namespace h2o {
// 把影像转换为 8bit 影像，会自动缩放到 1 - 255, 其中 0 是无效值， 会截断前后 clip 比例
cv::Mat convert_to_8bit(const cv::Mat &mat, const cv::Mat &mask = cv::Mat(), double clip = 0.0);

// 计算单波段影像的最大最小值
Vector2d image_minmax(const cv::Mat &mat, const cv::Mat &mask = cv::Mat());

// 计算多波段的最大最小值（最多四波段）
std::tuple<Vector4f, Vector4f> image_minmax_channel(const cv::Mat &mat, const cv::Mat &mask = cv::Mat());

// 根据直方图，截断首位，计算最大最小值
std::tuple<Vector4f, Vector4f> image_minmax_channel_clip(const cv::Mat &mat, double clip,
                                                         const cv::Mat &mask = cv::Mat());
std::tuple<float, float> image_minmax_clip(const cv::Mat &mat, double clip, const cv::Mat &mask = cv::Mat());
// 赋值单波段数据（不用 split 是因为不用创建新内存）
void image_copy_channel(const cv::Mat &src, cv::Mat &dst, int channel, const cv::Mat &mask = cv::Mat());

// 提取一波段数据
cv::Mat image_extract_channel(const cv::Mat &src, int channel);

// 带平滑的缩放
cv::Mat smooth_resize(const cv::Mat &mat, const Vector2i &dsize);

template <typename T>
T image_interpolate(const cv::Mat &mat, const Vector2f &pt, int border_type = cv::BORDER_REPLICATE);
/**
 * \brief 将影像根据最小最大值缩放到 1 - 255，其中0用来保存nodata
 * \param image 原始影像，需要是单波段
 * \param mask 无效值
 * \param percent 缩放比例
 */
cv::Mat image_percent_scale_8u(const cv::Mat &image, cv::Mat mask = cv::Mat(), double percent = 2.0);

/**
 * \brief opencv 的 remap 函数在处理影像大小大于 short 长度时会出现 bug
 * \param src
 * \param mapx 必须是 CV_32F 格式
 * \param mapy 必须是 CV_32F 格式
 */
cv::Mat remap_ipp(const cv::Mat &src, const cv::Mat &mapx, const cv::Mat &mapy, int interpolation = cv::INTER_LINEAR);

// dest = A * source
Matrix23d compute_affine(const std::vector<Vector2d> &source, const std::vector<Vector2d> &dest);

Matrix23d revers_affine(const Matrix23d &M);

// dest = H * source
Matrix3d compute_homography(const std::vector<Vector2d> &source, const std::vector<Vector2d> &dest);

} // namespace h2o

///////////////////////////////////////////////////////////////////////////////////////////
///
/// implementations
namespace h2o {
template <typename T> T image_interpolate(const cv::Mat &mat, const Vector2f &pt, int border_type) {
    int x = (int)pt.x();
    int y = (int)pt.y();

    int x0 = cv::borderInterpolate(x, mat.cols, border_type);
    int x1 = cv::borderInterpolate(x + 1, mat.cols, border_type);
    int y0 = cv::borderInterpolate(y, mat.rows, border_type);
    int y1 = cv::borderInterpolate(y + 1, mat.rows, border_type);
    float a = pt.x() - (float)x;
    float c = pt.y() - (float)y;

    const T &p00 = mat.ptr<T>(y0)[x0];
    const T &p01 = mat.ptr<T>(y1)[x0];
    const T &p10 = mat.ptr<T>(y0)[x1];
    const T &p11 = mat.ptr<T>(y1)[x1];

    T p = (p00 * (1.0f - a) + p01 * a) * (1.0f - c) + (p10 * (1.0f - a) + p11 * a) * c;
    return p;
}
} // namespace h2o

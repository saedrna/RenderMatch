/*!
 * \file geo_image.h
 *
 * \author Han
 * \date 2017/05/05
 *
 *
 */
#pragma once
#include <base/common.h>

namespace h2o {
class GeoImage {
public:
    GeoImage();
    ~GeoImage();

    /**
     * \brief 将投影坐标转为像素坐标
     * \param projection 投影坐标
     */
    Vector2d projection_to_pixel(const Vector2d &projection) const;

    /**
     * \brief 将像素坐标转换为投影坐标
     * \param pixel 影像像素坐标
     */
    Vector2d pixel_to_projection(const Vector2d &pixel) const;

    /**
     * \brief 计算变换的逆变换，其中正变换为从像素到投影，逆变换从投影到像素
     */
    void compute_inverse();

    /**
     * \brief 获取影像的opencv的mat
     */
    cv::Mat &get_image() { return image; }
    const cv::Mat &get_image() const { return image; }
    void set_image(const cv::Mat &_image) { image = _image; }

    const cv::Mat &get_mask() const { return mask; }
    void set_mask(const cv::Mat &_mask) { mask = _mask; }

    /**
     * \brief 获取正向的放射变换
     */
    const Matrix23d &get_xform() const { return xform; }

    void set_nodata(double v) { nodata = v; }
    double get_nodata() const { return nodata; }

    /**
     * \brief 获取DEM的原始路径
     */
    std::string get_path() const { return path; }
    /**
     * \brief 获取正向的仿射变换
     */
    Matrix23d &get_xform() { return xform; }
    void set_xform(const Matrix23d &xform);
    /**
     * \brief 初始化影像
     * \param rows 影像行数
     * \param cols 列数
     * \param type 影像类型，如CV_8U CV_32F等
     * \param xform 变换参数
     * \param _path 影像路径
     */
    void init_image(int rows, int cols, int type, const Matrix23d &xform, const std::string &_path = "");

    /**
     * \brief 读取8bit的灰度影像
     * \param path 影像路径，local 编码
     */
    void read_image_8u(const std::string &path);

    void read_image_multispectral_8u(const std::string &path);

    /**
     * \brief 读取原始影像，不改变格式
     * \param path 影像路径，local编码
     */
    void read_image_unchanged(const std::string &path);

    void save_image(const std::string &path, const Vector2d &offset = Vector2d::Zero());

    double get_point_t(const Vector2d &projection) const;

    /**
     * \brief 设置像素值
     * \param r
     * \param c
     * \param v
     */
    template <typename T> void set_point(int r, int c, T v);

    /**
     * \brief 获取像素值
     * \param r
     * \param c
     */
    template <typename T> T get_point(int r, int c) const;

    /**
     * \brief 获取像素值
     * \param x
     * \param y
     */
    template <typename T> T get_point(double x, double y) const;

    /**
     * \brief 获取像素值
     * \param projection
     */
    template <typename T> T get_point(const Vector2d &projection) const;

    bool point_in_bound(const Vector2d &projection) const;

    ///\brief 获取一个点的像素坐标
    Vector2i get_cell(double x, double y) const;
    Vector2i get_cell(const Vector2d &projection) const;

    ///\brief 获取一个点的像素坐标，如果在影像外则会 clamp 到边缘
    Vector2i get_cell_bounded(double x, double y) const;
    Vector2i get_cell_bounded(const Vector2d &projection) const;

    Vector2d get_pixel(const Vector2d &projection) const;
    Vector2d get_pixel(double x, double y) const;

    BoundingBox2d get_bounding_box() const;

protected:
    std::string path;
    cv::Mat image;
    cv::Mat mask;

    double nodata;

    ///\brief xform = [A,b]
    Matrix23d xform;
    Matrix23d iform;
};

using GeoImagePtr = std::shared_ptr<GeoImage>;

template <typename T> inline void GeoImage::set_point(int r, int c, T v) { image.ptr<T>(r)[c] = v; }

template <typename T> inline T GeoImage::get_point(int r, int c) const { return image.ptr<T>(r)[c]; }

template <typename T> inline T GeoImage::get_point(double x, double y) const {
    Vector2d projection(x, y);
    return get_point<T>(projection);
}

template <typename T> inline T GeoImage::get_point(const Vector2d &projection) const {
    Vector2d pixel = iform.block(0, 0, 2, 2) * projection + iform.col(2);
    int c = int(pixel.x() + 0.5);
    int r = int(pixel.y() + 0.5);

    if (c < 0) {
        c = 0;
    }
    if (c > image.cols - 1) {
        c = image.cols - 1;
    }
    if (r < 0) {
        r = 0;
    }
    if (r > image.rows - 1) {
        r = image.rows - 1;
    }

    return image.ptr<T>(r)[c];
}

} // namespace h2o

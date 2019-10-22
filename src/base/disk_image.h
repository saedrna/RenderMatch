//
// Created by han on 2018/3/26.
//

#pragma once
#include <base/common.h>

class GDALDataset;
namespace h2o {
class DiskImage {
public:
    // 不能用于创建影像，即使 read only 为否，也需要提前创建数据
    DiskImage(const std::string &path, bool read_only = true);
    ~DiskImage();

    // 根据需要的影像的 roi 和 目标影像大小，从对应的金字塔中读取数据，会同时返回 x'=Ax 的放射变换矩阵，x 为读取影像像素
    std::tuple<Matrix23d, cv::Mat> get_patch(const BoundingBox2i &roi, const Vector2i &target);
    // 读入单独某个波段的patch
    std::tuple<Matrix23d, cv::Mat> get_patch_band(const BoundingBox2i &roi, const Vector2i &target, int band);
    // 读入整个影像，仅支持四波段，读取大小由target决定，不是原始影像大小
    std::tuple<Matrix23d, cv::Mat> get_image(const Vector2i &target);
    // 读入影像的某个波段
    std::tuple<Matrix23d, cv::Mat> get_image_band(const Vector2i &target, int band);

    // 只针对单波段的整形影像，需要事先建立 lut
    std::tuple<Matrix23d, cv::Mat> get_patch_8u(const BoundingBox2i &roi, const Vector2i &target);
    std::tuple<Matrix23d, cv::Mat> get_image_8u(const Vector2i &target);

    std::string get_path() const { return path_; };

    // 影像投影信息
    std::string get_wkt_srs();
    void set_wkt_srs(const std::string &wkt);

    // 写一个 patch 到这个 roi，仅支持四波段
    void write_image_patch(const BoundingBox2i &roi, const cv::Mat &mat);
    // 写入单独一个波段的patch
    void write_image_patch_band(const BoundingBox2i &roi, const cv::Mat &mat, int band);

    // 无效值操作
    double get_nodata();
    void set_nodata(double nodata);

    // 整幅影像设置为这个值，所有波段均有效
    void fill_value(double value);

    // 建立影像金字塔，包含10层
    void build_overviews();
    // 计算所有波段的统计信息
    void compute_statistics();

    // 获取影像大小
    Vector2i get_size();

    // 获取影像在物方的包围盒
    BoundingBox2d get_bbox(int border = 0);

    // 获取仿射变换参数
    Matrix23d get_xform();
    void set_xform(const Matrix23d &xform);

    // 投影和像素坐标互转
    Vector2d to_pixel(const Vector2d &world);
    Vector2d to_world(const Vector2d &pixel);

    // 计算单波段的最大、最小值，会顾及多种不同无效值，会顾及金字塔信息
    Vector2d compute_minmax(const std::vector<double> &nodatas);

    // 获取opencv 的类型
    int get_cvtype();

    // 计算 查找表
    void calculate_lut(const Vector2i &minmax8u);

    // 创建影像金字塔
    static bool build_overviews(const std::string &path);
    // 创建影像， type 为 opencv 的类型
    static bool create_dataset(const std::string &path, const Vector2i &size, int type, const Matrix23d &xform);
    // depth 为 opencv 的值，如CV_8U
    static bool create_dataset(const std::string &path, const Vector2i &size, int depth, int channels,
                               const Matrix23d &xform);

protected:
    void init_dataset();
    void init_lut(const Vector2i &minmax8u);

    void open();
    void close();

protected:
    Matrix23d xform_;
    Matrix23d xform_reverse_;

    int channels_;     // band 数目
    int type_;         // OPENCV 对应的数据类型
    int pixel_stride_; // 一个像素的大小
    int gdal_datatype_;
    std::vector<uint8_t> lut_;        // 从原始像素坐标转换到8bit的lut
    std::string path_;                // 影像路径
    std::vector<Vector2i> pyr_sizes_; // 金字塔大小
    Vector2i size_;                   // 原始影像大小
    GDALDataset *dataset_;            // GDAL信息，在析构会重置

    bool read_only_;
};

using DiskImagePtr = std::shared_ptr<DiskImage>;
} // namespace h2o

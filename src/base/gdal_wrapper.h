/*!
 * \file gdal_wrapper.h
 *
 * \author Han
 * \date 2016/12/26
 *
 *
 */
#pragma once
#include <base/common.h>

using GDALDatasetH = void *;

namespace h2o {

#define IMAGE_8U 1
#define IMAGE_16U 2
#define IMAGE_16S 3
#define IMAGE_32U 4
#define IMAGE_32S 5
#define IMAGE_32F 6
#define IMAGE_64F 7

#define IMAGE_GRAY 1
#define IMAGE_PALETTE 2
#define IMAGE_RED 3
#define IMAGE_GREEN 4
#define IMAGE_BLUE 5
#define IMAGE_ALPHA 6

/*! Pixel data types */
enum H2oDataType {
    /*! Unknown or unspecified type */ DT_Unknown = 0,
    /*! Eight bit unsigned integer */ DT_Byte = 1,
    /*! Sixteen bit unsigned integer */ DT_UInt16 = 2,
    /*! Sixteen bit signed integer */ DT_Int16 = 3,
    /*! Thirty two bit unsigned integer */ DT_UInt32 = 4,
    /*! Thirty two bit signed integer */ DT_Int32 = 5,
    /*! Thirty two bit floating point */ DT_Float32 = 6,
    /*! Sixty four bit floating point */ DT_Float64 = 7,
    /*! Complex Int16 */ DT_CInt16 = 8,
    /*! Complex Int32 */ DT_CInt32 = 9,
    /*! Complex Float32 */ DT_CFloat32 = 10,
    /*! Complex Float64 */ DT_CFloat64 = 11,
    DT_TypeCount = 12 /* maximum type # + 1 */
};

typedef struct ImageHeader_ {
    ///\brief 宽
    int width;

    ///\brief 高
    int height;

    ///\brief 是否有金字塔
    bool pyramid;

    ///\brief 金字塔数目
    int levels;

    ///\brief 影像金字塔的行列号
    std::vector<int> pyr_width;

    ///\brief 影像金字塔的行列号
    std::vector<int> pyr_height;

    ///\brief 数据类型
    int datatype;

    ///\brief 波段
    int channel;

    ///\brief x方向的块大小
    int blockx;

    ///\brief y方向的块大小
    int blocky;

    ///\brief 波段的颜色顺序
    int color[4];

    ///\brief 真实的反映辐射分辨率的值：8bit-255 10bit-1023 12bit-4095 -1-位置
    int maxvalue;

} ImageHeader;

/*************************************************************************/
/**
 * \brief		读取四波段的多光谱影像，需要调用gdal的函数
 * \param[in]	path 影像路径
 * \param[in]	img 影像数据，可以是8bit，可以是16bit,波段顺序为B G R NIR
 */
/*************************************************************************/
void gdal_read_image(const std::string &path, cv::Mat &img);
void gdal_read_image(GDALDatasetH dataset, cv::Mat &img);
cv::Mat gdal_read_image(const std::string &path);

cv::Mat gdal_read_image_patch(const std::string &path, const Vector2i &offset, const Vector2i &size);
cv::Mat gdal_read_image_patch(GDALDatasetH dataset, const Vector2i &offset, const Vector2i &size);

/**
 * \brief 将影像写入磁盘，通常是 tif 影像
 */
void gdal_write_image(const std::string &path, cv::Mat &img);
void gdal_write_image(const std::string &path, cv::Mat &img, const Eigen::MatrixXd &xform);

void gdal_write_image_patch(const std::string &path, const cv::Mat &img, const Vector2i &offset);

/*************************************************************************/
/**
 * \brief		利用gdal读取影像头文件信息
 * \param[in]	path 影像路径
 * \param[out]	header 头文件信息
 */
/*************************************************************************/
void gdal_image_header(const std::string &path, ImageHeader &header);
void gdal_image_header(GDALDatasetH dataset, ImageHeader &header);

/**
 * \brief
 * \param path
 * \param mat
 * \param origin origin 是像素中心，而非左下角（DEM）或左上角（影像）,左下角中心
 * \param spaces
 */
void gdal_write_dem(const std::string &path, const cv::Mat &mat, const Eigen::Vector2d &origin,
                    const Eigen::Vector2d &spaces);

/**
 * \brief 获取无效值
 * \param path
 */
double gdal_get_nodata(const std::string &path);
double gdal_get_nodata(GDALDatasetH dataset);
/**
 * \brief
 * \param path
 * \param nodata
 */
void gdal_set_nodata(const std::string &path, double nodata);
void gdal_set_nodata(GDALDatasetH dataset, double nodata);

// 建立金字塔影像
void gdal_build_overviews(const std::string &path);
void gdal_build_overviews(GDALDatasetH dataset);

void gdal_compute_statistics(const std::string &path);
void gdal_compute_statistics(GDALDatasetH dataset);

/**
 * \brief
 * \param path
 */
Eigen::VectorXd gdal_get_nodata_multi(const std::string &path);

/**
 * \brief 需要注意的是gdal的transform的原点是左上角像素的左上角
 * 而通常的原点是左上角像素的中心，所以一般需要加0.5才是正确的值
 */
Eigen::MatrixXd gdal_get_transform(const std::string &path);
Eigen::MatrixXd gdal_get_transform(GDALDatasetH dataset);

void gdal_set_transform(const std::string &path, const Eigen::MatrixXd &xform);
void gdal_set_transform(GDALDatasetH dataset, const Eigen::MatrixXd &xform);

void gdal_set_projection(const std::string &path, const std::string &esri_string);
void gdal_set_projection(GDALDatasetH dataset, const std::string &esri_string);

std::string gdal_get_projection_wkt(const std::string &path);
std::string gdal_get_projection_wkt(GDALDatasetH dataset);

void gdal_set_projection_wkt(const std::string &path, const std::string &wkt);
void gdal_set_projection_wkt(GDALDatasetH dataset, const std::string &wkt);

// 导入的多边形只能是有一个ring的
using GdalPolygon2d = std::vector<Eigen::Vector2d>;
using GdalPolygon2dVec = std::vector<GdalPolygon2d>;

/**
 * \brief
 * \param path
 * \param polygons
 */
void gdal_write_polygon(const std::string &path, const GdalPolygon2dVec &polygons);

/**
 * \brief
 * \param path
 */
GdalPolygon2dVec gdal_read_polygon(const std::string &path);

/**
 * \brief
 * \param path
 */
std::vector<Eigen::Vector3d> gdal_read_points(const std::string &path);
/**
 * \brief
 * \param path
 * \param points
 */
void gdal_write_points(const std::string &path, const std::vector<Eigen::Vector3d> &points);

// 读取影像 Landspace 的旋转
int gdal_get_exif_orientation(const std::string &path);
int gdal_get_exif_orientation(GDALDatasetH dataset);

} // namespace h2o

/*!
 * \file geo_image.cpp
 *
 * \author Han
 * \date 2017/05/05
 *
 *
 */

#include <base/gdal_wrapper.h>
#include <base/geo_image.h>
#include <base/string.h>

#include <Eigen/LU>
#include <opencv2/imgproc.hpp>

using namespace std;
namespace h2o {

GeoImage::GeoImage() { nodata = DBL_MAX; }

GeoImage::~GeoImage() {}

Vector2d GeoImage::projection_to_pixel(const Vector2d &projection) const {
    Vector2d pixel;
    // X = A * p + b
    // p = A' * X - A'*b
    Matrix2d A = iform.block(0, 0, 2, 2);
    Vector2d b = iform.col(2);

    pixel = A * projection + b;
    return pixel;
}

Vector2d GeoImage::pixel_to_projection(const Vector2d &pixel) const {
    Vector2d projected;
    // X = A * p + b
    // p = A' * X - A'*b
    Matrix2d A = xform.block(0, 0, 2, 2);
    Vector2d b = xform.col(2);

    projected = A * pixel + b;
    return projected;
}

void GeoImage::compute_inverse() {
    // X = A * p + b
    // p = A' * X - A'*b
    Eigen::Matrix2d A = xform.block(0, 0, 2, 2);
    Eigen::Vector2d b = xform.col(2);

    Eigen::Matrix2d A_ = A.inverse();
    Eigen::Vector2d b_ = -A_ * b;

    iform = Matrix23d::Zero(2, 3);
    iform.block(0, 0, 2, 2) = A_;
    iform.col(2) = b_;
}

void GeoImage::set_xform(const Matrix23d &_xform) {
    xform = _xform;
    compute_inverse();
}

void GeoImage::init_image(int rows, int cols, int type, const Matrix23d &_xform, const std::string &_path) {
    path = _path;
    xform = _xform;
    image = cv::Mat::zeros(rows, cols, type);
    compute_inverse();
}

void GeoImage::read_image_8u(const string &_path) {
    path = _path;

    // 只考虑单波段影像
    gdal_read_image(path, image);
    if (image.channels() != 1) {
        read_image_multispectral_8u(_path);
        return;
    }

    if (nodata == DBL_MAX) {
        nodata = gdal_get_nodata(path);
    }

    if (image.rows == 0 || image.cols == 0) {
        LOG(WARNING) << "image " << string_utf8_to_fstream(path) << " is broken";
        return;
    }

    mask = cv::Mat::zeros(image.rows, image.cols, CV_8U);
    mask = cv::abs(image - nodata) > 0.001;

    double minv, maxv;
    cv::minMaxLoc(image, &minv, &maxv, NULL, NULL, mask);
    image.convertTo(image, CV_32F);
    image = (image - minv) / (maxv - minv);
    image.convertTo(image, CV_8U, 254.0 + 1.0);
    image.setTo(cv::Scalar(0.0), mask == 0);
    nodata = 0.0;

    // gdal 获取的是左上角的左上角
    xform = gdal_get_transform(path);
    xform(0, 2) += xform(0, 0) * 0.5 + xform(0, 1) * 0.5;
    xform(1, 2) += xform(1, 0) * 0.5 + xform(1, 1) * 0.5;

    compute_inverse();
}

void GeoImage::read_image_multispectral_8u(const std::string &_path) {
    path = _path;

    // 3波段 就是 BGR，四波段通常是 BGR 近红外
    image = gdal_read_image(path);
    if (image.channels() == 1) {
        read_image_8u(_path);
        return;
    }
    if (nodata == DBL_MAX) {
        nodata = gdal_get_nodata(path);
    }

    int nchannels = image.channels();
    std::vector<cv::Mat> mat_channels(nchannels);
    cv::split(image, mat_channels);
    mask = cv::Mat::zeros(image.rows, image.cols, CV_8U);
    mask = cv::abs(mat_channels[0] - nodata) > 0.001;

    cv::Mat mask_nodata = mask == 0;

    // 把影像纠正道 1-255，并且把 nodata 设置成 0
    for (int i = 0; i < nchannels; ++i) {
        double minv, maxv;
        cv::minMaxLoc(mat_channels[i], &minv, &maxv, NULL, NULL, mask);
        mat_channels[i].convertTo(mat_channels[i], CV_32F);
        mat_channels[i] = (mat_channels[i] - minv) / (maxv - minv);
        mat_channels[i].convertTo(mat_channels[i], CV_8U, 254.0, 1.0); // [1 - 255]
        mat_channels[i].setTo(cv::Scalar(0.0), mask_nodata);
    }
    nodata = 0.0;
    cv::merge(mat_channels, image);

    if (nchannels == 3) {
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    } else if (nchannels == 4) {
        cv::cvtColor(image, image, cv::COLOR_BGRA2GRAY);
    } else {
        image = mat_channels[0];
    }

    // gdal 获取的是左上角的左上角
    xform = gdal_get_transform(path);
    xform(0, 2) += xform(0, 0) * 0.5 + xform(0, 1) * 0.5;
    xform(1, 2) += xform(1, 0) * 0.5 + xform(1, 1) * 0.5;

    compute_inverse();
}

void GeoImage::read_image_unchanged(const std::string &_path) {
    path = _path;

    // 只考虑单波段影像
    gdal_read_image(path, image);
    xform = gdal_get_transform(path);
    xform(0, 2) += xform(0, 0) * 0.5 + xform(0, 1) * 0.5;
    xform(1, 2) += xform(1, 0) * 0.5 + xform(1, 1) * 0.5;

    compute_inverse();
}

void GeoImage::save_image(const std::string &path, const Vector2d &offset) {
    // 还原 xform
    Matrix23d xform_gdal = xform;
    xform_gdal(0, 2) -= xform(0, 0) * 0.5 + xform(0, 1) * 0.5 - offset(0);
    xform_gdal(1, 2) -= xform(1, 0) * 0.5 + xform(1, 1) * 0.5 - offset(1);
    gdal_write_image(path, image, xform_gdal);
    gdal_set_nodata(path, nodata);
    gdal_build_overviews(path);
    gdal_compute_statistics(path);
}

double GeoImage::get_point_t(const Vector2d &projection) const {
    CHECK(image.channels() == 1);
    int type = image.depth();
    if (type == CV_8U) {
        return (double)get_point<uint8_t>(projection);
    } else if (type == CV_8S) {
        return (double)get_point<int8_t>(projection);
    } else if (type == CV_16S) {
        return (double)get_point<int16_t>(projection);
    } else if (type == CV_16U) {
        return (double)get_point<uint16_t>(projection);
    } else if (type == CV_32S) {
        return (double)get_point<int32_t>(projection);
    } else if (type == CV_32F) {
        return (double)get_point<float>(projection);
    }
    return 0.0;
}

bool GeoImage::point_in_bound(const Vector2d &projection) const {
    Vector2d pixel = projection_to_pixel(projection);
    int c = int(pixel.x() + 0.5);
    int r = int(pixel.y() + 0.5);

    return c >= 0 && c < image.cols && r >= 0 && r < image.rows;
}

Vector2i GeoImage::get_cell(double x, double y) const { return get_cell(Vector2d(x, y)); }

Vector2i GeoImage::get_cell(const Vector2d &projection) const {
    Vector2d pixel = iform.block(0, 0, 2, 2) * projection + iform.col(2);
    int c = int(pixel.x() + 0.5);
    int r = int(pixel.y() + 0.5);

    return Vector2i(c, r);
}
Vector2i GeoImage::get_cell_bounded(double x, double y) const { return get_cell_bounded(Vector2d(x, y)); }

Vector2i GeoImage::get_cell_bounded(const Vector2d &projection) const {
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

    return Vector2i(c, r);
}
Vector2d GeoImage::get_pixel(const Vector2d &projection) const {
    return iform.block(0, 0, 2, 2) * projection + iform.col(2);
}
Vector2d GeoImage::get_pixel(double x, double y) const { return get_pixel(Vector2d(x, y)); }

BoundingBox2d GeoImage::get_bounding_box() const {
    BoundingBox2d bbox;
    std::vector<Vector2d> pixels = {
        Vector2d(0, 0), Vector2d(0, image.rows), Vector2d(image.cols, image.rows), {image.cols, 0}};
    for (const Vector2d &pixel : pixels) {
        bbox.extend(pixel_to_projection(pixel));
    }
    return bbox;
}
} // namespace h2o

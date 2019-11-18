//
// Created by han on 2018/3/26.
//

#include <base/disk_image.h>
#include <base/filesystem.h>
#include <base/gdal_wrapper.h>
#include <base/imgproc.h>
#include <base/string.h>

#include <QFile>
#include <QTextStream>
#include <gdal.h>
#include <gdal_priv.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace h2o {
DiskImage::DiskImage(const std::string &path, bool read_only) {
    read_only_ = read_only;
    path_ = path;
    size_ = Vector2i::Zero();
    dataset_ = nullptr;
    channels_ = -1;
    type_ = -1;

    if (channels_ == 1 && type_ == CV_16U) {
        init_lut(Vector2i(0, 255));
    }
}

DiskImage::~DiskImage() {
    if (dataset_ != nullptr) {
        GDALClose(dataset_);
        dataset_ = nullptr;
    }
}

std::tuple<Matrix23d, cv::Mat> DiskImage::get_patch(const BoundingBox2i &roi, const Vector2i &target) {

    // 内部会判断是否已经初始化
    init_dataset();

    Matrix23d xform;
    cv::Mat patch;

    if (roi.min()(0) < 0 || roi.min()(1) < 0 || roi.max()(0) > size_(0) || roi.max()(1) > size_(1)) {
        LOG(WARNING) << "Image roi is outside dataset with corners of (" << roi.min().transpose() << ") and ("
                     << roi.max().transpose() << ")";
        return std::make_tuple(xform, patch);
    }

    if (target(0) > roi.sizes()(0) || target(1) > roi.sizes()(1)) {
        patch = cv::Mat::zeros(roi.sizes()(1), roi.sizes()(0), type_);
    } else {
        patch = cv::Mat::zeros(target(1), target(0), type_);
    }

    int offx = roi.min()(0);
    int offy = roi.min()(1);
    int width = roi.sizes()(0);
    int height = roi.sizes()(1);

    open();
    for (int i = 0; i < channels_; ++i) {
        GDALRasterBand *band = dataset_->GetRasterBand(i + 1);
        band->RasterIO(GF_Read, offx, offy, width, height, patch.data + pixel_stride_ / channels_ * i, patch.cols,
                       patch.rows, (GDALDataType)gdal_datatype_, pixel_stride_, patch.step[0], NULL);
    }
    //     close();

    // 计算变换信息
    std::vector<cv::Point2f> origin2d = {cv::Point2f(offx - 0.5, offy - 0.5),
                                         cv::Point2f(offx + width - 0.5, offy - 0.5),
                                         cv::Point2f(offx + width - 0.5, offy + height - 0.5)};
    std::vector<cv::Point2f> target2d = {cv::Point2f(-0.5, -0.5), cv::Point2f(patch.cols - 0.5, -0.5),
                                         cv::Point2f(patch.cols - 0.5, patch.rows - 0.5)};

    // origin = M * target;
    cv::Mat M = cv::getAffineTransform(target2d, origin2d);
    cv::cv2eigen(M, xform);

    return std::make_tuple(xform, patch);
}

std::tuple<Matrix23d, cv::Mat> DiskImage::get_patch_band(const BoundingBox2i &roi, const Vector2i &target, int nband) {
    // 内部会判断是否已经初始化
    init_dataset();

    Matrix23d xform;
    cv::Mat patch;

    if (roi.min()(0) < 0 || roi.min()(1) < 0 || roi.max()(0) > size_(0) || roi.max()(1) > size_(1)) {
        LOG(WARNING) << "Image roi is outside dataset with corners of (" << roi.min().transpose() << ") and ("
                     << roi.max().transpose() << ")";
        return std::make_tuple(xform, patch);
    }
    if (nband >= channels_) {
        LOG(WARNING) << "Required channel #" << nband << " is larger than maximum number " << channels_;
        return std::make_tuple(xform, patch);
    }

    int cvdepth = CV_MAT_DEPTH(type_);
    if (target(0) > roi.sizes()(0) || target(1) > roi.sizes()(1)) {
        patch = cv::Mat::zeros(roi.sizes()(1), roi.sizes()(0), cvdepth);
    } else {
        patch = cv::Mat::zeros(target(1), target(0), cvdepth);
    }

    int offx = roi.min()(0);
    int offy = roi.min()(1);
    int width = roi.sizes()(0);
    int height = roi.sizes()(1);

    open();
    GDALRasterBand *band = dataset_->GetRasterBand(nband + 1);
    band->RasterIO(GF_Read, offx, offy, width, height, patch.data, patch.cols, patch.rows, (GDALDataType)gdal_datatype_,
                   pixel_stride_ / channels_, patch.step[0], NULL);
    close();

    // 计算变换信息
    std::vector<cv::Point2f> origin2d = {cv::Point2f(offx - 0.5, offy - 0.5),
                                         cv::Point2f(offx + width - 0.5, offy - 0.5),
                                         cv::Point2f(offx + width - 0.5, offy + height - 0.5)};
    std::vector<cv::Point2f> target2d = {cv::Point2f(-0.5, -0.5), cv::Point2f(patch.cols - 0.5, -0.5),
                                         cv::Point2f(patch.cols - 0.5, patch.rows - 0.5)};

    // origin = M * target;
    cv::Mat M = cv::getAffineTransform(target2d, origin2d);
    cv::cv2eigen(M, xform);

    return std::make_tuple(xform, patch);
}

std::tuple<Matrix23d, cv::Mat> DiskImage::get_image(const Vector2i &target) {
    // 内部会判断是否已经初始化
    init_dataset();

    Matrix23d xform;
    cv::Mat patch;

    if (target(0) > size_(0) || target(1) > size_(1)) {
        patch = cv::Mat::zeros(size_(1), size_(0), type_);
    } else {
        patch = cv::Mat::zeros(target(1), target(0), type_);
    }
    open();
    for (int i = 0; i < channels_; ++i) {
        GDALRasterBand *band = dataset_->GetRasterBand(i + 1);
        band->RasterIO(GF_Read, 0, 0, size_(0), size_(1), patch.data + pixel_stride_ / channels_ * i, patch.cols,
                       patch.rows, (GDALDataType)gdal_datatype_, pixel_stride_, patch.step[0], NULL);
    }
    close();

    // 计算变换信息
    std::vector<cv::Point2f> origin2d = {cv::Point2f(-0.5, -0.5), cv::Point2f(size_(0) - 0.5, -0.5),
                                         cv::Point2f(size_(0) - 0.5, size_(1) - 0.5)};
    std::vector<cv::Point2f> target2d = {cv::Point2f(-0.5, -0.5), cv::Point2f(patch.cols - 0.5, -0.5),
                                         cv::Point2f(patch.cols - 0.5, patch.rows - 0.5)};

    // origin = M * target;
    cv::Mat M = cv::getAffineTransform(target2d, origin2d);
    cv::cv2eigen(M, xform);

    return std::make_tuple(xform, patch);
}

std::tuple<Matrix23d, cv::Mat> DiskImage::get_image_band(const Vector2i &target, int nband) {
    // 内部会判断是否已经初始化
    init_dataset();

    Matrix23d xform;
    cv::Mat patch;

    int cvdepth = CV_MAT_DEPTH(type_);

    if (target(0) > size_(0) || target(1) > size_(1)) {
        patch = cv::Mat::zeros(size_(1), size_(0), cvdepth);
    } else {
        patch = cv::Mat::zeros(target(1), target(0), cvdepth);
    }

    open();
    GDALRasterBand *band = dataset_->GetRasterBand(nband + 1);
    band->RasterIO(GF_Read, 0, 0, size_(0), size_(1), patch.data, patch.cols, patch.rows, (GDALDataType)gdal_datatype_,
                   pixel_stride_ / channels_, patch.step[0], NULL);
    close();

    // 计算变换信息
    std::vector<cv::Point2f> origin2d = {cv::Point2f(-0.5, -0.5), cv::Point2f(size_(0) - 0.5, -0.5),
                                         cv::Point2f(size_(0) - 0.5, size_(1) - 0.5)};
    std::vector<cv::Point2f> target2d = {cv::Point2f(-0.5, -0.5), cv::Point2f(patch.cols - 0.5, -0.5),
                                         cv::Point2f(patch.cols - 0.5, patch.rows - 0.5)};

    // origin = M * target;
    cv::Mat M = cv::getAffineTransform(target2d, origin2d);
    cv::cv2eigen(M, xform);

    return std::make_tuple(xform, patch);
}

std::tuple<Matrix23d, cv::Mat> DiskImage::get_patch_8u(const BoundingBox2i &roi, const Vector2i &target) {
    // 内部会判断是否已经初始化
    init_dataset();

    Matrix23d xform;
    cv::Mat patch;

    if (roi.min()(0) < 0 || roi.min()(1) < 0 || roi.max()(0) > size_(0) || roi.max()(1) > size_(1)) {
        LOG(WARNING) << "Image roi is outside dataset with corners of (" << roi.min().transpose() << ") and ("
                     << roi.max().transpose() << ")";
        return std::make_tuple(xform, patch);
    }

    if (type_ != CV_16U) {
        LOG(WARNING) << "Implicit conversion to 8bit only support 16bit one channel image";
        return std::make_tuple(xform, patch);
    }

    std::tie(xform, patch) = get_patch(roi, target);
    cv::Mat patch8u = cv::Mat::zeros(patch.rows, patch.cols, CV_8U);

    uint16_t *patch_data = (uint16_t *)patch.data;
    uint8_t *patch8u_data = patch8u.data;

    for (int i = 0; i < patch.rows * patch.cols; ++i) {
        patch8u_data[i] = lut_[patch_data[i]];
    }
    return std::make_tuple(xform, patch8u);
}

std::tuple<Matrix23d, cv::Mat> DiskImage::get_image_8u(const Vector2i &target) {
    // 内部会判断是否已经初始化
    init_dataset();

    Matrix23d xform;
    cv::Mat patch;
    if (type_ != CV_16U) {
        return std::make_tuple(xform, patch);
    }

    std::tie(xform, patch) = get_image(target);
    cv::Mat patch8u = cv::Mat::zeros(patch.rows, patch.cols, CV_8U);

    uint16_t *patch_data = (uint16_t *)patch.data;
    uint8_t *patch8u_data = patch8u.data;

    for (int i = 0; i < patch.rows * patch.cols; ++i) {
        patch8u_data[i] = lut_[patch_data[i]];
    }
    return std::make_tuple(xform, patch8u);
}

std::string DiskImage::get_wkt_srs() {
    // 内部会判断是否已经初始化
    init_dataset();

    open();
    std::string srs = gdal_get_projection_wkt(dataset_);
    close();
    return srs;
}

void DiskImage::set_wkt_srs(const std::string &wkt) {
    // 内部会判断是否已经初始化
    init_dataset();

    if (read_only_) {
        return;
    }
    open();
    gdal_set_projection_wkt(dataset_, wkt);
    close();
}

void DiskImage::write_image_patch(const BoundingBox2i &roi, const cv::Mat &mat) {
    // 内部会判断是否已经初始化
    init_dataset();

    if (mat.rows != roi.sizes()(1) || mat.cols != roi.sizes()(0)) {
        LOG(WARNING) << "Image do not have the same size as roi in write patch";
        return;
    }
    if (read_only_) {
        return;
    }

    open();
    for (int i = 0; i < channels_; ++i) {
        GDALRasterBandH band = GDALGetRasterBand(dataset_, i + 1);

        int pos = i * pixel_stride_ / channels_;
        int offx = roi.min()(0);
        int offy = roi.min()(1);
        int width = mat.cols;
        int height = mat.rows;

        CPLErr err = GDALRasterIO(band, GF_Write, offx, offy, width, height, mat.data + pos, width, height,
                                  (GDALDataType)gdal_datatype_, pixel_stride_, mat.step[0]);
        CHECK(err == CE_None) << "Error in write data into image";
    }
    close();

    return;
}

void DiskImage::write_image_patch_band(const BoundingBox2i &roi, const cv::Mat &mat, int nband) {
    // 内部会判断是否已经初始化
    init_dataset();

    if (mat.rows != roi.sizes()(1) || mat.cols != roi.sizes()(0)) {
        LOG(WARNING) << "Image do not have the same size as roi in write patch";
        return;
    }
    if (read_only_) {
        return;
    }
    open();
    GDALRasterBandH band = GDALGetRasterBand(dataset_, nband + 1);

    int offx = roi.min()(0);
    int offy = roi.min()(1);
    int width = mat.cols;
    int height = mat.rows;

    CPLErr err = GDALRasterIO(band, GF_Write, offx, offy, width, height, mat.data, width, height,
                              (GDALDataType)gdal_datatype_, pixel_stride_ / channels_, mat.step[0]);
    CHECK(err == CE_None) << "Error in write data into image";
    close();

    return;
}

double DiskImage::get_nodata() {
    // 内部会判断是否已经初始化
    init_dataset();

    open();
    double nodata = gdal_get_nodata(dataset_);
    close();
    return nodata;
}

void DiskImage::set_nodata(double nodata) {
    // 内部会判断是否已经初始化
    init_dataset();

    if (read_only_) {
        return;
    }
    open();
    gdal_set_nodata(dataset_, nodata);
    close();
}

void DiskImage::fill_value(double value) {
    // 内部会判断是否已经初始化
    init_dataset();

    if (read_only_) {
        return;
    }
    open();
    int nbands = dataset_->GetRasterCount();
    for (int i = 0; i < nbands; ++i) {
        GDALRasterBand *band = dataset_->GetRasterBand(i + 1);
        band->Fill(value);
    }
    close();
}

bool DiskImage::build_overviews(const std::string &path) {

    GDALAllRegister();
    GDALDataset *dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_Update);
    ImageHeader header;
    gdal_image_header(dataset, header);
    if (header.levels == 0) {
        gdal_build_overviews(dataset);
    }
    GDALClose(dataset);
    return true;
}

void DiskImage::build_overviews() {
    // 内部会判断是否已经初始化
    init_dataset();

    open();
    gdal_build_overviews(dataset_);
    close();
}

void DiskImage::compute_statistics() {
    // 内部会判断是否已经初始化
    init_dataset();

    if (read_only_) {
        return;
    }
    open();
    for (int b = 0; b < channels_; ++b) {
        GDALRasterBand *band = dataset_->GetRasterBand(b + 1);
        double minv, maxv, meanv, stdv;
        band->ComputeStatistics(FALSE, &minv, &maxv, &meanv, &stdv, NULL, NULL);
        band->SetStatistics(minv, maxv, meanv, stdv);
    }
    close();
}

Vector2i DiskImage::get_size() {
    init_dataset();
    return size_;
}

bool DiskImage::create_dataset(const std::string &path, const Vector2i &size, int type, const Matrix23d &xform) {
    GDALAllRegister();
    std::string extension = string_to_lower(get_extension_name(path));

    GDALDriver *driver = nullptr;
    char **options = nullptr;

    if (extension == ".img") {
        driver = (GDALDriver *)GDALGetDriverByName("HFA");
        CSLSetNameValue(options, "COMPRESSED", "YES");
    } else if (extension == ".tiff" || extension == ".tif") {
        driver = (GDALDriver *)GDALGetDriverByName("GTiff");
        options = CSLSetNameValue(options, "TILED", "YES");
        options = CSLSetNameValue(options, "COMPRESS", "LZW");
        options = CSLSetNameValue(options, "TFW", "YES");
        if (size(0) >= 10000 || size(1) >= 10000) {
            options = CSLSetNameValue(options, "BIGTIFF", "YES");
        }
    }
    if (driver == nullptr) {
        LOG(WARNING) << "Can not create GDAL Driver";
        CSLDestroy(options);
        return false;
    }

    if (file_exist(path)) {
        driver->Delete(path.c_str());
    }

    int nchannels = CV_MAT_CN(type);
    int depth = CV_MAT_DEPTH(type);

    GDALDataType dtype;
    if (depth == CV_8U) {
        dtype = GDT_Byte;
    } else if (depth == CV_16U) {
        dtype = GDT_UInt16;
    } else if (depth == CV_16S) {
        dtype = GDT_Int16;
    } else if (depth == CV_32S) {
        dtype = GDT_Int32;
    } else if (depth == CV_32F) {
        dtype = GDT_Float32;
    } else if (depth == CV_64F) {
        dtype = GDT_Float64;
    }

    GDALDataset *dataset = driver->Create(path.c_str(), size(0), size(1), nchannels, dtype, options);
    Matrix23d xform2 = xform;
    xform2(0, 2) -= xform(0, 0) * 0.5 + xform(0, 1) * 0.5;
    xform2(1, 2) -= xform(1, 0) * 0.5 + xform(1, 1) * 0.5;

    gdal_set_transform(dataset, xform2);

    GDALClose(dataset);
    CSLDestroy(options);

    return true;
}

bool DiskImage::create_dataset(const std::string &path, const Vector2i &size, int depth, int channels,
                               const Matrix23d &xform) {
    GDALAllRegister();
    std::string extension = string_to_lower(get_extension_name(path));

    GDALDriver *driver = nullptr;
    char **options = nullptr;

    if (extension == ".img") {
        driver = (GDALDriver *)GDALGetDriverByName("HFA");
        CSLSetNameValue(options, "COMPRESSED", "YES");
    } else if (extension == ".tiff" || extension == ".tif") {
        driver = (GDALDriver *)GDALGetDriverByName("GTiff");
        options = CSLSetNameValue(options, "TILED", "YES");
        options = CSLSetNameValue(options, "COMPRESS", "LZW");
        options = CSLSetNameValue(options, "TFW", "YES");
    }
    if (driver == nullptr) {
        LOG(WARNING) << "Can not create GDAL Driver";
        CSLDestroy(options);
        return false;
    }

    if (file_exist(path)) {
        driver->Delete(path.c_str());
    }

    int nchannels = channels;

    GDALDataType dtype;
    if (depth == CV_8U) {
        dtype = GDT_Byte;
    } else if (depth == CV_16U) {
        dtype = GDT_UInt16;
    } else if (depth == CV_16S) {
        dtype = GDT_Int16;
    } else if (depth == CV_32S) {
        dtype = GDT_Int32;
    } else if (depth == CV_32F) {
        dtype = GDT_Float32;
    } else if (depth == CV_64F) {
        dtype = GDT_Float64;
    }

    GDALDataset *dataset = driver->Create(path.c_str(), size(0), size(1), nchannels, dtype, options);
    Matrix23d xform2 = xform;
    xform2(0, 2) -= xform(0, 0) * 0.5 + xform(0, 1) * 0.5;
    xform2(1, 2) -= xform(1, 0) * 0.5 + xform(1, 1) * 0.5;

    gdal_set_transform(dataset, xform2);

    GDALClose(dataset);
    CSLDestroy(options);

    return true;
}

BoundingBox2d DiskImage::get_bbox(int border) {
    // 内部会判断是否已经初始化
    init_dataset();

    Matrix23d xform = xform_;

    BoundingBox2d bbox;
    Vector2i size = get_size();
    Vector2d pixel(border, border);
    pixel = xform * pixel.homogeneous();
    bbox.extend(pixel);

    pixel = Vector2d(size(0) - border, size(1) - border);
    pixel = xform * pixel.homogeneous();
    bbox.extend(pixel);

    return bbox;
}

Matrix23d DiskImage::get_xform() {
    // 内部会判断是否已经初始化
    init_dataset();

    return xform_;
}

void DiskImage::set_xform(const Matrix23d &xform) {
    // 内部会判断是否已经初始化
    init_dataset();

    xform_ = xform;
    Matrix2d A = xform.leftCols(2);
    Vector2d b = xform.col(2);
    xform_reverse_.leftCols(2) = A.inverse();
    xform_reverse_.col(2) = -A.inverse() * b;

    if (!read_only_) {
        Matrix23d xform2 = xform;
        xform2(0, 2) -= xform(0, 0) * 0.5 + xform(0, 1) * 0.5;
        xform2(1, 2) -= xform(1, 0) * 0.5 + xform(1, 1) * 0.5;
        open();
        gdal_set_transform(dataset_, xform2);
        close();

        // 更新 tfw 文件
        std::string name = get_filename_noext(path_);
        std::string dir = get_directory(path_);
        name = join_paths(dir, name + ".tfw");
        // TFW 记录的是中心点，所以不需要处理
        QFile file(QString::fromStdString(name));
        file.open(QFile::WriteOnly);
        QTextStream ts(&file);
        ts.setCodec("UTF-8");
        ts << QString::number(xform(0, 0), 'f', 2) << "\n";
        ts << QString::number(xform(0, 1), 'f', 2) << "\n";
        ts << QString::number(xform(1, 0), 'f', 2) << "\n";
        ts << QString::number(xform(1, 1), 'f', 2) << "\n";
        ts << QString::number(xform(0, 2), 'f', 2) << "\n";
        ts << QString::number(xform(1, 2), 'f', 2) << "\n";
    }
}

Vector2d DiskImage::to_pixel(const Vector2d &world) {
    // 内部会判断是否已经初始化
    init_dataset();

    return xform_reverse_ * world.homogeneous();
}

Vector2d DiskImage::to_world(const Vector2d &pixel) {
    // 内部会判断是否已经初始化
    init_dataset();

    return xform_ * pixel.homogeneous();
}

Vector2d DiskImage::compute_minmax(const std::vector<double> &nodatas) {
    // 内部会判断是否已经初始化
    init_dataset();

    if (channels_ != 1) {
        return Vector2d::Zero();
    }

    Vector2d minmax;

    double scale = (double)size_(0) / 1024;
    Vector2i target(1024, size_(1) * scale + 0.5);

    cv::Mat mat;
    Matrix23d xform;
    std::tie(xform, mat) = get_image(target);

    int rows = mat.rows;
    int cols = mat.cols;
    cv::Mat mask(rows, cols, CV_8U, 255);

    for (double nodata : nodatas) {
        cv::Mat temp = cv::abs(mat - nodata) < 0.1f;
        mask.setTo(cv::Scalar(0), temp);
    }

    minmax = image_minmax(mat, mask);
    return minmax;
}

int DiskImage::get_cvtype() {
    // 内部会判断是否已经初始化
    init_dataset();

    return type_;
}

void DiskImage::calculate_lut(const Vector2i &minmax8u) {
    // 内部会判断是否已经初始化
    init_dataset();

    if (channels_ == 1 && type_ == CV_16U) {
        init_lut(minmax8u);
    }
}

void DiskImage::init_dataset() {
    if (size_(0) != 0 && size_(1) != 0) {
        // 已经初始化了
        return;
    }
    GDALAllRegister();

    if (read_only_) {
        dataset_ = (GDALDataset *)GDALOpen(path_.c_str(), GA_ReadOnly);
    } else {
        dataset_ = (GDALDataset *)GDALOpen(path_.c_str(), GA_Update);
    }

    ImageHeader header;
    gdal_image_header(dataset_, header);
    //     if (header.levels == 0 && read_only_) {
    //         GDALClose(dataset_);
    //         dataset_ = (GDALDataset *)GDALOpen(path_.c_str(), GA_Update);
    //         gdal_build_overviews(path_);
    //         GDALClose(dataset_);
    //         if (read_only_) {
    //             dataset_ = (GDALDataset *)GDALOpen(path_.c_str(), GA_ReadOnly);
    //         } else {
    //             dataset_ = (GDALDataset *)GDALOpen(path_.c_str(), GA_Update);
    //         }
    //     }
    gdal_image_header(dataset_, header);
    size_(0) = header.width;
    size_(1) = header.height;

    pyr_sizes_.resize(header.levels);
    for (int i = 0; i < pyr_sizes_.size(); ++i) {
        pyr_sizes_[i](0) = header.pyr_width[i];
        pyr_sizes_[i](1) = header.pyr_height[i];
    }

    channels_ = header.channel;
    gdal_datatype_ = header.datatype;
    switch (header.datatype) {
    case DT_Byte:
        if (channels_ == 1) {
            type_ = CV_8U;
        } else if (channels_ == 3) {
            type_ = CV_8UC3;
        } else if (channels_ == 4) {
            type_ = CV_8UC4;
        }
        pixel_stride_ = channels_ * 1;
        break;
    case DT_Int16:
        if (channels_ == 1) {
            type_ = CV_16S;
        } else if (channels_ == 3) {
            type_ = CV_16SC3;
        } else if (channels_ == 4) {
            type_ = CV_16SC4;
        }
        pixel_stride_ = channels_ * 2;
        break;
    case DT_UInt16:
        if (channels_ == 1) {
            type_ = CV_16U;
        } else if (channels_ == 3) {
            type_ = CV_16UC3;
        } else if (channels_ == 4) {
            type_ = CV_16UC4;
        }
        pixel_stride_ = channels_ * 2;
        break;
    case DT_Int32:
        if (channels_ == 1) {
            type_ = CV_32S;
        } else if (channels_ == 3) {
            type_ = CV_32SC3;
        } else if (channels_ == 4) {
            type_ = CV_32SC4;
        }
        pixel_stride_ = channels_ * 4;
        break;
    case DT_Float32:
        if (channels_ == 1) {
            type_ = CV_32F;
        } else if (channels_ == 3) {
            type_ = CV_32FC3;
        } else if (channels_ == 4) {
            type_ = CV_32FC4;
        }
        pixel_stride_ = channels_ * 4;
        break;
    default:
        CHECK(false) << "Datatype not supported";
        break;
    }

    Matrix23d xform = gdal_get_transform(dataset_);
    xform(0, 2) += xform(0, 0) * 0.5 + xform(0, 1) * 0.5;
    xform(1, 2) += xform(1, 0) * 0.5 + xform(1, 1) * 0.5;
    xform_ = xform;

    Matrix2d A = xform.leftCols(2);
    Vector2d b = xform.col(2);
    xform_reverse_.leftCols(2) = A.inverse();
    xform_reverse_.col(2) = -A.inverse() * b;

    close();
}

void DiskImage::init_lut(const Vector2i &minmax8u) {
    // 内部会判断是否已经初始化
    init_dataset();

    cv::Mat mat;
    Matrix23d xform;

    double scale = std::max(size_(0), size_(1)) / 1024;
    Vector2i target(size_(0) / scale, size_(1) / scale);
    std::tie(xform, mat) = get_image(target);

    float minmax[2];
    std::tie(minmax[0], minmax[1]) = image_minmax_clip(mat, 0.5);

    float length = minmax[1] - minmax[0];

    lut_.resize(std::pow(2, 16), 255);
    for (int i = 0; i < lut_.size(); ++i) {
        if (i < minmax[0]) {
            lut_[i] = minmax8u(0);
            continue;
        }
        if (i > minmax[1]) {
            lut_[i] = minmax8u(1);
            continue;
        }
        lut_[i] = minmax8u(0) + (i - minmax[0]) / (minmax[1] - minmax[0]) * (minmax8u(1) - minmax8u(0));
    }
}

void DiskImage::open() {
    if (dataset_) {
        return;
    }
    if (read_only_) {
        dataset_ = (GDALDataset *)GDALOpen(path_.c_str(), GA_ReadOnly);
    } else {
        dataset_ = (GDALDataset *)GDALOpen(path_.c_str(), GA_Update);
    }
}

void DiskImage::close() {
    GDALClose(dataset_);
    dataset_ = nullptr;
}

} // namespace h2o

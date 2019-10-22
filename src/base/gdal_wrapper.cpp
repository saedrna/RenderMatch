/*!
 * \file gdal_wrapper.cpp
 *
 * \author Han
 * \date 2016/12/26
 *
 *
 */
#include <base/filesystem.h>
#include <base/gdal_wrapper.h>
#include <base/string.h>

#include <gdal.h>
#include <gdal_frmts.h>
#include <gdal_priv.h>
#include <ogrsf_frmts.h>

#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
namespace h2o {

void gdal_read_image(const std::string &path, cv::Mat &img) {
    GDALDatasetH dataset;
    GDALAllRegister();
    dataset = GDALOpen(path.c_str(), GA_ReadOnly);
    if (dataset == NULL) {
        return;
    }
    gdal_read_image(dataset, img);
    GDALClose(dataset);
}

void gdal_read_image(GDALDatasetH dataset, cv::Mat &img) {
    GDALAllRegister();
    if (dataset == NULL) {
        return;
    }
    int width, height;
    int channels;

    channels = GDALGetRasterCount(dataset);
    width = GDALGetRasterXSize(dataset);
    height = GDALGetRasterYSize(dataset);

    GDALRasterBandH band = GDALGetRasterBand(dataset, 1);
    GDALDataType data_type = GDALGetRasterDataType(band);

    int pixel_depth;
    int pixel_stride;

    if (data_type == GDT_Byte) {
        if (channels == 1)
            img = cv::Mat(height, width, CV_8UC1);
        if (channels == 3)
            img = cv::Mat(height, width, CV_8UC3);
        if (channels == 4)
            img = cv::Mat(height, width, CV_8UC4);
        pixel_depth = 1;
    } else if (data_type == GDT_UInt16) {
        if (channels == 1)
            img = cv::Mat(height, width, CV_16UC1);
        if (channels == 3)
            img = cv::Mat(height, width, CV_16UC3);
        if (channels == 4)
            img = cv::Mat(height, width, CV_16UC4);
        pixel_depth = 2;
    } else if (data_type == GDT_Int16) {
        if (channels == 1)
            img = cv::Mat(height, width, CV_16SC1);
        if (channels == 3)
            img = cv::Mat(height, width, CV_16SC3);
        if (channels == 4)
            img = cv::Mat(height, width, CV_16SC4);
        pixel_depth = 2;
    } else if (data_type == GDT_Int32) {
        if (channels == 1)
            img = cv::Mat(height, width, CV_32S);
        if (channels == 3)
            img = cv::Mat(height, width, CV_32SC3);
        if (channels == 4)
            img = cv::Mat(height, width, CV_32SC4);
        pixel_depth = 4;
    } else if (data_type == GDT_Float32) {
        if (channels == 1)
            img = cv::Mat(height, width, CV_32FC1);
        if (channels == 3)
            img = cv::Mat(height, width, CV_32FC3);
        if (channels == 4)
            img = cv::Mat(height, width, CV_32FC4);
        pixel_depth = 4;
    } else if (data_type == GDT_Float64) {
        if (channels == 1)
            img = cv::Mat(height, width, CV_64FC1);
        if (channels == 3)
            img = cv::Mat(height, width, CV_64FC3);
        if (channels == 4)
            img = cv::Mat(height, width, CV_64FC4);
        pixel_depth = 8;
    } else {
        CHECK(false) << "dataset is not supported";
    }

    pixel_stride = pixel_depth * img.channels();

    for (int i = 0; i < channels; ++i) {
        GDALRasterBandH band = GDALGetRasterBand(dataset, i + 1);
        int band_x, band_y;
        GDALGetBlockSize(band, &band_x, &band_y);
        data_type = GDALGetRasterDataType(band);

        int pos = i * pixel_depth;
        CPLErr err = GDALRasterIO(band, GF_Read, 0, 0, width, height, img.data + pos, width, height, data_type,
                                  pixel_stride, pixel_stride * width);
        if (err != CE_None) {
            img = cv::Mat();
            return;
        }
    }

    ImageHeader header;
    gdal_image_header(dataset, header);

    //判断是否要更换波段顺序,OPENCV中存储的是BGR的顺序
    if (header.color[0] == IMAGE_RED && header.color[1] == IMAGE_GREEN && header.color[2] == IMAGE_BLUE &&
        data_type == GDT_Byte) {
        if (header.channel == 3)
            cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        else if (header.channel == 4)
            cv::cvtColor(img, img, cv::COLOR_RGBA2BGRA);
    }
}

cv::Mat gdal_read_image(const std::string &path) {
    cv::Mat mat;
    gdal_read_image(path, mat);
    return mat;
}

cv::Mat gdal_read_image_patch(const std::string &path, const Vector2i &offset, const Vector2i &size) {
    GDALDataset *dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_ReadOnly);
    CHECK(dataset != NULL);
    cv::Mat mat = gdal_read_image_patch(dataset, offset, size);
    GDALClose(dataset);
    return mat;
}

cv::Mat gdal_read_image_patch(GDALDatasetH _dataset, const Vector2i &offset, const Vector2i &size) {
    cv::Mat patch;
    int width = size.x();
    int height = size.y();

    GDALAllRegister();
    GDALDataset *dataset = (GDALDataset *)_dataset;
    int channels = dataset->GetRasterCount();
    GDALDataType dataype = dataset->GetRasterBand(1)->GetRasterDataType();
    int total_width = dataset->GetRasterXSize();
    int total_height = dataset->GetRasterYSize();

    int pixel_depth;
    if (dataype == GDT_Byte) {
        if (channels == 1)
            patch = cv::Mat(height, width, CV_8UC1);
        if (channels == 3)
            patch = cv::Mat(height, width, CV_8UC3);
        if (channels == 4)
            patch = cv::Mat(height, width, CV_8UC4);
        pixel_depth = 1;
    } else if (dataype == GDT_UInt16) {
        if (channels == 1)
            patch = cv::Mat(height, width, CV_16UC1);
        if (channels == 3)
            patch = cv::Mat(height, width, CV_16UC3);
        if (channels == 4)
            patch = cv::Mat(height, width, CV_16UC4);
        pixel_depth = 2;
    } else if (dataype == GDT_Int16) {
        if (channels == 1)
            patch = cv::Mat(height, width, CV_16SC1);
        if (channels == 3)
            patch = cv::Mat(height, width, CV_16SC3);
        if (channels == 4)
            patch = cv::Mat(height, width, CV_16SC4);
        pixel_depth = 2;
    } else if (dataype == GDT_Int32) {
        if (channels == 1)
            patch = cv::Mat(height, width, CV_32S);
        if (channels == 3)
            patch = cv::Mat(height, width, CV_32SC3);
        if (channels == 4)
            patch = cv::Mat(height, width, CV_32SC4);
        pixel_depth = 4;
    } else if (dataype == GDT_Float32) {
        if (channels == 1)
            patch = cv::Mat(height, width, CV_32FC1);
        if (channels == 3)
            patch = cv::Mat(height, width, CV_32FC3);
        if (channels == 4)
            patch = cv::Mat(height, width, CV_32FC4);
        pixel_depth = 4;
    } else if (dataype == GDT_Float64) {
        if (channels == 1)
            patch = cv::Mat(height, width, CV_64FC1);
        if (channels == 3)
            patch = cv::Mat(height, width, CV_64FC3);
        if (channels == 4)
            patch = cv::Mat(height, width, CV_64FC4);
        pixel_depth = 8;
    }

    int pixel_stride = pixel_depth * patch.channels();
    int line_stride = patch.step[0];
    vector<int> channels_map(channels);
    for (int i = 0; i < channels; ++i) {
        channels_map[i] = i + 1;
    }
    dataset->RasterIO(GF_Read, offset.x(), offset.y(), width, height, (void *)patch.data, width, height, dataype,
                      channels, 0, pixel_stride, line_stride, pixel_depth);

    ImageHeader header;
    gdal_image_header(_dataset, header);
    //判断是否要更换波段顺序,OPENCV中存储的是BGR的顺序
    if (header.color[0] == IMAGE_RED && header.color[1] == IMAGE_GREEN && header.color[2] == IMAGE_BLUE &&
        dataype == GDT_Byte) {
        if (header.channel == 3)
            cv::cvtColor(patch, patch, cv::COLOR_RGB2BGR);
        else if (header.channel == 4)
            cv::cvtColor(patch, patch, cv::COLOR_RGBA2BGRA);
    }

    return patch;
}

void gdal_write_image(const string &path, cv::Mat &img) {
    MatrixXd xform(2, 3);
    xform << 1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
    gdal_write_image(path, img, xform);
}

void gdal_write_image(const std::string &path, cv::Mat &mat, const Eigen::MatrixXd &xform) {

    GDALAllRegister();

    int channels = mat.channels();
    int width = mat.cols;
    int height = mat.rows;

    int type = mat.depth();

    GDALDriverH driver = NULL;
    string extname = string_to_lower(get_extension_name(path));

    char **options = NULL;
    string driver_name;

    if (extname == ".img") {
        driver_name = "HFA";
        driver = GDALGetDriverByName(driver_name.c_str());
    } else if (extname == ".tif" || extname == ".tiff") {
        driver_name = "GTiff";
        options = CSLSetNameValue(options, "TILED", "YES");
        options = CSLSetNameValue(options, "BLOCKXSIZE", "256");
        options = CSLSetNameValue(options, "BLOCKYSIZE", "256");
        options = CSLSetNameValue(options, "COMPRESS", "LZW");
        options = CSLSetNameValue(options, "TFW", "YES");

        driver = GDALGetDriverByName(driver_name.c_str());
    }

    CHECK(driver != NULL) << "GDAL Driver not created";
    if (file_exist(path)) {
        // 删除
        auto err = GDALDeleteDataset(driver, path.c_str());
        if (err != CE_None) {
            CHECK(delete_file(path)) << "Failed to delete file";
        }
    }

    int pixel_depth;
    GDALDataType data_type;
    if (type == CV_8U) {
        data_type = GDT_Byte;
        pixel_depth = 1;
    } else if (type == CV_16U) {
        data_type = GDT_UInt16;
        pixel_depth = 2;
    } else if (type == CV_16S) {
        data_type = GDT_Int16;
        pixel_depth = 2;
    } else if (type == CV_32S) {
        data_type = GDT_Int32;
        pixel_depth = 4;
    } else if (type == CV_32F) {
        data_type = GDT_Float32;
        pixel_depth = 4;
    } else if (type == CV_64F) {
        data_type = GDT_Float64;
        pixel_depth = 8;
    }

    GDALDatasetH dataset = GDALCreate(driver, path.c_str(), width, height, channels, data_type, options);

    CHECK(dataset != NULL) << "GDAL dataset not created";
    {
        double xtrans[6];
        xtrans[0] = xform(0, 2);
        xtrans[1] = xform(0, 0);
        xtrans[2] = xform(0, 1);
        xtrans[3] = xform(1, 2);
        xtrans[4] = xform(1, 0);
        xtrans[5] = xform(1, 1);
        auto err = GDALSetGeoTransform(dataset, xtrans);
        CHECK(err == CE_None) << "GDAL dataset not created";
    }

    GDALDataset *pdataset = (GDALDataset *)dataset;
    int pixel_stride = pixel_depth * channels;
    for (int i = 0; i < channels; ++i) {
        GDALRasterBandH band = GDALGetRasterBand(dataset, i + 1);

        int pos = i * pixel_depth;
        CPLErr err = GDALRasterIO(band, GF_Write, 0, 0, width, height, mat.data + pos, width, height, data_type,
                                  pixel_stride, pixel_stride * width);
        CHECK(err == CE_None) << "Error in write data into image";
    }

    int lists[10] = {2, 4, 8, 16, 32, 64, 128, 256, 512, 1024};

    if (driver_name == "GTiff") {
        CPLErr err = GDALBuildOverviews(dataset, "NEAREST", 10, lists, 0, NULL, GDALDummyProgress, NULL);
        CHECK(err == CE_None);
    }
    GDALClose(dataset);
    CSLDestroy(options);
}

void gdal_write_image_patch(const std::string &path, const cv::Mat &patch, const Vector2i &offset) {
    CHECK(file_exist(path));

    // boost 得到的文件后缀是带.的，例如".tif"
    string file_ext = get_extension_name(string_to_lower(path));
    CHECK(file_ext == ".tif" || file_ext == ".tiff" || file_ext == ".img")
        << "only support these files for patch write";

    GDALAllRegister();

    int channels = patch.channels();
    int depth = patch.depth(); // CV_8U, CV_16U, CV_16S, CV_32F,CV_64F等
    int pixel_depth;           //像素的大小CV_8U=1, CV_16U=2, CV_32F = 4, CV_64F = 8
    GDALDataType datatype;
    if (depth == CV_8U) {
        datatype = GDT_Byte;
        pixel_depth = 1;
    } else if (depth == CV_16U) {
        datatype = GDT_UInt16;
        pixel_depth = 2;
    } else if (depth == CV_16S) {
        datatype = GDT_Int16;
        pixel_depth = 2;
    } else if (depth == CV_32S) {
        datatype = GDT_Int32;
        pixel_depth = 4;
    } else if (depth == CV_32F) {
        datatype = GDT_Float32;
        pixel_depth = 4;
    } else if (depth == CV_64F) {
        datatype = GDT_Float64;
        pixel_depth = 8;
    }

    int pixel_stride = pixel_depth * patch.channels(); //像素的存储顺序是同一个像素存BGR
    int line_stride = patch.step[0];                   // patch中一行的大小

    GDALDataset *dataset;
    dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_Update);
    CHECK(dataset != NULL);

    // 影像原始的长宽
    ImageHeader header;
    gdal_image_header(dataset, header);
    CHECK(offset.x() >= 0 && offset.x() + patch.cols <= header.width && offset.y() >= 0 &&
          offset.y() + patch.rows <= header.height);

    // 往里面写数据了
    dataset->RasterIO(GF_Write, offset.x(), offset.y(), patch.cols, patch.rows, patch.data, patch.cols, patch.rows,
                      datatype, channels, NULL, pixel_stride, line_stride, pixel_depth);
    GDALClose(dataset);
}

void gdal_image_header(const string &path, ImageHeader &header) {
    GDALAllRegister();
    GDALDataset *dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_ReadOnly);
    CHECK(dataset != NULL) << "dataset open failed";
    gdal_image_header(dataset, header);
    GDALClose(dataset);
}

void gdal_image_header(GDALDatasetH hdataset, ImageHeader &header) {
    GDALDataset *dataset = (GDALDataset *)hdataset;

    CHECK(dataset != NULL) << "dataset open failed";

    int blockx, blocky;
    dataset->GetRasterBand(1)->GetBlockSize(&blockx, &blocky);

    header.channel = dataset->GetRasterCount();
    header.datatype = dataset->GetRasterBand(1)->GetRasterDataType();
    header.width = dataset->GetRasterXSize();
    header.height = dataset->GetRasterYSize();
    header.levels = dataset->GetRasterBand(1)->GetOverviewCount();
    header.pyramid = header.levels > 0;
    header.blockx = blockx;
    header.blocky = blocky;

    for (int i = 0; i < header.channel; ++i) {
        if (i < 4) {
            header.color[i] = dataset->GetRasterBand(i + 1)->GetColorInterpretation();
        }
    }
    for (int i = 0; i < header.levels; ++i) {
        int bandx, bandy;
        bandx = dataset->GetRasterBand(1)->GetOverview(i)->GetXSize();
        bandy = dataset->GetRasterBand(1)->GetOverview(i)->GetYSize();
        header.pyr_width.push_back(bandx);
        header.pyr_height.push_back(bandy);
    }

    //获取辐射分辨率
    {
        {
            //先根据TIFFTAG_MAXSAMPLEVALUE来判断，如果这个存在，就用这个值
            const char *tmp = dataset->GetMetadataItem("TIFFTAG_MAXSAMPLEVALUE");
            if (tmp != nullptr) {
                string s = tmp;
                if (!s.empty())
                    header.maxvalue = std::stoi(s);
            } else if (header.datatype == GDT_Byte) {
                header.maxvalue = 255;
            } else if (header.datatype == GDT_UInt16) {
                header.maxvalue = 65535;
            } else {
                header.maxvalue = -1;
            }
        }
    }
}

void gdal_write_dem(const string &path, const cv::Mat &mat, const Vector2d &origin, const Vector2d &spaces) {
    int width, height;
    int channels;
    GDALDatasetH dataset;
    GDALAllRegister();
    channels = mat.channels();
    width = mat.cols;
    height = mat.rows;

    int type = mat.depth();
    GDALDriverH driver = NULL;
    string extname = string_to_lower(get_extension_name(path));

    char **options = NULL;
    string driver_name;

    if (extname == ".img") {
        driver_name = "HFA";
        driver = GDALGetDriverByName(driver_name.c_str());
    } else if (extname == ".tif" || extname == ".tiff") {
        driver_name = "GTiff";
        options = CSLSetNameValue(options, "TILED", "YES");
        options = CSLSetNameValue(options, "BLOCKXSIZE", "256");
        options = CSLSetNameValue(options, "BLOCKYSIZE", "256");
        options = CSLSetNameValue(options, "COMPRESS", "LZW");
        options = CSLSetNameValue(options, "TFW", "YES");

        driver = GDALGetDriverByName(driver_name.c_str());
    }

    CHECK(driver != NULL) << "GDAL Driver not created";
    if (file_exist(path)) {
        GDALDeleteDataset(driver, path.c_str());
    }

    int pixel_depth;
    GDALDataType data_type;
    if (type == CV_8U) {
        data_type = GDT_Byte;
        pixel_depth = 1;
    } else if (type == CV_16U) {
        data_type = GDT_UInt16;
        pixel_depth = 2;
    } else if (type == CV_16S) {
        data_type = GDT_Int16;
        pixel_depth = 2;
    } else if (type == CV_32S) {
        data_type = GDT_Int32;
        pixel_depth = 4;
    } else if (type == CV_32F) {
        data_type = GDT_Float32;
        pixel_depth = 4;
    } else if (type == CV_64F) {
        data_type = GDT_Float64;
        pixel_depth = 8;
    }

    if (file_exist(path)) {
        // 删除
        auto err = GDALDeleteDataset(driver, path.c_str());
        CHECK(err == CE_None) << "failed to delete dataset " << path;
    }

    dataset = GDALCreate(driver, path.c_str(), width, height, channels, data_type, options);

    {
        // geotransformation
        double xtrans[6] = {origin.x() - spaces.x() / 2.0, // 左上角像素的左上角
                            spaces.x(),                    // x 分辨率
                            0.0,
                            origin.y() + spaces.y() * (height - 1 + 0.5), // 左上角像素的左上角
                            0.0,
                            -spaces.y()};
        auto err = GDALSetGeoTransform(dataset, xtrans);
        CHECK(err == CE_None) << "GDAL set geotransformation error";
    }

    CHECK(dataset != NULL) << "GDAL dataset not created";

    int pixel_stride = pixel_depth * channels;
    for (int i = 0; i < channels; ++i) {
        GDALRasterBandH band = GDALGetRasterBand(dataset, i + 1);

        int pos = i * pixel_depth;
        CPLErr err = GDALRasterIO(band, GF_Write, 0, 0, width, height, mat.data + pos, width, height, data_type,
                                  pixel_stride, pixel_stride * width);
        CHECK(err == CE_None) << "Error in write data into image";
    }

    int lists[3] = {2, 4, 8};

    if (driver_name == "GTiff") {
        CHECK(GDALBuildOverviews(dataset, "NEAREST", 3, lists, 0, NULL, GDALDummyProgress, NULL) == CE_None);
    }
    GDALClose(dataset);
    CSLDestroy(options);
}

double gdal_get_nodata(const string &path) {
    GDALDataset *dataset = NULL;
    GDALAllRegister();
    dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_Update);
    CHECK(dataset != NULL) << "can not open and update dataset" << path.c_str();
    double nodata = gdal_get_nodata(dataset);
    GDALClose(dataset);
    return nodata;
}

double gdal_get_nodata(GDALDatasetH _dataset) {
    GDALDataset *dataset = (GDALDataset *)_dataset;
    int num_band = dataset->GetRasterCount();
    int success = FALSE;
    double nodata = dataset->GetRasterBand(1)->GetNoDataValue(&success);
    if (success) {
        return nodata;
    } else {
        return DBL_MAX;
    }
}

void gdal_set_nodata(const string &path, double nodata) {
    GDALDataset *dataset = NULL;
    GDALAllRegister();
    dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_Update);
    CHECK(dataset != NULL) << "can not open and update dataset" << path.c_str();
    gdal_set_nodata(dataset, nodata);
    GDALClose(dataset);
}

void gdal_set_nodata(GDALDatasetH _dataset, double nodata) {
    GDALDataset *dataset = (GDALDataset *)_dataset;
    int num_band = dataset->GetRasterCount();

    for (int i = 0; i < num_band; ++i) {
        auto res = dataset->GetRasterBand(i + 1)->SetNoDataValue(nodata);
    }
}

void gdal_build_overviews(const std::string &path) {
    GDALDatasetH dataset = GDALOpen(path.c_str(), GA_Update);
    if (dataset == nullptr) {
        return;
    }
    gdal_build_overviews(dataset);
    GDALClose(dataset);
    return;
}

void gdal_build_overviews(GDALDatasetH dataset) {
    ImageHeader header;
    gdal_image_header(dataset, header);

    int min_size = std::min(header.width, header.height);
    int power = std::log2(min_size);

    std::vector<int> levels(power - 1);
    for (int i = 1; i < power; ++i) {
        levels[i - 1] = std::pow(2, i);
    }

    GDALBuildOverviews(dataset, "NEAREST", power - 1, levels.data(), 0, NULL, GDALDummyProgress, NULL);
}

void gdal_compute_statistics(const std::string &path) {
    GDALAllRegister();
    GDALDatasetH dataset = GDALOpen(path.c_str(), GA_Update);
    if (dataset == nullptr) {
        return;
    }
    gdal_compute_statistics(dataset);
    GDALClose(dataset);
    return;
}

void gdal_compute_statistics(GDALDatasetH _dataset) {
    GDALDataset *dataset = (GDALDataset *)_dataset;
    int num_bands = dataset->GetRasterCount();
    for (int i = 0; i < num_bands; ++i) {
        GDALRasterBand *band = dataset->GetRasterBand(i + 1);
        double minv, maxv, avgv, stdv;
        band->ComputeStatistics(FALSE, &minv, &maxv, &avgv, &stdv, nullptr, nullptr);
        band->SetStatistics(minv, maxv, avgv, stdv);
    }
}

VectorXd gdal_get_nodata_multi(const string &path) {
    GDALAllRegister();
    GDALDataset *dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_ReadOnly);

    CHECK(dataset != NULL) << "dataset can not be accessed";
    int channels = dataset->GetRasterCount();

    VectorXd nodata(channels);
    for (int i = 0; i < channels; ++i) {
        nodata(i) = dataset->GetRasterBand(i + 1)->GetNoDataValue();
    }
    GDALClose(dataset);
    return nodata;
}

MatrixXd gdal_get_transform(const string &path) {
    GDALAllRegister();
    GDALDataset *dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_ReadOnly);

    MatrixXd xform = gdal_get_transform(dataset);
    GDALClose(dataset);
    return xform;
}
MatrixXd gdal_get_transform(GDALDatasetH dataset) {
    CHECK(dataset != NULL) << "dataset read failed";

    double xform_ptr[6];
    ((GDALDataset *)dataset)->GetGeoTransform(xform_ptr);

    MatrixXd xform(2, 3);

    xform(0, 0) = xform_ptr[1];
    xform(0, 1) = xform_ptr[2];
    xform(0, 2) = xform_ptr[0];
    xform(1, 0) = xform_ptr[4];
    xform(1, 1) = xform_ptr[5];
    xform(1, 2) = xform_ptr[3];

    return xform;
}
void gdal_set_transform(const string &path, const MatrixXd &xform) {
    GDALAllRegister();
    GDALDataset *dataset = (GDALDataset *)GDALOpen(path.c_str(), GA_Update);

    gdal_set_transform(dataset, xform);
    GDALClose(dataset);
}
void gdal_set_transform(GDALDatasetH dataset, const MatrixXd &xform) {
    CHECK(dataset != NULL) << "dataset read failed";

    double xform_ptr[6];
    xform_ptr[1] = xform(0, 0);
    xform_ptr[2] = xform(0, 1);
    xform_ptr[0] = xform(0, 2);
    xform_ptr[4] = xform(1, 0);
    xform_ptr[5] = xform(1, 1);
    xform_ptr[3] = xform(1, 2);

    auto res = ((GDALDataset *)dataset)->SetGeoTransform(xform_ptr);
    CHECK(res == CE_None) << "error in set geotransform";
}

void gdal_set_projection(const std::string &path, const std::string &esri_string) {
    GDALAllRegister();
    GDALDatasetH dataset = GDALOpen(path.c_str(), GA_Update);
    CHECK(dataset != NULL);
    gdal_set_projection(dataset, esri_string);
    GDALClose(dataset);
}

void gdal_set_projection(GDALDatasetH handle, const std::string &esri_string) {
    GDALAllRegister();
    OGRSpatialReference projection;
    char *esri[] = {(char *)esri_string.c_str(), NULL};
    projection.importFromESRI(esri);

    char *proj4;
    projection.exportToWkt(&proj4);
    GDALDataset *dataset = (GDALDataset *)handle;
    dataset->SetProjection(proj4);
}

std::string gdal_get_projection_wkt(const std::string &path) {
    GDALAllRegister();
    GDALDatasetH dataset = GDALOpen(path.c_str(), GA_ReadOnly);
    CHECK(dataset != NULL);
    std::string wkt = gdal_get_projection_wkt(dataset);
    GDALClose(dataset);

    return wkt;
}

std::string gdal_get_projection_wkt(GDALDatasetH dataset) {
    const char *wkt = ((GDALDataset *)dataset)->GetProjectionRef();
    return std::string(wkt);
}

void gdal_set_projection_wkt(const std::string &path, const std::string &wkt) {
    if (wkt.empty()) {
        return;
    }
    GDALAllRegister();
    GDALDatasetH dataset = GDALOpen(path.c_str(), GA_Update);
    CHECK(dataset != NULL);
    gdal_set_projection_wkt(dataset, wkt);
    GDALClose(dataset);
    return;
}

void gdal_set_projection_wkt(GDALDatasetH dataset, const std::string &wkt) {
    if (wkt.empty()) {
        return;
    }
    ((GDALDataset *)dataset)->SetProjection(wkt.c_str());
    return;
}

void gdal_write_polygon(const string &path, const GdalPolygon2dVec &polygons) {
    GDALAllRegister();
    string driver_name = "ESRI Shapefile";

    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName(driver_name.c_str());
    CHECK(driver != NULL) << "ESRI Driver is not available";

    if (file_exist(path)) {
        GDALDeleteDataset(driver, path.c_str());
    }

    GDALDataset *dataset = driver->Create(path.c_str(), 0, 0, 0, GDT_Unknown, NULL);
    CHECK(dataset != NULL) << "dataset can not be created at: " << path;

    auto layer = dataset->CreateLayer("polygon", NULL, wkbPolygon, NULL);
    CHECK(layer != NULL) << "layer can not be created";

    for (int i = 0; i < polygons.size(); ++i) {
        auto *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        CHECK(feature != NULL) << "create feature failed for polygon #" << i;
        OGRPolygon ogr_polygon;
        OGRLinearRing ogr_ring;
        ogr_ring.setNumPoints(polygons[i].size());

        for (int j = 0; j < polygons[i].size(); ++j) {
            ogr_ring.setPoint(j, polygons[i][j].x(), polygons[i][j].y());
        }
        ogr_ring.closeRings();
        ogr_polygon.addRing(&ogr_ring);

        CHECK(feature->SetGeometry(&ogr_polygon) == OGRERR_NONE) << "Create polygon failed";
        CHECK(layer->CreateFeature(feature) == OGRERR_NONE) << "create feature failed";
        OGRFeature::DestroyFeature(feature);
    }
    GDALClose(dataset);
}
GdalPolygon2dVec gdal_read_polygon(const string &path) {
    GDALAllRegister();
    GdalPolygon2dVec polygons;

    GDALDataset *dataset = (GDALDataset *)GDALOpenEx(path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    CHECK(dataset != NULL) << "dataset can not be read at: " << path;
    int numlayer = dataset->GetLayerCount();
    CHECK(numlayer == 1) << "only support single layer shapefile and file has #" << numlayer << " layers";
    auto *layer = dataset->GetLayer(0);
    layer->ResetReading();
    OGRFeature *feature;
    while ((feature = layer->GetNextFeature())) {

        OGRPolygon *ogr_polygon = (OGRPolygon *)feature->GetGeometryRef();
        OGRLinearRing *org_ring = ogr_polygon->getExteriorRing();
        int npoints = org_ring->getNumPoints();
        CHECK(npoints > 0) << "geometry is invalid with #" << npoints << " points";

        GdalPolygon2d polygon(npoints);
        for (int i = 0; i < npoints; ++i) {
            OGRPoint point;
            org_ring->getPoint(i, &point);
            polygon[i] = {point.getX(), point.getY()};
        }
        OGRFeature::DestroyFeature(feature);
        polygons.push_back(polygon);
    }
    GDALClose(dataset);
    return polygons;
}

vector<Vector3d> gdal_read_points(const string &path) {
    GDALAllRegister();
    CHECK(string_to_lower(get_extension_name(path)) == ".txt" || string_to_lower(get_extension_name(path)) == ".csv" ||
          string_to_lower(get_extension_name(path)) == ".dxf" || string_to_lower(get_extension_name(path)) == ".dwg" ||
          string_to_lower(get_extension_name(path)) == ".shp")
        << "only support read these files";

    GDALDataset *dataset = (GDALDataset *)GDALOpenEx(path.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    int num_layer = dataset->GetLayerCount();
    CHECK(num_layer == 1) << "only support single layer file";
    auto *layer = dataset->GetLayer(0);
    layer->ResetReading();
    OGRFeature *feature;
    vector<Vector3d> points3d;
    while ((feature = layer->GetNextFeature())) {
        OGRPoint *ogr_point = (OGRPoint *)feature->GetGeometryRef();
        CHECK(ogr_point->Is3D()) << "this should read 3d points";
        points3d.emplace_back(ogr_point->getX(), ogr_point->getY(), ogr_point->getZ());
        OGRFeature::DestroyFeature(feature);
    }
    GDALClose(dataset);
    return points3d;
}
void gdal_write_points(const string &path, const vector<Vector3d> &points) {
    GDALAllRegister();
    CHECK(string_to_lower(get_extension_name(path)) == ".txt" || string_to_lower(get_extension_name(path)) == ".csv" ||
          string_to_lower(get_extension_name(path)) == ".dxf" || string_to_lower(get_extension_name(path)) == ".dwg" ||
          string_to_lower(get_extension_name(path)) == ".shp")
        << "only support read these files";

    string driver_name = "ESRI Shapefile";

    if (string_to_lower(get_extension_name(path)) == ".txt") {
        driver_name = "CSV";
    } else if (string_to_lower(get_extension_name(path)) == ".csv") {
        driver_name = "CSV";
    } else if (string_to_lower(get_extension_name(path)) == ".shp") {
        driver_name = "ESRI Shapefile";
    } else if (string_to_lower(get_extension_name(path)) == ".dxf") {
        driver_name = "CAD";
    } else if (string_to_lower(get_extension_name(path)) == ".dwg") {
        driver_name = "DXF";
    }

    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName(driver_name.c_str());
    CHECK(driver != NULL) << "vector driver is not available for " << driver_name;

    if (file_exist(path)) {
        GDALDeleteDataset(driver, path.c_str());
    }

    GDALDataset *dataset = driver->Create(path.c_str(), 0, 0, 0, GDT_Unknown, NULL);
    CHECK(dataset != NULL) << "dataset can not be created at: " << path;

    auto layer = dataset->CreateLayer("points", NULL, wkbPoint, NULL);
    CHECK(layer != NULL) << "layer can not be created";

    for (int i = 0; i < points.size(); ++i) {
        auto *feature = OGRFeature::CreateFeature(layer->GetLayerDefn());
        CHECK(feature != NULL) << "create feature failed for polygon #" << i;
        OGRPoint ogr_point;
        ogr_point.set3D(TRUE);
        ogr_point.setX(points[i].x());
        ogr_point.setY(points[i].y());
        ogr_point.setZ(points[i].z());
        CHECK(feature->SetGeometry(&ogr_point) == OGRERR_NONE) << "Create polygon failed";
        CHECK(layer->CreateFeature(feature) == OGRERR_NONE) << "create feature failed";
        OGRFeature::DestroyFeature(feature);
    }
    GDALClose(dataset);
}

int gdal_get_exif_orientation(const std::string &path) {
    GDALAllRegister();
    GDALDatasetH dataset = GDALOpen(path.c_str(), GA_ReadOnly);
    if (dataset == NULL) {
        GDALClose(dataset);
        return -1;
    }
    int o = gdal_get_exif_orientation(dataset);
    GDALClose(dataset);
    return o;
}

int gdal_get_exif_orientation(GDALDatasetH _dataset) {
    GDALDataset *dataset = (GDALDataset *)_dataset;
    auto exif_orientation = dataset->GetMetadataItem("EXIF_Orientation");
    if (exif_orientation != NULL) {
        return std::stoi(std::string(exif_orientation));
    } else {
        // TODO gdal貌似不能读TIFFTAG，如果没有上面这个值，默认返回1
        return 1;
    }
}

} // namespace h2o

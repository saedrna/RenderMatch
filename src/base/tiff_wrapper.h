/*
 * @Author: Han
 * @Date: 2018-05-10 19:19:30
 * 封装 tiff 函数
 */
#pragma once
#include <base/common.h>

namespace h2o {
enum TiffCompression { TiffCompressionLZW = 5, TiffCompressionJPEG = 7 };
std::vector<uint8_t> tiff_encode(const cv::Mat &mat, TiffCompression compression = TiffCompressionJPEG);
std::vector<uint8_t> tiff_encode_pyramid(const cv::Mat &mat, TiffCompression compression = TiffCompressionJPEG);

} // namespace h2o

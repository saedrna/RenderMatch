/*
 * @Author: Han
 * @Date: 2019-11-15 19:15:05
 * Extract SIFT features
 */

#ifdef _MSC_VER
extern "C" {
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
__declspec(dllexport) int NvOptimusEnablement = 1;
}
#endif

#include <ProgressBar.hpp>
#include <cxxopts.hpp>

#include <RenderMatch/block.h>
#include <base/base.h>
#include <match/sift_detector.h>

#include <opencv2/imgcodecs.hpp>

#include <QFileInfo>

int main(int argc, char **argv) {
    cxxopts::Options options("FeatureExtract", "Extract SIFT features");

    std::string path_at;

    // clang-format off
    options.add_options("FeatureExtract")
        ("i,input", "Input AT files in json format", cxxopts::value(path_at))
        ("h,help", "Print this help message");
    // clang-format on

    auto results = options.parse(argc, argv);
    if (results.arguments().empty() || results.count("help")) {
        std::cout << options.help({"FeatureExtract"}) << std::endl;
        return 1;
    }

    path_at = QFileInfo(QString::fromLocal8Bit(path_at.c_str())).absoluteFilePath().toStdString();

    using namespace h2o;
    h2o::Block block = h2o::load_block(path_at);

    SiftDetector sift;

    ProgressBar bar(block.photos.size(), 80);

    // GPU-SIFT must work in main thread
    for (const auto &pair : block.photos) {
        auto photo = pair.second;
        std::string path = photo.path;
        std::string dir = get_directory(path);
        std::string name = get_filename_noext(path);

        std::vector<cv::KeyPoint> keys;
        cv::Mat desc;

        {
            cv::Mat mat = cv::imread(path, cv::IMREAD_GRAYSCALE);
            mat = image_percent_scale_8u(mat);
            std::tie(keys, desc) = sift.detect_and_compute(mat);
            std::string path_sift = join_paths(dir, name + ".sift");
            sift_save(path_sift, keys, desc);
        }
        ++bar;
        bar.display();
    }

    bar.done();

    return 0;
}
/*
 * @Author: Han
 * @Date: 2019-10-21 19:25:02
 * Convert original images into tiled TIFF for faster reading
 */

#include <ProgressBar.hpp>
#include <cxxopts.hpp>

#include <RenderMatch/block.h>
#include <base/base.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <QFileInfo>
using namespace h2o;

namespace h2o {
class TileImageWorker {
public:
    TileImageWorker() {}
    ~TileImageWorker() {}

public:
    void run();

protected:
    cv::Mat process_image(const Photo &camera) const;
    // 统计所有影像的平均大小，然后假设一个线程处理得需要四倍影像大小，计算线程数目
    int compute_num_threds();

public:
    // 影像输出路径
    std::string folder_;

    Block block_;
    Block block_tiled_;
};

} // namespace h2o

int main(int argc, char **argv) {
    cxxopts::Options options("TileImage", "Convert original images into tiled TIFF for faster reading");

    std::string path_at;
    std::string path_outdir;

    // clang-format off
    options.add_options("TileImage")
        ("i,input", "Input AT file in BlockExchange Format", cxxopts::value(path_at))
        ("o,output", "Output directory containing the tiled images and reformated AT file", cxxopts::value(path_outdir))
        ("h,help", "Print help message");
    // clang-format on

    auto results = options.parse(argc, argv);
    if (results.arguments().empty() || results.count("help")) {
        std::cout << options.help({"TileImage"}) << std::endl;
        return 1;
    }

    path_at = QFileInfo(QString::fromLocal8Bit(path_at.c_str())).absoluteFilePath().toStdString();
    path_outdir = QFileInfo(QString::fromLocal8Bit(path_outdir.c_str())).absoluteFilePath().toStdString();

    std::string name = get_filename_noext(path_at);
    name = join_paths(path_outdir, name + ".json");

    h2o::Block block = load_block_xml(path_at);

    TileImageWorker tiler;
    tiler.folder_ = path_outdir;
    tiler.block_ = block;
    tiler.run();

    save_block(name, tiler.block_tiled_);

    return 0;
}

namespace h2o {
void TileImageWorker::run() {
    create_dir_if_not_exist(folder_);

    std::vector<uint32_t> iids;

    // Create a new block without distortion
    block_tiled_ = block_;

    for (auto &photo : block_tiled_.photos) {
        std::string path_tile = join_paths(folder_, "images");
        std::string path = photo.second.path;
        std::string name = get_filename_noext(path);
        path_tile = join_paths(folder_, name + ".tif");
        photo.second.path = path_tile;

        iids.push_back(photo.second.id);
    }

    ProgressBar bar(block_.photos.size(), 80);

    int np = compute_num_threds();

#pragma omp parallel for schedule(dynamic) num_threads(np)
    for (int i = 0; i < iids.size(); ++i) {
        uint32_t iid = iids[i];
        std::string path = block_tiled_.photos.at(iid).path;

        if (file_exist(path)) {
#pragma omp critical
            {
                ++bar;
                bar.display();
            }
            continue;
        }

        cv::Mat mat = process_image(block_.photos.at(iid));
        std::vector<uint8_t> buf = tiff_encode(mat);
        mat.release();

#pragma omp critical
        {
            QFile file(QString::fromStdString(path));
            file.open(QFile::WriteOnly);
            file.write((const char *)buf.data(), buf.size());

            ++bar;
            bar.display();
        }
    }
}

cv::Mat TileImageWorker::process_image(const Photo &photo) const {
    // 判断 frustum 和 bbox 的相交关系
    cv::Mat image;
    int orientation;
    std::vector<char> buffer;
#pragma omp critical
    {
        QFile file(QString::fromStdString(photo.path));
        file.open(QFile::ReadOnly);
        buffer.resize(file.size());
        file.read((char *)buffer.data(), buffer.size());
        orientation = gdal_get_exif_orientation(photo.path);
    }

    image = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
    buffer.clear();

    ///\s.a http://sylvana.net/jpegcrop/exif_orientation.html
    switch (orientation) {
    case 1:
        break;
    case 2:
        cv::flip(image, image, 1); // horizontal
        break;
    case 3:
        cv::flip(image, image, -1); // both
        break;
    case 4:
        cv::flip(image, image, 0); // vertical
        break;
    case 5:
        image = image.t(); // transpose
        break;
    case 6:
        cv::flip(image.t(), image, 0); // transpose -> vertical
        break;
    case 7:
        cv::flip(image.t(), image, -1); // transpose -> both
        break;
    case 8:
        cv::flip(image.t(), image, 1); // transpose -> horizontal
        break;
    default:
        LOG(WARNING) << "unknown orientation";
        break;
    }

    return image;
}

int TileImageWorker::compute_num_threds() {
    double rows = 0.0, cols = 0.0;
    for (const auto &pair : block_.photos) {
        uint32_t cid = pair.second.cid;
        int row = block_.groups.at(cid).height;
        int col = block_.groups.at(cid).width;

        rows += row;
        cols += col;
    }

    rows /= block_.photos.size();
    cols /= block_.photos.size();

    // Assume that the processed image should have 3 times of the image size
    double mb_per_threads = rows * cols * 3 * 3.0 / 1024.0 / 1024.0;
    double mb_remaining = get_system_memory();

    int num_threads = mb_remaining / mb_per_threads;
    int num_max_threads = Eigen::nbThreads();
    num_threads = std::min(num_threads, num_max_threads);
    return std::max(num_threads, 1);
}
} // namespace h2o

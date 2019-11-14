/*
 * @Author: Han
 * @Date: 2019-10-21 19:03:42
 * Undistort images in a blockexchange format AT file and export a new AT file
 */

#include <ProgressBar.hpp>
#include <cxxopts.hpp>

#include <RenderMatch/block.h>
#include <base/base.h>

#include <opencv2/imgcodecs.hpp>

namespace h2o {
class UndistortImageWorker {

public:
    void run();

protected:
    // rectify image
    cv::Mat process_image(const Photo &photo);

    // compute number of threads for parallel compute, under the limit of memory
    int compute_num_threads();

    void init_undistort_maps();

    void init_rectified_block();

    void rectify_images();

protected:
    // undistort maps for each camera
    std::map<uint32_t, cv::Mat> mapx_;
    std::map<uint32_t, cv::Mat> mapy_;

public:
    std::string folder_;

    Block block_;
    Block block_rectified_;
};
} // namespace h2o

using namespace h2o;

int main(int argc, char **argv) {
    cxxopts::Options options("UndistortImage",
                             "Convert original images to undistorted images for better match with rendered images");

    std::string path_at;
    std::string path_outdir;

    // clang-format off
    options.add_options("UndistortImage")
        ("i,input", "Input AT file in BlockExchange Format", cxxopts::value(path_at))
        ("o,output", "Output directory containing the undistorted images and reformated AT file", cxxopts::value(path_outdir))
        ("h,help", "Print help message");
    // clang-format on

    auto results = options.parse(argc, argv);
    if (results.arguments().empty() || results.count("help")) {
        std::cout << options.help({"UndistortImage"}) << std::endl;
        return 1;
    }

    path_at = QFileInfo(QString::fromLocal8Bit(path_at.c_str())).absoluteFilePath().toStdString();
    path_outdir = QFileInfo(QString::fromLocal8Bit(path_outdir.c_str())).absoluteFilePath().toStdString();

    std::string name = get_filename_noext(path_at);
    name = join_paths(path_outdir, name + ".json");

    h2o::Block block = load_block_xml(path_at);

    UndistortImageWorker worker;
    worker.folder_ = path_outdir;
    worker.block_ = block;
    worker.run();

    save_block(name, worker.block_rectified_);
}

namespace h2o {
void UndistortImageWorker::run() {
    create_dir_if_not_exist(folder_);

    init_rectified_block();
    init_undistort_maps();
    rectify_images();
}

cv::Mat UndistortImageWorker::process_image(const Photo &photo) {
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

    const cv::Mat &mapx = mapx_.at(photo.cid);
    const cv::Mat &mapy = mapy_.at(photo.cid);

    cv::remap(image, image, mapx, mapy, cv::INTER_LINEAR);
    return image;
}

int UndistortImageWorker::compute_num_threads() {
    double rows = 0.0, cols = 0.0;
    for (const auto &pair : block_.groups) {

        int row = pair.second.height;
        int col = pair.second.width;
        rows += row;
        cols += col;
    }

    rows /= block_.groups.size();
    cols /= block_.groups.size();

    double mb_per_threads = rows * cols * 3 * 3.0 / 1024.0 / 1024.0;
    double mb_remaining = get_system_memory();

    int num_threads = mb_remaining / mb_per_threads;
    int num_max_threads = Eigen::nbThreads();
    num_threads = std::min(num_threads, num_max_threads);
    return std::max(num_threads, 1);
}

void UndistortImageWorker::init_undistort_maps() {
    std::vector<uint32_t> cids;

    for (const auto &pair : block_.groups) {
        cids.push_back(pair.first);
    }

    std::cout << "Initialize distortion maps" << std::endl;
    ProgressBar bar(cids.size(), 80);

#pragma omp parallel for
    for (int i = 0; i < cids.size(); ++i) {
        uint32_t cid = cids[i];
        auto group = block_.groups.at(cid);

        FrameCamera camera = to_framecamera(group);
        cv::Mat mapx, mapy;
        std::tie(mapx, mapy) = get_distortion_mappings(camera);

#pragma omp critical
        {
            mapx_.emplace(cid, mapx);
            mapy_.emplace(cid, mapy);
            ++bar;
            bar.display();
        }
    }
    bar.done();
}

void UndistortImageWorker::init_rectified_block() {
    block_rectified_ = block_;
    for (auto &pair : block_rectified_.groups) {
        auto &camera = pair.second;
        camera.u0 = (camera.width - 1.0) / 2.0;
        camera.v0 = (camera.height - 1.0) / 2.0;
        camera.K(0, 2) = camera.u0;
        camera.K(1, 2) = camera.v0;
        camera.k1 = 0.0;
        camera.k2 = 0.0;
        camera.k3 = 0.0;
        camera.p1 = 0.0;
        camera.p2 = 0.0;
    }
    for (auto &pair : block_rectified_.photos) {
        auto &photo = pair.second;
        std::string name = get_filename_noext(photo.path);
        name = join_paths(folder_, name + ".jpg");
        photo.path = name;
    }
}
void UndistortImageWorker::rectify_images() {
    std::vector<uint32_t> iids;
    for (const auto &pair : block_.photos) {
        iids.push_back(pair.first);
    }

    int np = compute_num_threads();

    std::cout << "Undistort images" << std::endl;
    ProgressBar bar(iids.size(), 80);

#pragma omp parallel for schedule(dynamic) num_threads(np)
    for (int i = 0; i < iids.size(); ++i) {
        uint32_t iid = iids[i];
        Photo photo = block_.photos.at(iid);
        std::string path_undistort = block_rectified_.photos.at(iid).path;

        if (file_exist(path_undistort)) {
#pragma omp critical
            {
                ++bar;
                bar.display();
            }
            continue;
        }

        cv::Mat mat_undistort = process_image(photo);
        std::vector<uint8_t> buf;
        cv::imencode(".jpg", mat_undistort, buf);
        mat_undistort.release();

#pragma omp critical
        {
            QFile file(QString::fromStdString(path_undistort));
            file.open(QFile::WriteOnly);
            file.write((const char *)buf.data(), buf.size());

            ++bar;
            bar.display();
        }
    }
    bar.done();
}
} // namespace h2o

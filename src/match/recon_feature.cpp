/*!
 * \file recon_feature.cpp
 *
 * \author Han
 * \date 2017/05/09
 *
 * 特征点信息
 */
#include <fstream>

#include <match/recon_feature.h>
#include <base/base.h>

namespace h2o {

/************************************************************************/
/*                   FeaturePointsContainer                             */
/************************************************************************/
FeaturePoints FeaturePointsContainer::read_points(const std::string &path) {
    std::ifstream ifile(string_utf8_to_fstream(path), std::ios::binary);
    int npoints, desc;
    ifile.read((char *)&npoints, sizeof(int));
    ifile.read((char *)&desc, sizeof(int));
    CHECK(npoints > 0);
    FeaturePoints points(npoints);
    int size_points = npoints * sizeof(FeaturePoint);
    ifile.read((char *)points.data(), size_points);
    return points;
}

void FeaturePointsContainer::save_points(const std::string &path, const FeaturePoints &points) {
    std::ofstream ofile(string_utf8_to_fstream(path), std::ios::binary);
    int npoints = points.size();
    int ndesc = 0;

    ofile.write((char *)&npoints, sizeof(int));
    ofile.write((char *)&ndesc, sizeof(int));

    int size_points = npoints * sizeof(FeaturePoint);
    ofile.write((char *)points.data(), size_points);
}

std::tuple<FeaturePoints, cv::Mat> FeaturePointsContainer::read_features(const std::string &path) {
    cv::Mat desc;
    std::ifstream ifile(string_utf8_to_fstream(path), std::ios::binary);
    int npoints, ndesc;
    ifile.read((char *)&npoints, sizeof(int));
    ifile.read((char *)&ndesc, sizeof(int));
    CHECK(npoints > 0 && ndesc > 0 && npoints == ndesc);

    FeaturePoints points(npoints);
    int size_points = npoints * sizeof(FeaturePoint);
    ifile.read((char *)points.data(), size_points);

    desc = cv::Mat::zeros(ndesc, 128, CV_8U);
    int size_desc = ndesc * 128;
    ifile.read((char *)desc.data, size_desc);

    return std::make_tuple(points, desc);
}

void FeaturePointsContainer::save_features(const std::string &path, const FeaturePoints &points, const cv::Mat &desc) {

    std::ofstream ofile(string_utf8_to_fstream(path), std::ios::binary);
    int npoints = points.size();
    int ndesc = desc.rows;

    ofile.write((char *)&npoints, sizeof(int));
    ofile.write((char *)&ndesc, sizeof(int));

    int size_points = npoints * sizeof(FeaturePoint);
    ofile.write((char *)points.data(), size_points);

    int size_desc = ndesc * 128;
    ofile.write((char *)desc.data, size_desc);
}

/************************************************************************/
/*               create_feature_container_all_in_memory                 */
/************************************************************************/
class FeaturePointsContainerAllInMemory : public FeaturePointsContainer {

public:
    FeaturePointsContainerAllInMemory() {}
    ~FeaturePointsContainerAllInMemory() {}

    void set_points(std::shared_ptr<FeaturePointsImageMap> _points) { points = _points; }

    virtual void load(const std::unordered_map<iid_t, std::string> &paths) override;
    virtual FeaturePoints get_points(iid_t iid) override { return points->at(iid); }
    virtual std::vector<iid_t> get_image_indices() override;

private:
    std::shared_ptr<FeaturePointsImageMap> points;
};

void FeaturePointsContainerAllInMemory::load(const std::unordered_map<iid_t, std::string> &paths) {
    points = std::make_shared<FeaturePointsImageMap>();
    for (auto pair : paths) {
        points->insert({pair.first, read_points(pair.second)});
    }
}

std::vector<iid_t> FeaturePointsContainerAllInMemory::get_image_indices() {
    std::vector<iid_t> image_indices;
    for (const auto &pair : *points) {
        image_indices.push_back(pair.first);
    }
    std::sort(begin(image_indices), end(image_indices));
    return image_indices;
}

FeaturePoints features_from_keypoints(const std::vector<cv::KeyPoint> &keys) {
    FeaturePoints features(keys.size());
    for (int i = 0; i < features.size(); ++i) {
        features[i] = {keys[i].pt.x, keys[i].pt.y};
    }
    return features;
}

std::shared_ptr<FeaturePointsContainer> create_feature_container_all_in_memory(const FeaturePointsPath &paths) {
    std::shared_ptr<FeaturePointsContainer> feature = std::make_shared<FeaturePointsContainerAllInMemory>();
    feature->load(paths);
    return feature;
}

std::shared_ptr<FeaturePointsContainer>
create_feature_container_all_in_memory(const std::shared_ptr<FeaturePointsImageMap> &points) {
    std::shared_ptr<FeaturePointsContainer> feature = std::make_shared<FeaturePointsContainerAllInMemory>();
    ((FeaturePointsContainerAllInMemory *)feature.get())->set_points(points);
    return feature;
}

/************************************************************************/
/*                create_feature_container_lru_catch                    */
/************************************************************************/
class FeaturePointsContainerLru : public FeaturePointsContainer {
public:
    FeaturePointsContainerLru(int ncache) : cache(ncache) {}
    ~FeaturePointsContainerLru() {}

    void load(const FeaturePointsPath &paths) override;
    FeaturePoints get_points(iid_t iid) override;
    std::vector<iid_t> get_image_indices() override;

private:
    Cache<iid_t, FeaturePoints, std::mutex> cache;
    FeaturePointsPath paths;
};

void FeaturePointsContainerLru::load(const FeaturePointsPath &_paths) { paths = _paths; }

FeaturePoints FeaturePointsContainerLru::get_points(iid_t iid) {
    if (cache.contains(iid)) {
        return cache.get(iid);
    } else {
        CHECK(paths.count(iid) > 0);
        FeaturePoints points = read_points(paths.at(iid));
        cache.insert(iid, points);
        return points;
    }
    return FeaturePoints();
}

std::vector<iid_t> FeaturePointsContainerLru::get_image_indices() {
    std::vector<iid_t> image_indices;
    for (const auto &pair : paths) {
        image_indices.push_back(pair.first);
    }
    std::sort(begin(image_indices), end(image_indices));
    return image_indices;
}

std::shared_ptr<FeaturePointsContainer> create_feature_container_lru_cache(const FeaturePointsPath &paths, int ncache) {
    std::shared_ptr<FeaturePointsContainer> feature = std::make_shared<FeaturePointsContainerLru>(ncache);
    feature->load(paths);
    return feature;
}
} // namespace h2o

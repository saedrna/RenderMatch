/*!
 * \file recon_match.cpp
 *
 * \author Han
 * \date 2017/05/08
 *
 * 匹配结构
 */
#include <fstream>

#include <match/recon_match.h>
#include <base/base.h>

namespace h2o {
/************************************************************************/
/*               PairwiseMatchContainer IO                              */
/************************************************************************/
IndexMatches PairwiseMatchContainer::read_match(const std::string &path) {
    std::ifstream ifile(string_utf8_to_fstream(path), std::ios::binary);
    int nmatch;
    ifile.read((char *)&nmatch, sizeof(int));
    CHECK(nmatch > 0);

    IndexMatches matches(nmatch);
    int size_match = nmatch * sizeof(IndexMatch);
    ifile.read((char *)matches.data(), size_match);

    return matches;
}

void PairwiseMatchContainer::save_matches(const std::string &path, const IndexMatches &matches) {
    std::ofstream ofile(string_utf8_to_fstream(path), std::ios::binary);
    int nmatch = matches.size();
    CHECK(nmatch > 0);

    ofile.write((char *)&nmatch, sizeof(int));
    int size_match = nmatch * sizeof(IndexMatch);

    ofile.write((char *)matches.data(), size_match);
    return;
}

/************************************************************************/
/*                                                                      */
/************************************************************************/
class PairwiseMatchContainerAllInMemory : public PairwiseMatchContainer {
public:
    PairwiseMatchContainerAllInMemory() {}
    ~PairwiseMatchContainerAllInMemory() {}
    void set_matches(std::shared_ptr<PairwiseMatches> _matches) { matches = _matches; }

    void load(const PairwiseMatchePaths &paths) override;
    IndexMatches get_matches(const ImagePair &pair) override { return matches->at(pair); }
    ImagePairs get_pairs() override;

private:
    std::shared_ptr<PairwiseMatches> matches;
};
void PairwiseMatchContainerAllInMemory::load(const PairwiseMatchePaths &paths) {
    matches = std::make_shared<PairwiseMatches>();
    matches->clear();
    for (auto pair : paths) {
        matches->insert({pair.first, read_match(pair.second)});
    }
}

ImagePairs PairwiseMatchContainerAllInMemory::get_pairs() {
    ImagePairs pairs;
    for (const auto &pair : *matches) {
        pairs.push_back(pair.first);
    }
    std::sort(begin(pairs), end(pairs), [](const ImagePair &pair1, const ImagePair &pair2) {
        CHECK(pair1.first <= pair1.second);
        CHECK(pair2.first <= pair2.second);
        if (pair1.first < pair2.first) {
            return true;
        } else if (pair1.first == pair2.first) {
            return pair1.second < pair2.second;
        } else {
            return false;
        }
        return true;
    });
    return pairs;
}

std::shared_ptr<PairwiseMatchContainer> create_match_container_all_in_memory(const PairwiseMatchePaths &paths) {
    std::shared_ptr<PairwiseMatchContainer> match = std::make_shared<PairwiseMatchContainerAllInMemory>();
    match->load(paths);
    return match;
}

std::shared_ptr<PairwiseMatchContainer> create_match_container_all_in_memory(std::shared_ptr<PairwiseMatches> matches) {
    std::shared_ptr<PairwiseMatchContainer> match = std::make_shared<PairwiseMatchContainerAllInMemory>();
    ((PairwiseMatchContainerAllInMemory *)match.get())->set_matches(matches);
    return match;
}

class PairwiseMatchContainerLru : public PairwiseMatchContainer {
public:
    PairwiseMatchContainerLru(int ncache) : cache(ncache) {}

    void load(const PairwiseMatchePaths &_paths) override;
    IndexMatches get_matches(const ImagePair &pair) override;
    ImagePairs get_pairs() override;

private:
    PairwiseMatchePaths paths;
    Cache<ImagePair, IndexMatches, std::mutex,
          std::unordered_map<ImagePair, typename std::list<KeyValuePair<ImagePair, IndexMatches>>::iterator, hash_pair>>
        cache;
};

std::shared_ptr<PairwiseMatchContainer> create_match_container_lru_cache(const PairwiseMatchePaths &paths, int ncache) {
    std::shared_ptr<PairwiseMatchContainer> match = std::make_shared<PairwiseMatchContainerLru>(ncache);
    match->load(paths);
    return match;
}

void PairwiseMatchContainerLru::load(const PairwiseMatchePaths &_paths) { paths = _paths; }

IndexMatches PairwiseMatchContainerLru::get_matches(const ImagePair &pair) {
    if (cache.contains(pair)) {
        return cache.get(pair);
    } else {
        CHECK(paths.count(pair) > 0);
        IndexMatches matches = read_match(paths.at(pair));
        cache.insert(pair, matches);
        return matches;
    }
    return IndexMatches();
}

ImagePairs PairwiseMatchContainerLru::get_pairs() {
    ImagePairs pairs;
    for (const auto &pair : paths) {
        pairs.push_back(pair.first);
    }
    std::sort(begin(pairs), end(pairs), [](const ImagePair &pair1, const ImagePair &pair2) {
        CHECK(pair1.first <= pair1.second);
        CHECK(pair2.first <= pair2.second);
        if (pair1.first < pair2.first) {
            return true;
        } else if (pair1.first == pair2.first) {
            return pair1.second < pair2.second;
        } else {
            return false;
        }
        return true;
    });
    return pairs;
}

} // namespace h2o

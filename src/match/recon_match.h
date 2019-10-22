/*!
 * \file recon_match.h
 *
 * \author Han
 * \date 2017/05/08
 *
 * 匹配结构
 */
#pragma once
#include <base/common.h>
#include <memory>
#include <unordered_map>
namespace h2o {

///\brief 一个两两匹配点
 struct IndexMatch {
     ///\brief first image
     fid_t i;
     ///\brief second image
     fid_t j;

     IndexMatch() : i(UNDEFINED_ID), j(UNDEFINED_ID) {}
     IndexMatch(int _i, int _j) : i(_i), j(_j) {}
 };

///\brief 一个立体像对的匹配点
typedef std::vector<IndexMatch> IndexMatches;

///\brief 影像像对
typedef std::pair<iid_t, iid_t> ImagePair;
typedef std::vector<ImagePair> ImagePairs;

///\brief pair -> matches
typedef std::unordered_map<ImagePair, IndexMatches, hash_pair> PairwiseMatches;

///\brief pair -> path
typedef std::unordered_map<ImagePair, std::string, hash_pair> PairwiseMatchePaths;

/**
 * \brief 可提供所有匹配点信息，可通过 all in memory 和 lru cache 进行封装
 */
class PairwiseMatchContainer {
public:
    PairwiseMatchContainer(){};
    ~PairwiseMatchContainer(){};

    virtual void load(const PairwiseMatchePaths &paths) = 0;
    virtual IndexMatches get_matches(const ImagePair &pair) = 0;
    virtual ImagePairs get_pairs() = 0;

    static IndexMatches read_match(const std::string &path);
    static void save_matches(const std::string &path, const IndexMatches &matches);
};

/**
 * \brief 读取全部匹配文件，并按需返回
 * \param paths 匹配文件路径
 */
std::shared_ptr<PairwiseMatchContainer> create_match_container_all_in_memory(const PairwiseMatchePaths &paths);

/**
 * \brief 读取全部匹配文件，并按需返回
 * \param matches 匹配信息
 */
std::shared_ptr<PairwiseMatchContainer> create_match_container_all_in_memory(std::shared_ptr<PairwiseMatches> matches);

/**
 * \brief 缓存一定数目的匹配信息
 * \param paths 匹配文件路径
 * \param ncache 缓存大小
 */
std::shared_ptr<PairwiseMatchContainer> create_match_container_lru_cache(const PairwiseMatchePaths &paths, int ncache);
} // namespace h2o

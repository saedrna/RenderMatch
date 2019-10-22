#ifndef __NN_KDTREE_HPP
#define __NN_KDTREE_HPP

#include <algorithm>
#include <cassert>
#include <functional>
#include <queue>
#include <random>
#include <vector>

#include <stdint.h>

#include "dist_l2_funcs.hpp"

namespace fastann {

namespace nn_kdtree_internal {

static const unsigned leaf_max_points = 14;
static const unsigned varest_max_points = 128;
static const unsigned varest_max_randsz = 5;

template <class DistFloat> struct second_cmp_functor {
    bool operator()(const std::pair<unsigned, DistFloat> &lhs, const std::pair<unsigned, DistFloat> &rhs) const {
        return lhs.second < rhs.second;
    }
};

template <class Float> class kdtree_node;

template <class Float> class kdtree_types {
  public:
    typedef Float DiscFloat; // Discriminant dimension type.
    typedef Float DistFloat; // Distance type.
};

template <> class kdtree_types<unsigned char> {
  public:
    typedef float DiscFloat;
    typedef unsigned DistFloat;
};

template <class Float> class kdtree_node {
    typedef kdtree_node<Float> this_type;

  public:
    typedef typename kdtree_types<Float>::DiscFloat DiscFloat;
    typedef typename kdtree_types<Float>::DistFloat DistFloat;
    typedef std::priority_queue<std::pair<DiscFloat, kdtree_node<Float> *>,
                                std::vector<std::pair<DiscFloat, kdtree_node<Float> *>>,
                                std::greater<std::pair<DiscFloat, kdtree_node<Float> *>>>
        BPQ;

  public:
    struct internal_node_data_ {
        this_type *right_;
        DiscFloat disc_;
        unsigned disc_dim_;
    };
    struct leaf_node_data_ {
        unsigned num_points_;
        unsigned indices_[leaf_max_points];
    };
    /**
     * left_ == 0 iff this node is a leaf.
     */
    this_type *left_;

    union {
        struct internal_node_data_ internal_node_data;
        struct leaf_node_data_ leaf_node_data;
    };

    inline bool is_leaf() const { return left_ == 0; }

    std::pair<unsigned, DiscFloat> choose_split(const Float *pnts, const unsigned *inds, unsigned N, unsigned D,
                                                std::default_random_engine *state) {
        // Find mean & variance of each dimension.
        std::vector<DiscFloat> sum_x(D, DiscFloat(0));
        std::vector<DiscFloat> sum_xx(D, DiscFloat(0));
        unsigned count = std::min(N, varest_max_points);
        for (unsigned n = 0; n < count; ++n) {
            for (unsigned d = 0; d < D; ++d) {
                sum_x[d] += pnts[inds[n] * D + d];
                sum_xx[d] += (pnts[inds[n] * D + d] * pnts[inds[n] * D + d]);
            }
        }

        std::vector<std::pair<DiscFloat, unsigned>> var_dim(D);
        for (unsigned d = 0; d < D; ++d) {
            if (count <= 1)
                var_dim[d].first = DiscFloat(0);
            else
                var_dim[d].first = (sum_xx[d] - (DiscFloat(1) / count) * sum_x[d] * sum_x[d]) / (count - 1);
            var_dim[d].second = d;
        }

        // Partial sort makes a BIG difference to the build time.
        unsigned nrand = std::min(varest_max_randsz, D);
        std::partial_sort(var_dim.begin(), var_dim.begin() + nrand, var_dim.end(),
                          std::greater<std::pair<DiscFloat, unsigned>>());
        unsigned randd = var_dim[(*state)() % (nrand - 1)].second;

        return std::make_pair(randd, sum_x[randd] / count);
    }

    void split_points(const Float *pnts, unsigned *inds, unsigned N, unsigned D, std::default_random_engine *state) {
        std::pair<unsigned, DiscFloat> spl = choose_split(pnts, inds, N, D, state);

        internal_node_data.disc_dim_ = spl.first;
        internal_node_data.disc_ = spl.second;

        size_t l = 0;
        size_t r = N;
        while (l != r) {
            if (pnts[inds[l] * D + internal_node_data.disc_dim_] < internal_node_data.disc_)
                l++;
            else {
                r--;
                std::swap(inds[l], inds[r]);
            }
        }

        // If either partition is empty -> vectors identical!
        if (l == 0 || l == N) {
            l = N / 2;
        } // The vectors are identical, so keep nlogn performance.

        size_t left_sz = sizeof(this_type);
        size_t right_sz = sizeof(this_type);
        if (l > leaf_max_points) { // The left node will be internal
            left_sz = sizeof(void *) + sizeof(internal_node_data);
        }
        if ((N - l) > leaf_max_points) { // The right node will be internal
            right_sz = sizeof(void *) + sizeof(internal_node_data);
        }

        left_ = (this_type *)malloc(left_sz);
        internal_node_data.right_ = (this_type *)malloc(right_sz);

        new (left_) this_type(pnts, inds, l, D, state);
        new (internal_node_data.right_) this_type(pnts, &inds[l], N - l, D, state);
    }

  public:
    kdtree_node() : left_(0) /*, right_(0)*/ {}

    kdtree_node(const Float *pnts, unsigned *inds, unsigned N, unsigned D, std::default_random_engine *state)
        : left_(0) /*, right_(0)*/
    {
        if (N > leaf_max_points) { // Internal node
            split_points(pnts, inds, N, D, state);
        } else {
            leaf_node_data.num_points_ = N;
            std::copy(inds, inds + N, leaf_node_data.indices_);
        }
    }

    ~kdtree_node() {
        if (!is_leaf()) {
            left_->~kdtree_node();
            internal_node_data.right_->~kdtree_node();
            free(left_);
            free(internal_node_data.right_);
        }
    }

    void search(const Float *qu, BPQ &pri_branch, dist_l2_wrapper<Float> dist,
                std::vector<std::pair<unsigned, DistFloat>> &nns, std::vector<bool> &seen, const Float *pnts,
                unsigned D, DiscFloat mindsq) {
        this_type *cur = this;
        this_type *other = 0;

        while (!cur->is_leaf()) { // Follow best bin first until we hit a leaf
            DiscFloat diff = qu[cur->internal_node_data.disc_dim_] - cur->internal_node_data.disc_;

            if (diff < 0) {
                other = cur->internal_node_data.right_;
                cur = cur->left_;
            } else {
                other = cur->left_;
                cur = cur->internal_node_data.right_;
            }

            pri_branch.push(std::make_pair(mindsq + diff * diff, other));
        }

        unsigned *cur_inds = cur->leaf_node_data.indices_;
        unsigned ncur_inds = cur->leaf_node_data.num_points_;

        unsigned i;
        for (i = 0; i < ncur_inds; ++i) {
            if (!seen[cur_inds[i]]) {
                DistFloat dsq;
                dist.func(qu, &pnts[cur_inds[i] * D], 1, D, &dsq);
                nns.push_back(std::make_pair(cur_inds[i], dsq));

                seen[cur_inds[i]] = true;
            }
        }
    }
};
}

template <class Float> class nn_kdtree {
    typedef nn_kdtree_internal::kdtree_node<Float> node_type;
    typedef typename node_type::DiscFloat DiscFloat;
    typedef typename node_type::DistFloat DistFloat;
    typedef typename node_type::BPQ BPQ;

    std::vector<node_type *> trees_;
    unsigned N_;
    unsigned D_;
    const Float *pnts_;

    std::default_random_engine state_;

  public:
    nn_kdtree(const Float *pnts, unsigned N, unsigned D, unsigned ntrees = 8, unsigned seed = 42)
        : N_(N), D_(D), pnts_(pnts) {
        // 保证结果是可 reproduced
        state_.seed();
        // Create inds.
        std::vector<unsigned> inds(N);
        for (size_t n = 0; n < N; ++n)
            inds[n] = n;

        // Create trees.
        for (unsigned t = 0; t < ntrees; ++t) {
            trees_.push_back(new node_type(pnts, &inds[0], N, D, &state_));
        }
    }

    ~nn_kdtree() {
        for (size_t t = 0; t < trees_.size(); ++t) {
            delete trees_[t];
        }
    }

    void search(const Float *qu, dist_l2_wrapper<Float> dist, unsigned numnn, std::pair<unsigned, DistFloat> *ret_nns,
                unsigned nchecks) const {
        // We should 16-byte align qu here.
        Float *qu_new_base = (Float *)malloc(D_ * sizeof(Float) + 16);
        Float *qu_new = (Float *)(((intptr_t)qu_new_base + 15) & -16);
        std::copy(qu, qu + D_, qu_new);

        if (nchecks < numnn) {
            nchecks = numnn;
        }
        if (nchecks > N_) {
            nchecks = N_;
        }
        BPQ pri_branch;

        std::vector<std::pair<unsigned, DistFloat>> nns;
        std::vector<bool> seen(N_, false);

        nns.reserve((3 * nchecks) / 2); // Might as well

        // Search each tree at least once.
        for (size_t t = 0; t < trees_.size(); ++t) {
            trees_[t]->search(qu, pri_branch, dist, nns, seen, pnts_, D_, DiscFloat());
        }

        // Continue search until we've performed enough distances
        nn_kdtree_internal::second_cmp_functor<DistFloat> cmp;
        while (nns.size() < nchecks) {
            std::pair<DiscFloat, node_type *> pr = pri_branch.top();
            pri_branch.pop();

            pr.second->search(qu_new, pri_branch, dist, nns, seen, pnts_, D_, pr.first);
        }

        std::partial_sort(nns.begin(), nns.begin() + numnn, nns.end(), cmp);

        std::copy(nns.begin(), nns.begin() + std::min(numnn, nchecks), ret_nns);

        free(qu_new_base);
    }
};
}

#endif

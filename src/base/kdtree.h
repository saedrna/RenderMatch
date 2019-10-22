/*!
 * \file kdtree.h
 *
 * \author Han
 * \date 2017/05/13
 *
 * kd 树
 */
#pragma once

#include <base/common.h>
#include <nanoflann.hpp>

namespace nanoflann {
template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2> struct KDTreeAdaptor {
    using self_t = KDTreeAdaptor<MatrixType, DIM, Distance>;
    using num_t = typename MatrixType::Scalar;
    using IndexType = typename MatrixType::Index;
    using metric_t = typename Distance::template traits<num_t, self_t>::distance_t;
    using index_t = KDTreeSingleIndexAdaptor<metric_t, self_t, DIM, IndexType>;

    index_t *index; //! The kd-tree index for the user to call its methods as usual with any other FLANN index.

    KDTreeAdaptor(const int dimensionality, const MatrixType &mat, const int leaf_max_size = 10) : m_data_matrix(mat) {
        const IndexType dims = dimensionality /*mat.rows()*/;
        if (dims != dimensionality)
            throw std::runtime_error("Error: 'dimensionality' must match column count in data matrix");
        if (DIM > 0 && static_cast<int>(dims) != DIM)
            throw std::runtime_error("Data set dimensionality does not match the 'DIM' template argument");
        index = new index_t(dims, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size));
        index->buildIndex();
    }

private:
    /** Hidden copy constructor, to disallow copying this class (Not implemented) */
    KDTreeAdaptor(const self_t &);

public:
    ~KDTreeAdaptor() { delete index; }

    const MatrixType &m_data_matrix;

    /** Query for the \a num_closest closest points to a given point (entered as query_point[0:dim-1]).
     *  Note that this is a short-cut method for index->findNeighbors().
     *  The user can also call index->... methods as desired.
     * \note nChecks_IGNORED is ignored but kept for compatibility with the original FLANN interface.
     */
    inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices,
                      num_t *out_distances_sq, const int /* nChecks_IGNORED */ = 10) const {
        nanoflann::KNNResultSet<num_t, IndexType> resultSet(num_closest);
        resultSet.init(out_indices, out_distances_sq);
        index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
    }

    inline void query(const num_t *query_point, const num_t radius, std::vector<std::pair<IndexType, num_t>> &results,
                      const int /* nChecks_IGNORED */ = 10) const {
        index->radiusSearch(query_point, radius * radius, results, nanoflann::SearchParams());
    }

    /** @name Interface expected by KDTreeSingleIndexAdaptor
     * @{ */

    const self_t &derived() const { return *this; }
    self_t &derived() { return *this; }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return m_data_matrix.cols(); }

    // Returns the L2 distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the
    // class:
    inline num_t kdtree_distance(const num_t *p1, const IndexType idx_p2, IndexType size) const {
        num_t s = 0;
        for (IndexType i = 0; i < size; i++) {
            const num_t d = p1[i] - m_data_matrix.coeff(i, idx_p2);
            s += d * d;
        }
        return s;
    }

    // Returns the dim'th component of the idx'th point in the class:
    inline num_t kdtree_get_pt(const IndexType idx, int dim) const { return m_data_matrix.coeff(IndexType(dim), idx); }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it
    //   again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX> bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
};
} // namespace nanoflann

using KDTreeAdaptor = nanoflann::KDTreeAdaptor<Eigen::MatrixXf, 2, nanoflann::metric_L2>;
using KDTreeAdaptorPtr = std::shared_ptr<KDTreeAdaptor>;

#include "fastann.hpp"
#include "dist_l2.hpp"
#include "nn_kdtree.hpp"

namespace fastann {

template <class Float> class nn_obj_exact : public nn_obj<Float> {
  public:
    typedef typename nn_obj<Float>::float_type float_type;
    typedef typename nn_obj<Float>::accum_float_type accum_float_type;

    virtual void search_nn(const float_type *qus, unsigned N, unsigned *argmins, accum_float_type *mins) const {
        std::vector<accum_float_type> dsqout(npoints_);
        for (unsigned n = 0; n < N; ++n) {
            dist_.func(qus + n * ndims_, pnts_, npoints_, ndims_, &dsqout[0]);

            argmins[n] = (unsigned)(std::min_element(dsqout.begin(), dsqout.end()) - dsqout.begin());
            mins[n] = dsqout[argmins[n]];
        }
    }

    virtual void search_knn(const float_type *qus, unsigned N, unsigned K, unsigned *argmins,
                            accum_float_type *mins) const {
        // Fix for when the user asks for too many points.
        K = std::min(K, npoints_);
        std::vector<accum_float_type> dsqout(npoints_);
        std::vector<std::pair<accum_float_type, unsigned>> knn_prs(npoints_);
        for (unsigned n = 0; n < N; ++n) {
            dist_.func(qus + n * ndims_, pnts_, npoints_, ndims_, &dsqout[0]);

            for (unsigned p = 0; p < npoints_; ++p)
                knn_prs[p] = std::make_pair(dsqout[p], p);

            std::partial_sort(knn_prs.begin(), knn_prs.begin() + K, knn_prs.end());

            for (unsigned k = 0; k < K; ++k) {
                argmins[n * K + k] = knn_prs[k].second;
                mins[n * K + k] = knn_prs[k].first;
            }
        }
    }

    virtual unsigned ndims() const { return ndims_; }
    virtual unsigned npoints() const { return npoints_; }

    nn_obj_exact(const Float *pnts, unsigned N, unsigned D)
        : pnts_(pnts), ndims_(D), npoints_(N), dist_(dist_l2_best<Float>(D)) {}

  private:
    const Float *pnts_;
    unsigned ndims_;
    unsigned npoints_;
    dist_l2_wrapper<Float> dist_;
};

template <class Float> class nn_obj_kdtree : public nn_obj<Float> {
  public:
    typedef typename nn_obj<Float>::float_type float_type;
    typedef typename nn_obj<Float>::accum_float_type accum_float_type;

    virtual void search_nn(const float_type *qus, unsigned N, unsigned *argmins, accum_float_type *mins) const {
        for (unsigned n = 0; n < N; ++n) {
            std::pair<unsigned, accum_float_type> nn;
            kdt_.search(qus + n * ndims_, dist_, 1, &nn, nchecks_);
            argmins[n] = nn.first;
            mins[n] = nn.second;
        }
    }

    virtual void search_knn(const float_type *qus, unsigned N, unsigned K, unsigned *argmins,
                            accum_float_type *mins) const {
        // Fix for when the user asks for too many points.
        K = std::min(K, npoints_);
        std::vector<std::pair<unsigned, accum_float_type>> nns(K);
        for (unsigned n = 0; n < N; ++n) {
            kdt_.search(qus + n * ndims_, dist_, K, &nns[0], nchecks_);
            for (unsigned k = 0; k < K; ++k) {
                argmins[n * K + k] = nns[k].first;
                mins[n * K + k] = nns[k].second;
            }
        }
    }

    virtual unsigned ndims() const { return ndims_; }
    virtual unsigned npoints() const { return npoints_; }

    nn_obj_kdtree(const Float *pnts, unsigned N, unsigned D, unsigned ntrees, unsigned nchecks)
        : kdt_(pnts, N, D, ntrees), npoints_(N), ndims_(D), nchecks_(nchecks), dist_(dist_l2_best<Float>(D)) {}

    virtual ~nn_obj_kdtree() {}

  private:
    nn_kdtree<Float> kdt_;
    unsigned npoints_;
    unsigned ndims_;
    unsigned nchecks_;
    dist_l2_wrapper<Float> dist_;
};

template <class Float>
nn_obj<Float> *nn_obj_build_kdtree(const Float *pnts, unsigned N, unsigned D, unsigned ntrees, unsigned nchecks) {
    return new nn_obj_kdtree<Float>(pnts, N, D, ntrees, nchecks);
}

template nn_obj<unsigned char> *nn_obj_build_kdtree<unsigned char>(const unsigned char *pnts, unsigned N, unsigned D,
                                                                   unsigned ntrees, unsigned nchecks);

template nn_obj<float> *nn_obj_build_kdtree<float>(const float *pnts, unsigned N, unsigned D, unsigned ntrees,
                                                   unsigned nchecks);

template nn_obj<double> *nn_obj_build_kdtree<double>(const double *pnts, unsigned N, unsigned D, unsigned ntrees,
                                                     unsigned nchecks);

template <class Float> nn_obj<Float> *nn_obj_build_exact(const Float *pnts, unsigned N, unsigned D) {
    return new nn_obj_exact<Float>(pnts, N, D);
}
template nn_obj<unsigned char> *nn_obj_build_exact(const unsigned char *pnts, unsigned N, unsigned D);
template nn_obj<float> *nn_obj_build_exact(const float *pnts, unsigned N, unsigned D);
template nn_obj<double> *nn_obj_build_exact(const double *pnts, unsigned N, unsigned D);
}

/**
 * C interface.
 */
#define JOIN(a, b) a##b
#define C_BUILD_EXACT(SUFFIX, TYPE)                                                                                    \
    extern "C" void *JOIN(fastann_nn_obj_build_exact_, SUFFIX)(TYPE * pnts, unsigned N, unsigned D) {                  \
        fastann::nn_obj<TYPE> *nno = new fastann::nn_obj_exact<TYPE>(pnts, N, D);                                      \
        return (void *)nno;                                                                                            \
    }
C_BUILD_EXACT(c, unsigned char)
C_BUILD_EXACT(s, float)
C_BUILD_EXACT(d, double)
#undef C_BUILD_EXACT

#define C_BUILD_KDT(SUFFIX, TYPE)                                                                                      \
    extern "C" void *JOIN(fastann_nn_obj_build_kdtree_, SUFFIX)(TYPE * pnts, unsigned N, unsigned D, unsigned ntrees,  \
                                                                unsigned nchecks) {                                    \
        fastann::nn_obj_kdtree<TYPE> *nno = new fastann::nn_obj_kdtree<TYPE>(pnts, N, D, ntrees, nchecks);             \
        return (void *)nno;                                                                                            \
    }
C_BUILD_KDT(c, unsigned char)
C_BUILD_KDT(s, float)
C_BUILD_KDT(d, double)
#undef C_BUILD_KDT

#define C_NNOBJ_SEARCH_NN(SUFFIX, TYPE1, TYPE2)                                                                        \
    extern "C" void JOIN(fastann_nn_obj_search_nn_, SUFFIX)(void *p, TYPE1 *qus, unsigned N, unsigned *argmins,        \
                                                            TYPE2 *mins) {                                             \
        fastann::nn_obj<TYPE1> *nno = (fastann::nn_obj<TYPE1> *)p;                                                     \
        nno->search_nn(qus, N, argmins, mins);                                                                         \
    }
C_NNOBJ_SEARCH_NN(c, unsigned char, unsigned)
C_NNOBJ_SEARCH_NN(s, float, float)
C_NNOBJ_SEARCH_NN(d, double, double)
#undef C_NNOBJ_SEARCH_NN

#define C_NNOBJ_SEARCH_KNN(SUFFIX, TYPE1, TYPE2)                                                                       \
    extern "C" void JOIN(fastann_nn_obj_search_knn_, SUFFIX)(void *p, TYPE1 *qus, unsigned N, unsigned K,              \
                                                             unsigned *argmins, TYPE2 *mins) {                         \
        fastann::nn_obj<TYPE1> *nno = (fastann::nn_obj<TYPE1> *)p;                                                     \
        nno->search_knn(qus, N, K, argmins, mins);                                                                     \
    }
C_NNOBJ_SEARCH_KNN(c, unsigned char, unsigned)
C_NNOBJ_SEARCH_KNN(s, float, float)
C_NNOBJ_SEARCH_KNN(d, double, double)
#undef C_NNOBJ_SEARCH_KNN

#define C_NNOBJ_ADD_POINTS(SUFFIX, TYPE)                                                                               \
    extern "C" void JOIN(fastann_nn_obj_add_points_, SUFFIX)(void *p, TYPE *pnts, unsigned N) {                        \
        fastann::nn_obj<TYPE> *nno = (fastann::nn_obj<TYPE> *)p;                                                       \
        nno->add_points(pnts, N);                                                                                      \
    }
C_NNOBJ_ADD_POINTS(c, unsigned char)
C_NNOBJ_ADD_POINTS(s, float)
C_NNOBJ_ADD_POINTS(d, double)
#undef C_NNOBJ_ADD_POINTS

#define C_NNOBJ_NDIMS(SUFFIX, TYPE)                                                                                    \
    extern "C" unsigned JOIN(fastann_nn_obj_ndims_, SUFFIX)(void *p) {                                                 \
        fastann::nn_obj<TYPE> *nno = (fastann::nn_obj<TYPE> *)p;                                                       \
        return nno->ndims();                                                                                           \
    }
C_NNOBJ_NDIMS(c, unsigned char)
C_NNOBJ_NDIMS(s, float)
C_NNOBJ_NDIMS(d, double)
#undef C_NNOBJ_NDIMS

#define C_NNOBJ_NPOINTS(SUFFIX, TYPE)                                                                                  \
    extern "C" unsigned JOIN(fastann_nn_obj_npoints_, SUFFIX)(void *p) {                                               \
        fastann::nn_obj<TYPE> *nno = (fastann::nn_obj<TYPE> *)p;                                                       \
        return nno->npoints();                                                                                         \
    }
C_NNOBJ_NPOINTS(c, unsigned char)
C_NNOBJ_NPOINTS(s, float)
C_NNOBJ_NPOINTS(d, double)
#undef C_NNOBJ_NDIMS

#define C_NNOBJ_DEL(SUFFIX, TYPE)                                                                                      \
    extern "C" void JOIN(fastann_nn_obj_del_, SUFFIX)(void *p) {                                                       \
        fastann::nn_obj<TYPE> *nno = (fastann::nn_obj<TYPE> *)p;                                                       \
        delete nno;                                                                                                    \
    }
C_NNOBJ_DEL(c, unsigned char)
C_NNOBJ_DEL(s, float)
C_NNOBJ_DEL(d, double)
#undef C_NNOBJ_NDIMS

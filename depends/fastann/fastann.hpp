/**
 * Functions for returning a 'nn_obj' -- an abstract base class which
 * can perform nearest neighbour searches.
 */
#ifndef __FASTANN_FASTANN_HPP
#define __FASTANN_FASTANN_HPP

//#include "rand_point_gen.hpp"

namespace fastann {

template <class Float> class nn_obj {
  public:
    typedef Float float_type;
    typedef Float accum_float_type;

    virtual void search_nn(const Float *qus, unsigned N, unsigned *argmins, Float *mins) const = 0;
    virtual void search_knn(const Float *qus, unsigned N, unsigned K, unsigned *argmins, Float *mins) const = 0;

    virtual void add_points(const Float *pnts, unsigned N) {}

    virtual unsigned ndims() const = 0;
    virtual unsigned npoints() const = 0;

    virtual ~nn_obj() {}
};

template <> class nn_obj<unsigned char> {
  public:
    typedef unsigned char float_type;
    typedef unsigned accum_float_type;

    virtual void search_nn(const unsigned char *qus, unsigned N, unsigned *argmins, unsigned *mins) const = 0;
    virtual void search_knn(const unsigned char *qus, unsigned N, unsigned K, unsigned *argmins,
                            unsigned *mins) const = 0;

    virtual void add_points(const unsigned char *pnts, unsigned N) {}

    virtual unsigned ndims() const = 0;
    virtual unsigned npoints() const = 0;

    virtual ~nn_obj() {}
};

template <class Float> nn_obj<Float> *nn_obj_build_exact(const Float *pnts, unsigned N, unsigned D);

template <class Float>
nn_obj<Float> *nn_obj_build_kdtree(const Float *pnts, unsigned N, unsigned D, unsigned ntrees, unsigned nchecks);
}

#endif

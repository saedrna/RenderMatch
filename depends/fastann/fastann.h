#ifndef __FASTANN_FASTANN_H
#define __FASTANN_FASTANN_H

#define JOIN(a, b) a##b
#define C_BUILD_EXACT(SUFFIX, TYPE)                                                                                    \
    extern "C" void *JOIN(fastann_nn_obj_build_exact_, SUFFIX)(TYPE * pnts, unsigned N, unsigned D);
C_BUILD_EXACT(c, unsigned char)
C_BUILD_EXACT(s, float)
C_BUILD_EXACT(d, double)
#undef C_BUILD_EXACT

#define C_BUILD_KDT(SUFFIX, TYPE)                                                                                      \
    extern "C" void *JOIN(fastann_nn_obj_build_kdtree_, SUFFIX)(TYPE * pnts, unsigned N, unsigned D, unsigned ntrees,  \
                                                                unsigned nchecks);
C_BUILD_KDT(c, unsigned char)
C_BUILD_KDT(s, float)
C_BUILD_KDT(d, double)
#undef C_BUILD_KDT

#define C_NNOBJ_SEARCH_NN(SUFFIX, TYPE1, TYPE2)                                                                        \
    extern "C" void JOIN(fastann_nn_obj_search_nn_, SUFFIX)(void *p, TYPE1 *qus, unsigned N, unsigned *argmins,        \
                                                            TYPE2 *mins);
C_NNOBJ_SEARCH_NN(c, unsigned char, unsigned)
C_NNOBJ_SEARCH_NN(s, float, float)
C_NNOBJ_SEARCH_NN(d, double, double)
#undef C_NNOBJ_SEARCH_NN

#define C_NNOBJ_SEARCH_KNN(SUFFIX, TYPE1, TYPE2)                                                                       \
    extern "C" void JOIN(fastann_nn_obj_search_knn_, SUFFIX)(void *p, TYPE1 *qus, unsigned N, unsigned K,              \
                                                             unsigned *argmins, TYPE2 *mins);
C_NNOBJ_SEARCH_KNN(c, unsigned char, unsigned)
C_NNOBJ_SEARCH_KNN(s, float, float)
C_NNOBJ_SEARCH_KNN(d, double, double)
#undef C_NNOBJ_SEARCH_KNN

#define C_NNOBJ_ADD_POINTS(SUFFIX, TYPE)                                                                               \
    extern "C" void JOIN(fastann_nn_obj_add_points_, SUFFIX)(void *p, TYPE *pnts, unsigned N);
C_NNOBJ_ADD_POINTS(c, unsigned char)
C_NNOBJ_ADD_POINTS(s, float)
C_NNOBJ_ADD_POINTS(d, double)
#undef C_NNOBJ_ADD_POINTS

#define C_NNOBJ_NDIMS(SUFFIX, TYPE) extern "C" unsigned JOIN(fastann_nn_obj_ndims_, SUFFIX)(void *p);
C_NNOBJ_NDIMS(c, unsigned char)
C_NNOBJ_NDIMS(s, float)
C_NNOBJ_NDIMS(d, double)
#undef C_NNOBJ_NDIMS

#define C_NNOBJ_NPOINTS(SUFFIX, TYPE) extern "C" unsigned JOIN(fastann_nn_obj_npoints_, SUFFIX)(void *p);
C_NNOBJ_NPOINTS(c, unsigned char)
C_NNOBJ_NPOINTS(s, float)
C_NNOBJ_NPOINTS(d, double)
#undef C_NNOBJ_NDIMS

#define C_NNOBJ_DEL(SUFFIX, TYPE) extern "C" void JOIN(fastann_nn_obj_del_, SUFFIX)(void *p);
C_NNOBJ_DEL(c, unsigned char)
C_NNOBJ_DEL(s, float)
C_NNOBJ_DEL(d, double)
#undef C_NNOBJ_NDIMS

#endif

mkdir build
cd build

# -DLaszip_LIBRARY_RELEASE=$CONDA_PREFIX/lib/liblaszip.so \
#       -DLaszip_LIBRARY_DEBUG=$CONDA_PREFIX/lib/liblaszip.so \
#       -DOpenMP_gomp_LIBRARY=$CONDA_PREFIX/lib/libgomp.so \
#       -DTIFF_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libtiff.so \
#       -DTIFF_LIBRARY_DEBUG=$CONDA_PREFIX/lib/libtiff.so \
#       -DOpenMP_pthread_LIBRARY=$CONDA_PREFIX/$HOST/sysroot/usr/lib/libpthread.so \


cmake -GNinja \
      -DCMAKE_PREFIX_PATH=$CONDA_PREFIX \
      -DCMAKE_SYSROOT=$CONDA_PREFIX/$HOST/sysroot \
      -DCMAKE_BUILD_TYPE=Release \
      -DOpenGL_GL_PREFERENCE=LEGACY \
      -DOPENGL_gl_LIBRARY=$CONDA_PREFIX/$HOST/sysroot/usr/lib64/libGL.so \
      -DOSG_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libosg.so \
      -DOSGDB_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libosgDB.so \
      -DOSGGA_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libosgGA.so \
      -DOSGUTIL_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libosgUtil.so \
      -DOSGVIEWER_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libosgViewer.so \
      -DOPENTHREADS_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libOpenThreads.so \
      -DTIFF_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libtiff.so \
      -DOpenMP_gomp_LIBRARY=$CONDA_PREFIX/lib/libgomp.so \
      -DOpenMP_pthread_LIBRARY=$CONDA_PREFIX/$HOST/sysroot/usr/lib/libpthread.so \
      -DTBB_tbb_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libtbb.so \
      -DTBB_tbbmalloc_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libtbbmalloc.so \
      -DTBB_tbbmalloc_proxy_LIBRARY_RELEASE=$CONDA_PREFIX/lib/libtbbmalloc_proxy.so \
      ../
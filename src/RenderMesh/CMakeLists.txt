file(GLOB SOURCES "*.h" "*.cpp")
h2o_add_executable(RenderMesh ${SOURCES})
target_link_libraries(RenderMesh
                      PRIVATE h2o_base
                              RenderMatch
                              ${OPENSCENEGRAPH_LIBRARIES}
                              OpenGL::GL
                              OpenGL::GLU
                              TBB::tbb
                              TBB::tbbmalloc)

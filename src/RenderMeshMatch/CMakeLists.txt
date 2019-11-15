file(GLOB SOURCES "*.cpp" "*.h")
h2o_add_executable(RenderMeshMatch ${SOURCES})
target_link_libraries(RenderMeshMatch
                      PRIVATE RenderMatch
                              h2o_match
                              h2o_base
                              ${OPENSCENEGRAPH_LIBRARIES}
                              OpenGL::GL
                              OpenGL::GLU)

file(GLOB SOURCES "*.h" "*cpp")
h2o_add_library(h2o_modelpro ${CMAKE_CURRENT_LIST_DIR}/../../ruby/h2o_osketchpro
                ${SOURCES})

target_link_libraries(h2o_modelpro PUBLIC h2o_base h2o_sketchup)
target_link_libraries(h2o_modelpro
                      PRIVATE MathGeoLib
                              Geogram::Geogram
                              Embree3::Embree3
                              Boost::filesystem
                              ${OPENSCENEGRAPH_LIBRARIES}
                              ${OSKETCH_3RDPARTY_LIBRARIES})

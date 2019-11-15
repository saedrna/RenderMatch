file(GLOB SOURCES "*.cpp")
h2o_add_executable(FeatureExtract ${SOURCES})
target_link_libraries(FeatureExtract PRIVATE h2o_base h2o_match RenderMatch)

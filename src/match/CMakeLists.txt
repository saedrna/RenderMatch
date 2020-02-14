file(GLOB MATCH_SRC "*.h" "*.cpp")
h2o_add_library(h2o_match ${MATCH_SRC})
target_link_libraries(h2o_match PUBLIC h2o_base)
target_link_libraries(h2o_match
                      PRIVATE siftgpu
                              fastann
                              acransac
                              ceres)
if(MSVC)
target_link_libraries(h2o_match PRIVATE Winmm)
endif(MSVC) 
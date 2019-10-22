file(GLOB ACRANSAC_SRC "*.h" "*.hpp" "*.cpp")
add_library(acransac STATIC ${ACRANSAC_SRC})
target_link_libraries(acransac ${OpenCV_LIBS})

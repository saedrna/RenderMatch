file(GLOB SOURCES "*.h" "*.cpp")
add_library(RenderMatch STATIC ${SOURCES})
target_link_libraries(RenderMatch h2o_base)

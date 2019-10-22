file(GLOB SOURCES "*.h" "*.cpp")
h2o_add_executable(TileImage ${SOURCES})
target_link_libraries(TileImage h2o_base RenderMatch)

file(GLOB SOURCES "*.cpp" "*.h")
h2o_add_executable(UndistortImage ${SOURCES})
target_link_libraries(UndistortImage PRIVATE h2o_base RenderMatch)

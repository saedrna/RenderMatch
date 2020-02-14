file(GLOB SIFTGPU_SRC "*.h" "*.cpp")

find_package(GLEW REQUIRED)

include_directories(${GLEW_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
add_library(siftgpu STATIC ${SIFTGPU_SRC})

target_link_libraries(siftgpu GLEW::GLEW ${OPENGL_LIBRARIES})
if(UNIX)
    find_package(glfw3 REQUIRED)
    target_link_libraries(siftgpu glfw)
endif()

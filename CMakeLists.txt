cmake_minimum_required(VERSION 3.11)

######################################################################
#
# CMake Project Settings
#
######################################################################
project(RenderMatch LANGUAGES C CXX)
set(VERSION_MAJOR "0")
set(VERSION_MINOR "1")
set(VERSION_BugFix "0")
set(PROJECT_VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_BugFix})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /MP /wd4996 /wd4819 /wd4251 /wd4267 /wd4244 /wd4275 /wd4005 /wd5033 /wd4018")
    add_definitions(-DGLOG_NO_ABBREVIATED_SEVERITIES -DWIN32_LEAN_AND_MEAN -DNOMINMAX -D_ENABLE_EXTENDED_ALIGNED_STORAGE) # windows use windows api
endif(MSVC)
if(APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-inconsistent-missing-override -Wno-unused-command-line-argument -Wno-unused-result -Wno-exceptions")
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.9" CACHE STRING "Minimum OS X deployment version")
endif(APPLE)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

######################################################################
#
# 设置用于 install 的变量
#
######################################################################
# Introduce variables:
# * CMAKE_INSTALL_LIBDIR
# * CMAKE_INSTALL_BINDIR
include(GNUInstallDirs)

# Include module with fuction 'write_basic_package_version_file'
include(CMakePackageConfigHelpers)

# Layout. This works for all platforms:
#   * <prefix>/lib*/cmake/<PROJECT-NAME>
#   * <prefix>/lib*/
#   * <prefix>/include/
set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/H2O")
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
# Configuration
set(version_config "${generated_dir}/H2OConfigVersion.cmake")
set(project_config "${generated_dir}/H2OConfig.cmake")
set(TARGETS_EXPORT_NAME "H2OTargets")
set(namespace "H2O::")

######################################################################
#
# options
#
######################################################################
option(H2O_BUILD_SHARED "" ON)
option(H2O_BUILD_AVX "Use AVX instructions for fastann" OFF)
set(H2O_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/data CACHE PATH "")

if(H2O_BUILD_SHARED)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    set(BUILD_SHARED_LIBS ON)
else()
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS OFF)
    set(BUILD_SHARED_LIBS OFF)
endif(H2O_BUILD_SHARED)

######################################################################
#
# Packages
#
######################################################################
# OpenCL::OpenCL
# find_package(OpenCL REQUIRED)
# viennacl::viennacl
# find_package(viennacl REQUIRED)
# ArrayFire_OpenCL_LIBRARIES
# find_package(ArrayFire REQUIRED OpenCL)

find_package(OpenGL REQUIRED)

# EIGEN3_INCLUDE_DIR
find_package(Eigen3 REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets LinguistTools Network WinExtras)
include(Qt5LinguistToolsMacrosNoInclude) # 添加创建翻译文件的函数

# OpenCV_LIBS
find_package(OpenCV REQUIRED core imgproc calib3d ximgproc highgui imgcodecs features2d videoio flann)

# CGAL::CGAL
find_package(CGAL REQUIRED)
add_definitions(-DCGAL_HEADER_ONLY)

# Boost_LIBRARIES
set(Boost_USE_STATIC_LIBS ON FORCE)
find_package(Boost REQUIRED filesystem locale)
set(Boost_LIBRARIES Boost::filesystem Boost::locale)
add_definitions(-DBOOST_ALL_NO_LIB=1)

# GDAL_LIBRARY
find_package(GDAL REQUIRED)

# glog::glog
find_package(Glog REQUIRED)

# ceres targets
find_package(Ceres REQUIRED)

# IPP_LIBRARIES
find_package(IPP REQUIRED)

# CAPSTONE::CAPSTONE
find_package(CAPSTONE REQUIRED)

# OPENSCENEGRAPH_LIBRARIES OPENSCENEGRAPH_INCLUDE_DIRS
find_package(OpenSceneGraph REQUIRED osg osgDB osgViewer osgGA osgUtil)
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})

# TIFF::TIFF
find_package(TIFF REQUIRED)

# OpenMVG::openMVG_(modules)
# find_package(OpenMVG REQUIRED)

# Geogram::Geogram
find_package(Geogram REQUIRED)

# Embree3::Embree3
find_package(Embree3 REQUIRED)


find_package(TBB REQUIRED)

# openmp
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# OPENGL_LIBRARIES
find_package(OpenGL REQUIRED)

# Assimp::Assimp
# find_package(Assimp REQUIRED)

# if(H2O_BUILD_OSALS)
# # pdal_util pdalcpp
#     find_package(PDAL REQUIRED)
# endif(H2O_BUILD_OSALS)


# 把一些库的RelWithDebInfo Map 到 Release
set(MAP_LIBRARIES ${OpenCV_LIBS} glog::glog ceres CAPSTONE::CAPSTONE)
# if(H2O_BUILD_OSALS)
#     list(APPEND MAP_LIBRARIES pdal_util pdalcpp)
# endif(H2O_BUILD_OSALS)

if(TARGET gflags_shared)
    list(APPEND MAP_LIBRARIES gflags_shared)
endif(TARGET gflags_shared)

set_target_properties(${MAP_LIBRARIES} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release)

include_directories(${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_LIST_DIR}/depends ${CMAKE_CURRENT_SOURCE_DIR}/src ${IPP_INCLUDE_DIRS})

######################################################################
#
# Source files
#
######################################################################
add_subdirectory(depends)
add_subdirectory(src)

if(H2O_BUILD_TEST)
    add_subdirectory(tests)
endif(H2O_BUILD_TEST)

######################################################################
#
# Install Libraries
#
######################################################################
# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Use:
#   * PROJECT_VERSION
write_basic_package_version_file(
    "${version_config}" COMPATIBILITY SameMajorVersion
)

# Configure '<PROJECT-NAME>Config.cmake'
# Use variables:
#   * TARGETS_EXPORT_NAME
#   * PROJECT_NAME
configure_package_config_file(
    "Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
)

# Config
#   * <prefix>/lib/cmake/Foo/FooConfig.cmake
#   * <prefix>/lib/cmake/Foo/FooConfigVersion.cmake
install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
)

# Config
#   * <prefix>/lib/cmake/Foo/FooTargets.cmake
install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    NAMESPACE "${namespace}"
    DESTINATION "${config_install_dir}"
)

# install ruby
# add_subdirectory(ruby)

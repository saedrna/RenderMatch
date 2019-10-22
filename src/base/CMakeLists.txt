file(GLOB SOURCES
          "*.h"
          "*.hpp"
          "*.cpp")

h2o_add_library(h2o_base ${SOURCES})

target_link_libraries(h2o_base
                      PUBLIC Qt5::Widgets
                             Qt5::Core
                             Qt5::Network
                             glog::glog
                             ${OpenCV_LIBS})
target_link_libraries(h2o_base
                      PRIVATE ${Boost_LIBRARIES}
                              ${GDAL_LIBRARY}
                              TIFF::TIFF
                              ${IPP_LIBRARIES})

/*!
 * \file logging.cpp
 *
 * \author Han
 * \date 2017/03/27
 *
 *
 */
#include <base/common.h>
#include <glog/logging.h>

namespace h2o {

void initilize_glog(char **argv) {
#ifndef _MSC_VER // Broken in MSVC
    google::InstallFailureSignalHandler();
#endif
    google::InitGoogleLogging(argv[0]);
    google::LogToStderr();
}

} // namespace h2o

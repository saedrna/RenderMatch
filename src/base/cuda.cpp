/*
 * @Author: Han
 * @Date: 2018-03-06 11:19:59
 * 检测CUDA环境
 */
#include <base/cuda.h>

#ifdef ZERO_BUILD_CUDA
#include <cuda_runtime_api.h>
#endif

#include <glog/logging.h>

namespace h2o {

int cuda_get_runtime_version() {
#ifdef ZERO_BUILD_CUDA
    int version;
    cudaRuntimeGetVersion(&version);
    return version;
#endif
    return 0;
}

int cuda_get_driver_version() {
#ifdef ZERO_BUILD_CUDA
    int version;
    cudaDriverGetVersion(&version);
    return version;
#endif // ZERO_BUILD_CUDA
    return 0;
}

int cuda_get_compute_major() {
#ifdef ZERO_BUILD_CUDA
    int max_compute = 0;

    int num_devices;
    cudaGetDeviceCount(&num_devices);
    for (int i = 0; i < num_devices; i++) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, i);
        if (prop.major > max_compute)
            max_compute = prop.major;
    }

    return max_compute;
#endif // ZERO_BUILD_CUDA
    return 0;
}
int cuda_get_num_devices() {
#ifdef ZERO_BUILD_CUDA
    int num_devices;
    cudaGetDeviceCount(&num_devices);
    for (int i = 0; i < num_devices; i++) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, i);
        std::string message =
            string_printf("\n CUDA Device Number : %d\n"
                          "  Device name: %s\n"
                          "  Memory Clock Rate (KHz): %d\n"
                          "  Memory Bus Width (bits): %d\n"
                          "  Peak Memory Bandwidth (GB/s): %f\n"
                          "  Compute Capability is: %d.%d\n\n",
                          i, prop.name, prop.memoryClockRate, prop.memoryBusWidth,
                          2.0 * prop.memoryClockRate * (prop.memoryBusWidth / 8) / 1.0e6, prop.major, prop.minor);
        LOG(INFO) << message;
    }
    return num_devices;
#else
    return 0;
#endif // ZERO_BUILD_CUDA
}

void cuda_set_best_devices() {
#ifdef ZERO_BUILD_CUDA
    int num_devices;
    cudaGetDeviceCount(&num_devices);
    if (num_devices == 0) {
        return;
    }

    std::vector<double> bandwidth;
    std::vector<std::string> names;
    for (int i = 0; i < num_devices; i++) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, i);
        bandwidth.push_back(2.0 * prop.memoryClockRate * (prop.memoryBusWidth / 8) / 1.0e6);
        names.emplace_back(prop.name);
    }

    int maxid = std::distance(begin(bandwidth), std::max_element(begin(bandwidth), end(bandwidth)));
    cudaSetDevice(maxid);

    LOG(INFO) << "\nSet CUDA device to " << names[maxid];

#endif
}
} // namespace h2o

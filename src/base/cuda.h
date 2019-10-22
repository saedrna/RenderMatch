/*
 * @Author: Han
 * @Date: 2018-03-06 11:19:51
 * 检测CUDA环境
 */
#pragma once

namespace h2o {
int cuda_get_runtime_version();
int cuda_get_driver_version();
int cuda_get_compute_major();
int cuda_get_num_devices();
void cuda_set_best_devices();
}
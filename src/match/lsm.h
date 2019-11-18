/*
 * @Author: Han
 * @Date: 2019-11-18 11:25:17
 * 最小二乘匹配
 */

#pragma once

#include <base/common.h>

namespace h2o {
struct LSMSummary {
    ///\brief 初始的cost
    double initial_cost;

    ///\brief 最终的cost值
    double final_cost;

    ///\brief 记录每次迭代的值
    std::vector<std::vector<double>> param_history;

    ///\brief 最终的参数
    std::vector<double> x;

    ///\brief 迭代次数
    int iterations;

    ///\brief 是否收敛
    bool is_converge;

    ///\brief 匹配时间
    double runtime;
};

LSMSummary lsm_match(const cv::Mat &mat, const cv::Mat &search,
                     const std::vector<double> &param = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0});

} // namespace h2o

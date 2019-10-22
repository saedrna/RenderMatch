/*
 * @Author: Han
 * @Date: 2018-05-05 20:49:21
 * 相机信息
 */
#pragma once

#include <base/common.h>

namespace h2o {

class FrameCamera {
public:
    // 默认构造函数
    FrameCamera();

    // 从摄影测量模式构建相机信息
    FrameCamera(uint32_t cid, uint32_t iid, int width, int height, double delta, double f, double x0, double y0,
                const Vector3d &r, const Vector3d &X);
    ~FrameCamera();

public:
    uint32_t cid_;
    uint32_t iid_;

    Vector2i size_;
    Matrix3d K;
    Matrix3d R;
    Vector3d C;
    Matrix34d P;
    std::vector<double> distortion_;

    std::string path_;
};

using FrameCameraPtr =  std::shared_ptr<FrameCamera> ;

} // namespace h2o

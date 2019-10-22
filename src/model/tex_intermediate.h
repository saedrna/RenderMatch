/*
 * @Author: Han Hu
 * @Date: 2018-09-15 16:04:39
 * 中等模式纹理映射
 */
#pragma once

#include <modelpro/mesh.h>
#include <modelpro/mesh_bvh.h>
#include <modelpro/tex_simple.h>

namespace h2o {
class TexIntermediate : public TexSimple {
public:
    TexInformation compute(iid_t best = INVALID_INDEX) override;

protected:
    // 计算遮挡比例，每个平面，对每个影像的遮挡比例
    std::unordered_map<uint32_t, double> estimate_visibility();

public:
    MeshPtr mesh_;
    MeshEmbreeBVHPtr bvh_;

    float dnear_ = 1.0f;
    float dfar_ = 100000000.0f;
};

} // namespace h2o

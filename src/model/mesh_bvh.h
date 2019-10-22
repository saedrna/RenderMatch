/*!
 * \file mesh_bvh.h
 *
 * \author Han
 * \date 2017/04/26
 *
 * BVH 类，进行遮挡检测，采用embree
 */
#pragma once
#include <base/common.h>
#include <modelpro/mesh.h>

extern "C" {
typedef struct RTCDeviceTy *RTCDevice;
typedef struct RTCSceneTy *RTCScene;
}

namespace h2o {
struct Hit {
    uint32_t id = -1;  // primitive id
    uint32_t gid = -1; // geometry id
    float u = 0.0f;    // barycentric coordinates
    float v = 0.0f;    // barycentric coordinates
    float t = -1.0f;   // distance = direction*t to intersection
};

class MeshEmbreeBVH {
public:
    MeshEmbreeBVH();
    ~MeshEmbreeBVH();

private:
    // 禁用复制和拷贝构造函数
    MeshEmbreeBVH(const MeshEmbreeBVH &rhs){};
    MeshEmbreeBVH &operator=(const MeshEmbreeBVH &rhs){};

public:
    ///\brief 利用一个 mesh 构建一个 bvh
    void init(const GEO::Mesh &mesh);

    /**
     * \brief 获取射线和场景相交的位置信息
     * \param origin 射线原点
     * \param direction 射线方向
     * \param dnear 线段的最近点
     * \param dfar 线段的最远点
     * \param mask 不知道有什么用 embree 内部要用到
     */
    Hit ray_intersect(const Vector3d &origin, const Vector3d &direction, float dnear = 0.0f, float dfar = FLT_MAX,
                      int mask = 0xFFFFFFFF);

    /**
     * \brief 同时求取多个角点，采用stream 的方法，可能速度会快一些
     */
    std::vector<Hit> ray_intersect(const Vector3d &origin, const std::vector<Vector3d> &direction, float dnear = 0.0f,
                                   float dfar = FLT_MAX, int mask = 0xFFFFFFFF);

    /**
     * \brief 仅判断线段和场景是否相交，接口同上，速度会快一些
     */
    bool ray_intersected(const Vector3d &origin, const Vector3d &direction, float dnear = 0.0f, float dfar = FLT_MAX,
                         int mask = 0xFFFFFFFF);

    /**
     * \brief 仅判断线段和场景是否相交，接口同上，速度会快一些
     */
    std::vector<bool> ray_intersected(const Vector3d &origin, const std::vector<Vector3d> &direction,
                                      float dnear = 0.0f, float dfar = FLT_MAX, int mask = 0xFFFFFFFF);

private:
    struct Vertex {
        float x, y, z, a;
    };
    struct Triangle {
        uint32_t v0, v1, v2;
    };

protected:
    ///\brief embree scene
    RTCScene scene_;

    ///\brief 可以有多个 geometry，geometry内部的三角形是primitive
    uint32_t meshid_;

    ///\brief 是否已经初始化好
    bool initialized_;

private:
    RTCDevice device_;
};

using MeshEmbreeBVHPtr = std::shared_ptr<MeshEmbreeBVH>;

} // namespace h2o

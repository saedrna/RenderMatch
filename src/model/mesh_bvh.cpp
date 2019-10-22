/*!
 * \file mesh_bvh.cpp
 *
 * \author Han
 * \date 2017/04/26
 *
 * bvh 实现文件
 */
#include <embree3/rtcore.h>
#include <embree3/rtcore_ray.h>

#include <base/base.h>
#include <modelpro/mesh_bvh.h>

#include <geogram/mesh/mesh.h>
#include <iostream>

namespace h2o {
#define CHECK_EMBREE_ERROR(device)                                                                                     \
    {                                                                                                                  \
        auto error = rtcGetDeviceError(device);                                                                        \
        CHECK(error == RTC_ERROR_NONE) << "Error code " << error;                                                      \
    }

MeshEmbreeBVH::MeshEmbreeBVH() {
    initialized_ = false;
    device_ = 0;

    device_ = rtcNewDevice(nullptr);
    CHECK_EMBREE_ERROR(device_);
}

MeshEmbreeBVH::~MeshEmbreeBVH() {
    rtcReleaseDevice(device_);
    device_ = 0;
}

void MeshEmbreeBVH::init(const GEO::Mesh &mesh) {
    // 首先创建一个 scene

    scene_ = rtcNewScene(device_);

    // create triangle mesh geometry in that scene
    RTCGeometry rtcmesh = rtcNewGeometry(device_, RTC_GEOMETRY_TYPE_TRIANGLE);

    // fill vertex buffer
    Vertex *vertices = (Vertex *)rtcSetNewGeometryBuffer(rtcmesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3,
                                                         sizeof(Vertex), mesh.vertices.nb());
    for (int v = 0; v < mesh.vertices.nb(); ++v) {
        auto vec = mesh.vertices.point(v);
        vertices[v].x = (float)vec[0];
        vertices[v].y = (float)vec[1];
        vertices[v].z = (float)vec[2];
    }

    // fill triangle buffer
    Triangle *triangles = (Triangle *)rtcSetNewGeometryBuffer(rtcmesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
                                                              sizeof(Triangle), mesh.facets.nb());
    for (int f = 0; f < mesh.facets.nb(); ++f) {
        triangles[f].v0 = mesh.facets.vertex(f, 0);
        triangles[f].v1 = mesh.facets.vertex(f, 1);
        triangles[f].v2 = mesh.facets.vertex(f, 2);
    }

    rtcCommitGeometry(rtcmesh);
    meshid_ = rtcAttachGeometry(scene_, rtcmesh);
    rtcReleaseGeometry(rtcmesh);
    rtcCommitScene(scene_);
    CHECK_EMBREE_ERROR(device_);

    initialized_ = true;
}

Hit MeshEmbreeBVH::ray_intersect(const Vector3d &origin, const Vector3d &direction, float dnear /*= 0.0f*/,
                                 float dfar /*= FLT_MAX*/, int mask /*= 0xFFFFFFFF*/) {

    Hit hit;
    RTCRayHit rtc_rayhit;
    rtc_rayhit.ray.org_x = origin(0);
    rtc_rayhit.ray.org_y = origin(1);
    rtc_rayhit.ray.org_z = origin(2);

    rtc_rayhit.ray.dir_x = direction(0);
    rtc_rayhit.ray.dir_y = direction(1);
    rtc_rayhit.ray.dir_z = direction(2);

    rtc_rayhit.ray.tnear = dnear;
    rtc_rayhit.ray.tfar = dfar;
    rtc_rayhit.ray.mask = -1;
    rtc_rayhit.ray.time = 0.0f;
    rtc_rayhit.ray.flags = 0;
    rtc_rayhit.ray.id = 0;

    rtc_rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rtc_rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;
    rtc_rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
    rtc_rayhit.hit.u = 0.0f;
    rtc_rayhit.hit.v = 0.0f;

    // shot ray
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    rtcIntersect1(scene_, &context, &rtc_rayhit);
    CHECK_EMBREE_ERROR(device_);

    if (rtc_rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
        hit.id = rtc_rayhit.hit.primID;
        hit.gid = rtc_rayhit.hit.geomID;
        hit.u = rtc_rayhit.hit.u;
        hit.v = rtc_rayhit.hit.v;
        hit.t = rtc_rayhit.ray.tfar;
    }
    // 如果不相交的话，hit 的 geometry id 应该是 -1
    return hit;
}

std::vector<Hit> MeshEmbreeBVH::ray_intersect(const Vector3d &origin, const std::vector<Vector3d> &direction,
                                              float dnear /*= 0.0f*/, float dfar /*= FLT_MAX*/,
                                              int mask /*= 0xFFFFFFFF*/) {

    std::vector<Hit> hits(direction.size());
    std::vector<RTCRayHit> rays(direction.size());
    for (int i = 0; i < direction.size(); ++i) {
        RTCRayHit rtc_rayhit;
        rtc_rayhit.ray.org_x = origin(0);
        rtc_rayhit.ray.org_y = origin(1);
        rtc_rayhit.ray.org_z = origin(2);

        rtc_rayhit.ray.dir_x = direction[i](0);
        rtc_rayhit.ray.dir_y = direction[i](1);
        rtc_rayhit.ray.dir_z = direction[i](2);

        rtc_rayhit.ray.tnear = dnear;
        rtc_rayhit.ray.tfar = dfar;
        rtc_rayhit.ray.mask = mask;
        rtc_rayhit.ray.time = 0.0f;
        rtc_rayhit.ray.flags = 0;
        rtc_rayhit.ray.id = i;

        rtc_rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        rtc_rayhit.hit.primID = RTC_INVALID_GEOMETRY_ID;
        rtc_rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
        rtc_rayhit.hit.u = 0.0f;
        rtc_rayhit.hit.v = 0.0f;
        rays[i] = rtc_rayhit;
    }

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcIntersect1M(scene_, &context, rays.data(), rays.size(), sizeof(RTCRayHit));
    CHECK_EMBREE_ERROR(device_);

    for (int i = 0; i < rays.size(); ++i) {
        hits[i].id = rays[i].hit.primID;
        hits[i].gid = rays[i].hit.geomID;
        hits[i].u = rays[i].hit.u;
        hits[i].v = rays[i].hit.v;
        hits[i].t = rays[i].ray.tfar;
    }

    return hits;
}

bool MeshEmbreeBVH::ray_intersected(const Vector3d &origin, const Vector3d &direction, float dnear /*= 0.0f*/,
                                    float dfar /*= FLT_MAX*/, int mask /*= 0xFFFFFFFF*/) {

    RTCRayHit ray;
    ray.ray.org_x = origin(0);
    ray.ray.org_y = origin(1);
    ray.ray.org_z = origin(2);

    ray.ray.dir_x = direction(0);
    ray.ray.dir_y = direction(1);
    ray.ray.dir_z = direction(2);

    ray.ray.tnear = dnear;
    ray.ray.tfar = dfar;
    ray.ray.mask = -1;
    ray.ray.time = 0.0f;
    ray.ray.flags = 0;
    ray.ray.id = 0;

    ray.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.hit.primID = RTC_INVALID_GEOMETRY_ID;
    ray.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
    ray.hit.u = 0.0f;
    ray.hit.v = 0.0f;

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);
    rtcOccluded1(scene_, &context, &ray.ray);
    CHECK_EMBREE_ERROR(device_);

    if (ray.ray.tfar == -std::numeric_limits<float>::infinity()) {
        return true;
    } else {
        return false;
    }
}

std::vector<bool> MeshEmbreeBVH::ray_intersected(const Vector3d &origin, const std::vector<Vector3d> &direction,
                                                 float dnear /*= 0.0f*/, float dfar /*= FLT_MAX*/,
                                                 int mask /*= 0xFFFFFFFF*/) {

    std::vector<bool> hits(direction.size());
    std::vector<RTCRay> rays(direction.size());
    for (int i = 0; i < direction.size(); ++i) {
        RTCRay ray;
        ray.org_x = origin(0);
        ray.org_y = origin(1);
        ray.org_z = origin(2);

        ray.dir_x = direction[i](0);
        ray.dir_y = direction[i](1);
        ray.dir_z = direction[i](2);

        ray.tnear = dnear;
        ray.tfar = dfar;
        ray.mask = -1;
        ray.time = 0.0f;
        ray.flags = 0;
        ray.id = i;

        rays[i] = ray;
    }

    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    rtcOccluded1M(scene_, &context, rays.data(), rays.size(), sizeof(RTCRay));
    CHECK_EMBREE_ERROR(device_);

    std::vector<bool> results(rays.size(), false);
    for (int i = 0; i < rays.size(); ++i) {
        if (rays[i].tfar == -std::numeric_limits<float>::infinity()) {
            results[i] = true;
        } else {
            results[i] = false;
        }
    }

    return results;
}

} // namespace h2o

/*
 * @Author: Han Hu
 * @Date: 2018-06-21 20:57:49
 * 一些简单的几何处理
 */
#include <base/base.h>
#include <modelpro/mesh.h>

#include <geogram/basic/command_line_args.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_io.h>

namespace h2o {

void mesh_initilize() {
    GEO::initialize();
    GEO::CmdLine::import_arg_group("standard");
    GEO::CmdLine::import_arg_group("pre");
    GEO::CmdLine::import_arg_group("remesh");
    GEO::CmdLine::import_arg_group("algo");
    GEO::CmdLine::import_arg_group("post");
    GEO::CmdLine::import_arg_group("opt");
    GEO::CmdLine::import_arg_group("co3ne");
    GEO::CmdLine::import_arg_group("tet");
    GEO::CmdLine::import_arg_group("poly");

    GEO::mesh_io_initialize();
}

BoundingBox3d mesh_bbox(const GEO::Mesh &mesh) {
    BoundingBox3d bb;
    GEO::get_bbox(mesh, bb.min().data(), bb.max().data());
    return bb;
}

std::vector<Eigen::Vector3d> mesh_barycenter(const GEO::Mesh &mesh) {
    // 计算每个面的 barycenter
    std::vector<Vector3d> bcs;
    if (mesh.facets.nb()) {
        bcs.resize(mesh.facets.nb());
        for (int f = 0; f < bcs.size(); ++f) {
            auto p0 = mesh.vertices.point(mesh.facets.vertex(f, 0));
            auto p1 = mesh.vertices.point(mesh.facets.vertex(f, 1));
            auto p2 = mesh.vertices.point(mesh.facets.vertex(f, 2));

            auto bc = GEO::Geom::barycenter(p0, p1, p2);
            bcs[f] = {(double)bc[0], (double)bc[1], (double)bc[2]};
        }
    }
    return bcs;
}

MeshPtr mesh_load(const std::string &path) {
    MeshPtr mesh = std::make_shared<GEO::Mesh>();
    std::string path_fstream = string_utf8_to_fstream(path);
    GEO::mesh_load(path_fstream, *mesh);
    return mesh;
}

void mesh_transform(GEO::Mesh &M, const Matrix4d &xform) {
    using namespace GEO;
    int num_vertices = M.vertices.nb();
    for (int v = 0; v < num_vertices; ++v) {
        Vector3d point = Vector3d(M.vertices.point(v).data());
        point = (xform * point.homogeneous()).eval().hnormalized();
        M.vertices.point(v) = vec3(point.data());
    }
}

} // namespace h2o

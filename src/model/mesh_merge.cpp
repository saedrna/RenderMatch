/*
 * @Author: Han Hu
 * @Date: 2018-06-21 20:54:30
 * 合并两个mesh
 */

#include <modelpro/mesh.h>

#include <geogram/mesh/mesh.h>

namespace h2o {

void mesh_merge(GEO::Mesh &target, const GEO::Mesh &other) {
    CHECK(other.facets.are_simplices() && target.facets.are_simplices());
    int nv0 = target.vertices.nb();
    int nf0 = target.facets.nb();

    if (other.vertices.nb() > 0 && other.facets.nb() > 0) {
        // nv0 是创建了新的点之后的起始的顶点 id
        nv0 = target.vertices.create_vertices(other.vertices.nb());
        // 将 other 的顶点复制到当前 target mesh
        for (int v = 0; v < other.vertices.nb(); ++v) {
            target.vertices.point(nv0 + v) = other.vertices.point(v);
        }

        // nf0 是创建额外的 face 之后的面 id
        nf0 = target.facets.create_triangles(other.facets.nb());
        for (uint32_t f = 0; f < other.facets.nb(); ++f) {
            uint32_t v1 = nv0 + other.facets.vertex(f, 0);
            uint32_t v2 = nv0 + other.facets.vertex(f, 1);
            uint32_t v3 = nv0 + other.facets.vertex(f, 2);

            // 设置对应面的 face 的顶点坐标
            target.facets.set_vertex(nf0 + f, 0, v1);
            target.facets.set_vertex(nf0 + f, 1, v2);
            target.facets.set_vertex(nf0 + f, 2, v3);
        }
    }
}

void mesh_remove_by_bbox(GEO::Mesh &mesh, BoundingBox3d &bb) {
    auto bcs = mesh_barycenter(mesh);

    // 有待移除面，对需要移除的 会将id 设置为 -1
    GEO::vector<uint32_t> to_delete(mesh.facets.nb(), 0);
    for (uint32_t f = 0; f < mesh.facets.nb(); ++f) {
        if (bb.contains(bcs[f])) {
            to_delete[f] = GEO::NO_FACET;
        }
    }

    // 这个函数会自己删除无用点，mesh.vertices.remove_isolated
    mesh.facets.delete_elements(to_delete);
}

} // namespace h2o

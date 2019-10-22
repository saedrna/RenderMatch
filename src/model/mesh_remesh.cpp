/*
 * @Author: Han Hu
 * @Date: 2018-06-22 18:52:09
 * 采用CGAL isometric_remesh
 */

#include <modelpro/mesh.h>

#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <geogram/mesh/mesh.h>

using namespace GEO;

using Kernel = CGAL::Simple_cartesian<double>;
using Surface_mesh = CGAL::Surface_mesh<Kernel::Point_3>;
using vertex_t = Surface_mesh::Vertex_index;
using edge_t = Surface_mesh::Edge_index;
using face_t = Surface_mesh::Face_index;

namespace PMP = CGAL::Polygon_mesh_processing;

namespace h2o {

///\brief geogram 转换为 cgal 的 mesh
Surface_mesh geo_to_cgal(const GEO::Mesh &M) {
    Surface_mesh m;
    int num_points = M.vertices.nb();
    for (uint32_t i = 0; i < num_points; ++i) {
        auto p = M.vertices.point(i);
        m.add_vertex({p[0], p[1], p[2]});
    }

    int num_faces = M.facets.nb();
    for (uint32_t f = 0; f < num_faces; ++f) {
        vertex_t v1 = (vertex_t)M.facets.vertex(f, 0);
        vertex_t v2 = (vertex_t)M.facets.vertex(f, 1);
        vertex_t v3 = (vertex_t)M.facets.vertex(f, 2);

        m.add_face(v1, v2, v3);
    }
    return m;
}

///\brief cgal 转换为 geogram 的mesh
void cgal_to_geo(const Surface_mesh &m, GEO::Mesh &M) {
    // false - 不保留attribute信息
    M.clear(false);

    int num_vertex = m.number_of_vertices();
    M.vertices.create_vertices(num_vertex);
    for (auto v : vertices(m)) {
        auto p = m.point(v);
        M.vertices.point(uint32_t(v)) = {p.x(), p.y(), p.z()};
    }

    int num_faces = m.number_of_faces();
    M.facets.create_triangles(num_faces);
    for (auto f : faces(m)) {
        // vertices_around_face 返回对应面的顶点 id
        auto vertices = CGAL::vertices_around_face(m.halfedge(f), m);
        CHECK(vertices.size() == 3);
        std::vector<uint32_t> vs(vertices.begin(), vertices.end());

        M.facets.set_vertex((uint32_t)f, 0, vs[0]);
        M.facets.set_vertex((uint32_t)f, 1, vs[1]);
        M.facets.set_vertex((uint32_t)f, 2, vs[2]);
    }
    M.facets.connect();
}

void mesh_remesh(const GEO::Mesh &M, GEO::Mesh &M_out, double target_length, const std::vector<uint32_t> &edges_fix,
                 int iter) {
    // 将 edge 的 id 转换为对应顶点的 pair
    std::set<std::pair<geo_index_t, geo_index_t>> vertex_pairs;
    for (auto e : edges_fix) {
        geo_index_t v1 = M.edges.vertex(e, 0);
        geo_index_t v2 = M.edges.vertex(e, 1);

        vertex_pairs.emplace(v1, v2);
    }
    mesh_remesh(M, M_out, target_length, vertex_pairs, iter);
}

void mesh_remesh(const GEO::Mesh &M, GEO::Mesh &M_out, double target_length,
                 const std::set<std::pair<uint32_t, uint32_t>> &edges_fix, int iter /*= 1*/) {
    Surface_mesh mesh = geo_to_cgal(M);

    auto &constrained_edges = mesh.add_property_map<edge_t, bool>("e:is_constrained", false).first;
    std::vector<edge_t> edges;
    // 遍历所有的边，如果边在 fix 的 set 里面，将他的属性设为 true
    for (auto e : mesh.edges()) {
        geo_index_t v1 = (geo_index_t)mesh.vertex(e, 0);
        geo_index_t v2 = (geo_index_t)mesh.vertex(e, 1);
        auto pair1 = std::make_pair(v1, v2);
        auto pair2 = std::make_pair(v2, v1);
        if (edges_fix.count(pair1) != 0 || edges_fix.count(pair2) != 0) {
            constrained_edges[e] = true;
        }

        edges.push_back(e);
    }

    PMP::split_long_edges(edges, target_length, mesh);
    constrained_edges = mesh.property_map<edge_t, bool>("e:is_constrained").first;

    // 采用 isometric_remeshing 算法重构 mesh
    PMP::isotropic_remeshing(faces(mesh), target_length, mesh,
                             PMP::parameters::number_of_iterations(iter)
                                 .edge_is_constrained_map(constrained_edges)
                                 .protect_constraints(true));

    // 如果不 collect garbage，会在转换中出现bug
    mesh.collect_garbage();

    cgal_to_geo(mesh, M_out);
}

} // namespace h2o

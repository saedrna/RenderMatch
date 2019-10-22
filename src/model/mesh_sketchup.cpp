/*
 * @Author: Han Hu
 * @Date: 2018-06-21 20:25:17
 * 从Sketchup实体中提取三维模型
 */

#include <modelpro/mesh.h>
#include <sketchup/sketchup.h>

#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_repair.h>

namespace h2o {
MeshPtr mesh_extract(VALUE enumerable, const BoundingBox3d &bounds) {
    MeshPtr mesh = std::make_shared<GEO::Mesh>();
    std::vector<VALUE> rfaces = su_enumerable_faces_nocomponent(enumerable);
    if (rfaces.empty()) {
        return nullptr;
    }

    std::vector<MatrixXd> vertices(rfaces.size());
    std::vector<MatrixXu> faces(rfaces.size());
    int num_vertices = 0;
    int num_faces = 0;

    int index_vertex = 0;
    int index_facet = 0;
    for (int i = 0; i < rfaces.size(); ++i) {
        MatrixXd V;
        MatrixXu F;
        std::tie(V, F) = su_face_mesh(rfaces[i]);

        // 增加顶点
        mesh->vertices.create_vertices(V.cols());
        for (int v = 0; v < V.cols(); ++v) {
            mesh->vertices.point(v + index_vertex) = GEO::vec3(V(0, v), V(1, v), V(2, v));
        }

        // 增加 face
        mesh->facets.create_triangles(F.cols());
        for (int f = 0; f < F.cols(); ++f) {
            mesh->facets.set_vertex(f + index_facet, 0, F(0, f) + index_vertex);
            mesh->facets.set_vertex(f + index_facet, 1, F(1, f) + index_vertex);
            mesh->facets.set_vertex(f + index_facet, 2, F(2, f) + index_vertex);
        }

        index_vertex += V.cols();
        index_facet += F.cols();
    }

    // 修复，并合并顶点
    mesh_repair(*mesh, GEO::MESH_REPAIR_DEFAULT, EPSILON_LENGTH);
    return mesh;
}

MeshPtr mesh_extract(const MatrixXd &V, const MatrixXu &F) {
    if (V.cols() == 0 || F.cols() == 0) {
        return nullptr;
    }

    MeshPtr mesh = std::make_shared<GEO::Mesh>();
    // 增加顶点
    mesh->vertices.create_vertices(V.cols());
    for (int v = 0; v < V.cols(); ++v) {
        mesh->vertices.point(v) = GEO::vec3(V(0, v), V(1, v), V(2, v));
    }

    // 增加 face
    mesh->facets.create_triangles(F.cols());
    for (int f = 0; f < F.cols(); ++f) {
        mesh->facets.set_vertex(f, 0, F(0, f));
        mesh->facets.set_vertex(f, 1, F(1, f));
        mesh->facets.set_vertex(f, 2, F(2, f));
    }
    return mesh;
}

} // namespace h2o

/*!
 * \file mesh_subdivision.cpp
 *
 * \author Han
 * \date 2017/04/25
 *
 * 采用简单的边的中点分割方法进行 mesh 的分割
 */
#include <modelpro/mesh.h>

#include <geogram/mesh/mesh_geometry.h>
#include <glog/logging.h>
namespace h2o {

using namespace GEO;

void mesh_subdivide_iter(Mesh &M, const std::vector<uint32_t> &faces) {
    uint32_t nv0 = M.vertices.nb();
    uint32_t nf0 = M.facets.nb();

    // Compute corner to new vertex mapping
    vector<uint32_t> ctov(M.facet_corners.nb(), NO_VERTEX);
    uint32_t nbnewv = 0;
    for (auto f : faces) {
        for (uint32_t c = M.facets.corners_begin(f); c < M.facets.corners_end(f); ++c) {
            if (ctov[c] == uint32_t(-1)) {
                ctov[c] = nbnewv;
                uint32_t f2 = M.facet_corners.adjacent_facet(c);
                if (f2 != NO_FACET) {
                    for (uint32_t c2 = M.facets.corners_begin(f2); c2 != M.facets.corners_end(f2); ++c2) {
                        if (M.facet_corners.adjacent_facet(c2) == f) {
                            ctov[c2] = nbnewv;
                            break;
                        }
                    }
                }
                ++nbnewv;
            }
        }
    }

    // Create vertices
    M.vertices.create_vertices(nbnewv);
    for (auto f : faces) {
        for (uint32_t c1 = M.facets.corners_begin(f); c1 < M.facets.corners_end(f); ++c1) {
            uint32_t c2 = M.facets.next_corner_around_facet(f, c1);
            uint32_t v1 = M.facet_corners.vertex(c1);
            uint32_t v2 = M.facet_corners.vertex(c2);
            uint32_t v12 = ctov[c1] + nv0;
            const double *p1 = M.vertices.point_ptr(v1);
            const double *p2 = M.vertices.point_ptr(v2);
            double *p12 = M.vertices.point_ptr(v12);
            for (uint32_t coord = 0; coord < M.vertices.dimension(); ++coord) {
                p12[coord] = 0.5 * (p1[coord] + p2[coord]);
            }
        }
    }

    // Create facets
    M.facets.create_triangles(3 * faces.size());
    uint32_t newf = 0;
    for (auto f : faces) {
        uint32_t v1 = M.facets.vertex(f, 0);
        uint32_t v2 = M.facets.vertex(f, 1);
        uint32_t v3 = M.facets.vertex(f, 2);
        uint32_t v12 = ctov[M.facets.corners_begin(f)] + nv0;
        uint32_t v23 = ctov[M.facets.corners_begin(f) + 1] + nv0;
        uint32_t v31 = ctov[M.facets.corners_begin(f) + 2] + nv0;
        M.facets.set_vertex(f, 0, v31);
        M.facets.set_vertex(f, 1, v12);
        M.facets.set_vertex(f, 2, v23);
        M.facets.set_vertex(nf0 + 3 * newf, 0, v1);
        M.facets.set_vertex(nf0 + 3 * newf, 1, v12);
        M.facets.set_vertex(nf0 + 3 * newf, 2, v31);
        M.facets.set_vertex(nf0 + 3 * newf + 1, 0, v12);
        M.facets.set_vertex(nf0 + 3 * newf + 1, 1, v2);
        M.facets.set_vertex(nf0 + 3 * newf + 1, 2, v23);
        M.facets.set_vertex(nf0 + 3 * newf + 2, 0, v31);
        M.facets.set_vertex(nf0 + 3 * newf + 2, 1, v23);
        M.facets.set_vertex(nf0 + 3 * newf + 2, 2, v3);
        newf++;
    }
    M.facets.connect();
}

void mesh_subdivision(Mesh &M, double max_length, int max_iter /*= 5*/) {
    CHECK(M.facets.are_simplices());
    double max_area = 0.5 * max_length * max_length;

    for (int i = 0; i < max_iter; ++i) {
        int nv0 = M.vertices.nb();
        int nf0 = M.facets.nb();

        std::vector<uint32_t> large_facet;
        large_facet.reserve(nf0);
        // 计算存在长边的面
        for (uint32_t f = 0; f < nf0; ++f) {
            double area = Geom::mesh_facet_area(M, f);
            if (area > max_area) {
                large_facet.push_back(f);
            }
        }
        if (large_facet.size() == 0) {
            return;
        } else {
            mesh_subdivide_iter(M, large_facet);
        }
    }
}

} // namespace h2o

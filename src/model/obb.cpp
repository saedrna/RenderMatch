/*
 * @Author: Han Hu
 * @Date: 2018-08-03 16:29:33
 * 封装 MathGeoLib 的 OBB 类
 */

#include <MathGeoLib/MathGeoLib.h>

#include <modelpro/obb.h>
#include <modelpro/plane.h>
#include <sketchup/sketchup.h>

namespace h2o {
OBB3d to_h2o(const OBB &obb) {
    OBB3d obb2;
    Vector3d center(obb.pos.x, obb.pos.y, obb.pos.z);
    Vector3d r(obb.r.x, obb.r.y, obb.r.z);
    Vector3d axis0(obb.axis[0].x, obb.axis[0].y, obb.axis[0].z);
    Vector3d axis1(obb.axis[1].x, obb.axis[1].y, obb.axis[1].z);
    Vector3d axis2(obb.axis[2].x, obb.axis[2].y, obb.axis[2].z);

    obb2.center = center;
    obb2.r = r;
    obb2.axis[0] = axis0;
    obb2.axis[1] = axis1;
    obb2.axis[2] = axis2;

    return obb2;
}

OBB to_mathgeolib(const OBB3d &obb) {
    vec pos = vec(obb.center.x(), obb.center.y(), obb.center.z());
    vec r = vec(obb.r.x(), obb.r.y(), obb.r.z());
    vec axis0 = vec(obb.axis[0].x(), obb.axis[0].y(), obb.axis[0].z());
    vec axis1 = vec(obb.axis[1].x(), obb.axis[1].y(), obb.axis[1].z());
    vec axis2 = vec(obb.axis[2].x(), obb.axis[2].y(), obb.axis[2].z());

    OBB obb2(pos, r, axis0, axis1, axis2);
    return obb2;
}

OBB3d obb_compute(const std::vector<Vector3d> &points) {
    std::vector<vec> points2(points.size());
    for (int i = 0; i < points.size(); ++i) {
        points2[i].x = points[i].x();
        points2[i].y = points[i].y();
        points2[i].z = points[i].z();
    }

    OBB obb = OBB::OptimalEnclosingOBB(points2.data(), points2.size());
    if (obb.IsDegenerate()) {
        obb = OBB::BruteEnclosingOBB(points2.data(), points2.size());
    }
    return to_h2o(obb);
}

OBB3d obb_compute(const std::vector<Vector3d> &points, const Vector3d &dir1, const Vector3d &dir2) {
    std::vector<vec> points2(points.size());
    for (int i = 0; i < points.size(); ++i) {
        points2[i].x = points[i].x();
        points2[i].y = points[i].y();
        points2[i].z = points[i].z();
    }

    vec d1(dir1.x(), dir1.y(), dir1.z());
    vec d2(dir2.x(), dir2.y(), dir2.z());

    OBB obb = OBB::FixedOrientationEnclosingOBB(points2.data(), points2.size(), d1, d2);
    return to_h2o(obb);
}

OBB3d obb_compute(const BoundingBox3d &aabb) {
    std::vector<Vector3d> points(8);
    for (int i = 0; i < 8; ++i) {
        points[i] = aabb.corner(BoundingBox3d::CornerType(i));
    }

    Vector3d dir1 = Vector3d(1, 0, 0);
    Vector3d dir2 = Vector3d(0, 1, 0);

    return obb_compute(points, dir1, dir2);
}

OBB3d obb_compute(VALUE face, double thickness) {
    PlaneRotated plane = plane_from_face(face);
    return plane.get_obb(thickness);
}

OBB3d obb_scale(const OBB3d &obb, double scale) {
    OBB obb2 = to_mathgeolib(obb);
    obb2.Scale(obb2.pos, scale);
    return to_h2o(obb2);
}

OBB3d obb_scale(const OBB3d &obb, const Vector3d &scale) {
    OBB obb2 = to_mathgeolib(obb);
    vec scale2(scale(0), scale(1), scale(2));
    obb2.Scale(obb2.pos, scale2);
    return to_h2o(obb2);
}

OBB3d obb_scale(const OBB3d &obb, double scale, const Vector3d &center) {
    vec center2(center(0), center(1), center(2));
    OBB obb2 = to_mathgeolib(obb);
    obb2.Scale(center2, scale);
    return to_h2o(obb2);
}

OBB3d obb_scale(const OBB3d &obb, const Vector3d &scale, const Vector3d &center) {
    vec center2(center(0), center(1), center(2));
    OBB obb2 = to_mathgeolib(obb);
    vec scale2(scale(0), scale(1), scale(2));
    obb2.Scale(center2, scale2);
    return to_h2o(obb2);
}

double obb_volume(const OBB3d &obb) {
    OBB obb2 = to_mathgeolib(obb);
    return obb2.Volume();
}

MatrixXd obb_triangulate(const OBB3d &obb, MatrixXd *normal) {
    OBB obb2 = to_mathgeolib(obb);

    int num_vertices = OBB::NumVerticesInTriangulation(1, 1, 1);

    std::vector<vec> vertices(num_vertices), normals(normal == nullptr ? 0 : num_vertices);
    obb2.Triangulate(1, 1, 1, vertices.data(), normal ? normals.data() : nullptr, nullptr, true);

    MatrixXd V(3, num_vertices);
    for (int i = 0; i < V.cols(); ++i) {
        V(0, i) = vertices[i].x;
        V(1, i) = vertices[i].y;
        V(2, i) = vertices[i].z;

        if (normal) {
            normal->coeffRef(0, i) = normals[i].x;
            normal->coeffRef(1, i) = normals[i].y;
            normal->coeffRef(2, i) = normals[i].z;
        }
    }

    return V;
}
MatrixXd obb_edges(const OBB3d &obb) {
    OBB obb2 = to_mathgeolib(obb);
    MatrixXd E(3, 24); // 12条线段
    for (int i = 0; i < 12; ++i) {
        LineSegment seg = obb2.Edge(i);
        E(0, 2 * i + 0) = seg.a.x;
        E(1, 2 * i + 0) = seg.a.y;
        E(2, 2 * i + 0) = seg.a.z;

        E(0, 2 * i + 1) = seg.b.x;
        E(1, 2 * i + 1) = seg.b.y;
        E(2, 2 * i + 1) = seg.b.z;
    }
    return E;
}
} // namespace h2o

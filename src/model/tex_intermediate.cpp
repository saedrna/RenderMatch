/*
 * @Author: Han Hu
 * @Date: 2018-09-15 16:04:51
 * 中等模式纹理映射
 */
#include <base/base.h>
#include <modelpro/mesh_bvh.h>
#include <modelpro/tex_intermediate.h>

#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_geometry.h>

namespace h2o {
TexInformation TexIntermediate::compute(iid_t best_view /*= INVALID_INDEX*/) {
    TexInformation tex;

    if (best_view == INVALID_INDEX) {
        // 首先计算最佳影像
        std::unordered_map<uint32_t, std::tuple<double, double, double>> view_info;

        // view_id => [投影到影像的面积，影像视角和平面夹角]
        std::unordered_map<uint32_t, double> view_area, view_angle, view_vis;

        view_angle = estimate_angle();
        view_area = estimate_area();
        view_vis = estimate_visibility();

        for (const auto &area : view_area) {
            iid_t view = area.first;

            // view info 只记录在 area 和 angle 都获得了值的影像
            if (view_angle.count(view) && view_vis.count(view)) {
                view_info[view] = std::make_tuple(area.second, view_angle.at(view), view_vis.at(view));
            }
        }

        if (!view_info.size()) {
            return TexInformation();
        }

        // 归一化值, 0 - area ; 1- angle
        BoundingBox2d minmax;
        for (const auto &info : view_info) {
            double area, angle, vis;
            std::tie(area, angle, vis) = info.second;
            minmax.extend(Vector2d(area, angle));
        }

        // 计算score
        std::vector<std::pair<uint32_t, double>> score;
        for (const auto &info : view_info) {
            double area, angle, vis;
            std::tie(area, angle, vis) = info.second;
            // area 是越大 socre 越高
            area = (area - minmax.min()(0)) / (minmax.max()(0) - minmax.min()(0));
            // angle 是越小score 越高
            angle = (minmax.max()(1) - angle) / (minmax.max()(1) - minmax.min()(1));

            double score_weighted = (1.0 - weight_angle_) * area + weight_angle_ * angle;
            score.emplace_back(info.first, score_weighted * vis);
        }

        std::sort(begin(score), end(score), [&](const auto &p1, const auto &p2) { return p1.second > p2.second; });
        best_view = score[0].first;
    }

    if (cameras_->count(best_view) == 0 || disk_images_->count(best_view) == 0) {
        return TexInformation();
    }

    // 纹理映射
    {
        FrameCamera *camera = cameras_->at(best_view).get();
        DiskImage *image = disk_images_->at(best_view).get();

        PlaneRotated plane = plane_;

        std::vector<Vector3d> corners3d = plane.get_corners3d();
        std::vector<Vector2d> corners2d = plane.get_corners2d();
        // 把包围和的点投影到影像上
        std::vector<Vector2d> image2d = project(*camera, corners3d);
        double gsd = estimate_gsd(corners2d, image2d);
        plane.set_gsd(gsd);

        // 获取纹理影像坐标和纹理

        std::vector<Vector2d> mat_points;
        cv::Mat mat;
        std::tie(mat_points, mat) = subset_texture_by_plane(plane, camera, image);
        if (mat_points.empty()) {
            return TexInformation();
        }

        // 纹理映射之前，需要把坐标变回局部坐标
        Matrix4d face_xform = xform_.inverse().eval();
        for (Vector3d &point : corners3d) {
            point = (face_xform * point.homogeneous()).hnormalized();
        }

        TexInformation info;
        info.mat = mat;
        info.mat_points = mat_points;
        info.obj_points = corners3d;
        info.view_id_ = best_view;
        return info;
    }

    return tex;
}

std::unordered_map<uint32_t, double> TexIntermediate::estimate_visibility() {
    std::unordered_map<uint32_t, double> visareas;

    int num_facets = mesh_->facets.nb();
    std::vector<double> face_areas(num_facets, 0.0);
    double sum_areas = 0.0;

    // 计算所有相机ID
    std::vector<uint32_t> camears_indices;
    for (const auto &pair : *cameras_) {
        iid_t iid = pair.first;
        camears_indices.push_back(iid);
        visareas[iid] = 0.0;
    }

    // 计算总面积
    for (int f = 0; f < num_facets; ++f) {
        double area = GEO::Geom::mesh_facet_area(*mesh_, f);
        sum_areas += area;
        face_areas[f] = area;
    }

    // 对每个 face， 计算到所有相机的遮挡关系，如果没遮挡，把他的面积加到 visarea中
    for (int f = 0; f < num_facets; ++f) {
        double area = face_areas[f];
        GEO::vec3 temp = GEO::Geom::mesh_facet_center(*mesh_, f);
        Vector3d bc(temp.x, temp.y, temp.z);

        std::vector<Vector3d> directions(cameras_->size());
        for (int c = 0; c < camears_indices.size(); ++c) {
            Vector3d C = cameras_->at(camears_indices[c])->C;
            Vector3d direction = C - bc;
            direction.normalize();
            directions[c] = direction;
        }
        std::vector<bool> results = bvh_->ray_intersected(bc, directions, dnear_, dfar_);
        for (int c = 0; c < camears_indices.size(); ++c) {
            iid_t iid = camears_indices[c];
            visareas[iid] += results[iid] ? area : 0.0;
        }
    }

    for (uint32_t iid : camears_indices) {
        visareas[iid] /= sum_areas;
    }

    return visareas;
}

} // namespace h2o

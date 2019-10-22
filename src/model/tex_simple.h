/*
 * @Author: Han Hu
 * @Date: 2018-09-01 19:24:33
 * 仅顾及角度和面积的纹理映射
 */
#pragma once

#include <base/base.h>
#include <modelpro/plane.h>

namespace h2o {
struct TexInformation {
    // 物方点，FACE 的局部坐标系
    std::vector<Vector3d> obj_points;
    // 像素值
    std::vector<Vector2d> mat_points;
    // 纹理影像
    cv::Mat mat;

    // 默认是无效值
    iid_t view_id_ = INVALID_INDEX;
};
// 输入一个平面，计算并裁剪下来这个平面最佳的投影影像
class TexSimple {
public:
    virtual TexInformation compute(iid_t best = INVALID_INDEX);

protected:
    virtual std::unordered_map<iid_t, double> estimate_angle();
    virtual std::unordered_map<iid_t, double> estimate_area();
    // 从影像中裁剪出当前的纹理影像，并且计算四个角点信息，影像已经纠正过了
    virtual std::tuple<std::vector<Vector2d>, cv::Mat>
    subset_texture_by_plane(const PlaneRotated &plane, const FrameCamera *camera, DiskImage *image);

public:
    // SU Face 生成的 平面
    PlaneRotated plane_;
    // SU Face 的变换，最后用来做纹理映射，需要把顶点变换回局部坐标
    Matrix4d xform_;

    // 最大角度阈值
    double max_angle_;
    // 面积和角度权重
    double weight_angle_;
    // 顾及畸变的黑边
    int black_border_;

    std::unordered_map<iid_t, FrameCameraPtr> *cameras_;
    std::unordered_map<iid_t, DiskImagePtr> *disk_images_;
};
} // namespace h2o

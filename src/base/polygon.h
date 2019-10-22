/*
 * @Author: Han Hu
 * @Date: 2019-01-23 16:24:23
 * 多边形类
 */
#pragma once

#include <base/clipper.hpp>
#include <base/common.h>

#include <QPainterPath>
namespace h2o {
using Path = ClipperLib::Path;
using Paths = ClipperLib::Paths;
using IntPoint = ClipperLib::IntPoint;

using PolygonRing = std::vector<Vector2f>;
using PolygonRings = std::vector<PolygonRing>;

int64_t constexpr POLYGON_SCALE = 1000;

class PolygonClipper {
public:
    PolygonClipper(){};

    bool empty() const;
    void init(const PolygonRing &ring);

    void clear();
    void clean();
    QPainterPath create_qpath() const;
    PolygonRings create_rings() const;

    PolygonClipper simplify(double distance) const;
    bool in_polygon(const Vector2f &point) const;
    BoundingBox2f get_bounds() const;

public:
    // 采用 even odd 的填充方法
    Paths paths_;
};

} // namespace h2o

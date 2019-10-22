/*
 * @Author: Han Hu
 * @Date: 2019-01-23 16:24:34
 * 多边形类
 */
#include <base/polygon.h>

#include <QPainterPath>
#include <QPolygon>

namespace h2o {
bool PolygonClipper::empty() const { return paths_.empty(); }
void PolygonClipper::init(const PolygonRing &ring) {
    Path p(ring.size());
    for (int i = 0; i < ring.size(); ++i) {
        p[i].X = llround((double)ring[i].x() * POLYGON_SCALE);
        p[i].Y = llround((double)ring[i].y() * POLYGON_SCALE);
    }
    paths_.clear();
    paths_.push_back(p);
}

void PolygonClipper::clear() { paths_.clear(); }
void PolygonClipper::clean() { ClipperLib::SimplifyPolygons(paths_); }
QPainterPath PolygonClipper::create_qpath() const {
    QPainterPath qpath;

    for (const auto &path : paths_) {
        QVector<QPointF> points(path.size());
        for (int i = 0; i < path.size(); ++i) {
            points[i].setX((double)path[i].X / POLYGON_SCALE);
            points[i].setY((double)path[i].Y / POLYGON_SCALE);
        }

        QPolygonF polygon(points);
        qpath.addPolygon(polygon);
    }
    qpath.setFillRule(Qt::OddEvenFill);
    return qpath;
}

PolygonRings PolygonClipper::create_rings() const {
    PolygonRings rings;
    rings.reserve(paths_.size());

    for (const auto &path : paths_) {
        PolygonRing ring(path.size());
        for (int i = 0; i < path.size(); ++i) {
            ring[i].x() = (double)path[i].X / POLYGON_SCALE;
            ring[i].y() = (double)path[i].Y / POLYGON_SCALE;
        }

        rings.push_back(ring);
    }
    return rings;
}

bool PolygonClipper::in_polygon(const Vector2f &point) const {
    // 只有第一个是 outer 其他全部是 hole
    bool first = true;
    IntPoint ipoint;
    ipoint.X = point.x() * POLYGON_SCALE;
    ipoint.Y = point.y() * POLYGON_SCALE;

    for (const auto &path : paths_) {
        int inside = ClipperLib::PointInPolygon(ipoint, path);
        if (first && inside != 1) {
            return false;
        } else if (!first && inside == 1) {
            return false;
        }
    }

    return true;
}

BoundingBox2f PolygonClipper::get_bounds() const {
    BoundingBox2f bounds;
    for (const auto &path : paths_) {
        for (const auto &p : path) {
            Vector2f point;
            point.x() = (double)p.X / POLYGON_SCALE;
            point.y() = (double)p.Y / POLYGON_SCALE;
            bounds.extend(point);
        }
    }
    return bounds;
}

} // namespace h2o
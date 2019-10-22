/*
 * @Author: Han Hu
 * @Date: 2019-01-23 19:44:08
 * 简化多边形
 */

#include <base/polygon.h>

#include <CGAL/Polygon_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Simple_cartesian.h>

namespace h2o {
PolygonClipper PolygonClipper::simplify(double distance) const {
    using K = CGAL::Simple_cartesian<double>;
    using Polygon_2 = CGAL::Polygon_2<K>;
    using Point_2 = CGAL::Point_2<K>;
    namespace PS = CGAL::Polyline_simplification_2;
    using Cost = PS::Squared_distance_cost;

    PolygonClipper other;

    for (const auto &path : paths_) {
        std::vector<Point_2> points(path.size());
        for (int i = 0; i < points.size(); ++i) {
            points[i] = Point_2(path[i].X / POLYGON_SCALE, path[i].Y / POLYGON_SCALE);
        }
        Polygon_2 polygon(begin(points), end(points));
        polygon = PS::simplify(polygon, Cost(),
                               [&](const auto &cdt, auto vertex, auto cost, size_t initial_cound,
                                   size_t count) -> bool { return cost > distance * distance; });
        Path path2(polygon.size());
        for (int i = 0; i < path2.size(); ++i) {
            path2[i].X = polygon[i].x() * POLYGON_SCALE;
            path2[i].Y = polygon[i].y() * POLYGON_SCALE;
        }
        other.paths_.push_back(path2);
    }

    return other;
}

} // namespace h2o

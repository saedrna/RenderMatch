/*
 * @Author: Han
 * @Date: 2019-11-15 13:57:55
 * Calculate the normal vectors as point clouds
 */

#define CGAL_LINKED_WITH_TBB

#include <CGAL/Simple_cartesian.h>
#include <CGAL/pca_estimate_normals.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <base/base.h>

using namespace h2o;

cv::Mat pcd_normal(const cv::Mat &mat_xyz, const Vector2i &size) {
    using Kernel = CGAL::Simple_cartesian<float>;
    using Point_3 = Kernel::Point_3;
    using Vector_3 = Kernel::Vector_3;
    using Parallel_tag = CGAL::Parallel_tag;
    using Pwn = std::pair<Point_3, Vector_3>;

    cv::Mat mat_xyz_d;
    if (size.x() > 0 && size.y() > 0) {
        cv::resize(mat_xyz, mat_xyz_d, cv::Size(size.x(), size.y()), 0.0, 0.0, cv::INTER_NEAREST);
    } else {
        mat_xyz_d = mat_xyz;
    }

    int width = mat_xyz_d.cols;
    int height = mat_xyz_d.rows;

    std::vector<Pwn> pwn;
    std::vector<Vector2w> indices;
    pwn.reserve(width * height);
    indices.reserve(width * height);

    int K = 12;

    for (int r = 0; r < height; ++r) {
        cv::Point3f *ptr_xyz = mat_xyz_d.ptr<cv::Point3f>(r);
        for (int c = 0; c < width; ++c) {
            cv::Point3f point = ptr_xyz[c];
            if (point.x == FLT_MAX) {
                continue;
            }

            Vector_3 n;
            Point_3 p(point.x, point.y, point.z);
            pwn.push_back(std::make_pair(p, n));
            indices.push_back(Vector2w(c, r));
        }
    }

    CGAL::pca_estimate_normals<Parallel_tag>(pwn, K,
                                             CGAL::parameters::point_map(CGAL::First_of_pair_property_map<Pwn>())
                                                 .normal_map(CGAL::Second_of_pair_property_map<Pwn>()));

    // ¥Ê»Î map
    cv::Mat mat_nor = cv::Mat::zeros(height, width, CV_32FC3);
#pragma omp parallel for
    for (int i = 0; i < pwn.size(); ++i) {
        Vector3f p(pwn[i].first[0], pwn[i].first[1], pwn[i].first[2]);
        Vector3f n(pwn[i].second[0], pwn[i].second[1], pwn[i].second[2]);

        int c = indices[i].x();
        int r = indices[i].y();

        mat_nor.at<cv::Vec3f>(r, c) = cv::Vec3f(n.data());
    }

    cv::resize(mat_nor, mat_nor, cv::Size(mat_xyz.cols, mat_xyz.rows), 0.0, 0.0, cv::INTER_NEAREST);

    return mat_nor;
}

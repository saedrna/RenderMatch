/*
 * @Author: Han
 * @Date: 2019-11-18 11:25:17
 * 最小二乘匹配
 */
#include "lsm.h"

#include <base/base.h>
#include <ceres/ceres.h>
#include <opencv2/imgproc.hpp>

using namespace ceres;
namespace ceres {

// A jet traits class to make it easier to work with mixed auto / numeric diff.
template <typename T> struct JetOps {
    static bool IsScalar() { return true; }
    static T GetScalar(const T &t) { return t; }
    static void SetScalar(const T &scalar, T *t) { *t = scalar; }
    static void ScaleDerivative(double scale_by, T *value) {
        // For double, there is no derivative to scale.
        (void)scale_by; // Ignored.
        (void)value;    // Ignored.
    }
};

template <typename T, int N> struct JetOps<Jet<T, N>> {
    static bool IsScalar() { return false; }
    static T GetScalar(const Jet<T, N> &t) { return t.a; }
    static void SetScalar(const T &scalar, Jet<T, N> *t) { t->a = scalar; }
    static void ScaleDerivative(double scale_by, Jet<T, N> *value) { value->v *= scale_by; }
};

template <typename FunctionType, int kNumArgs, typename ArgumentType> struct Chain {
    static ArgumentType Rule(const FunctionType &f, const FunctionType dfdx[kNumArgs], const ArgumentType x[kNumArgs]) {
        // In the default case of scalars, there's nothing to do since there are
        // no
        // derivatives to propagate.
        (void)dfdx; // Ignored.
        (void)x;    // Ignored.
        return f;
    }
};

//这里实现的过程是，给定f和dfdx几个值，然后手动的构建ceres::Jet<T,N>
// jet.a是scalar部分，jet.v是一次项部分，即
// f = f0 + dfdx1 * dx1 + dfdx2 * dx2 + ...
// f = f0 + dfdx * dx
//考虑到x应该也可能是变量z的函数，所以还要考虑到dxdz，然后再相乘
// f = f0 + dfdx * dxdz * dz
template <typename FunctionType, int kNumArgs, typename T, int N> struct Chain<FunctionType, kNumArgs, Jet<T, N>> {
    static Jet<T, N> Rule(const FunctionType &f, const FunctionType dfdx[kNumArgs], const Jet<T, N> x[kNumArgs]) {
        // x is itself a function of another variable ("z"); what this function
        // needs to return is "f", but with the derivative with respect to z
        // attached to the jet. So combine the derivative part of x's jets to
        // form
        // a Jacobian matrix between x and z (i.e. dx/dz).
        Eigen::Matrix<T, kNumArgs, N> dxdz;
        for (int i = 0; i < kNumArgs; ++i) {
            dxdz.row(i) = x[i].v.transpose();
        }

        // Map the input gradient dfdx into an Eigen row vector.
        Eigen::Map<const Eigen::Matrix<FunctionType, 1, kNumArgs>> vector_dfdx(dfdx, 1, kNumArgs);

        // Now apply the chain rule to obtain df/dz. Combine the derivative with
        // the scalar part to obtain f with full derivative information.
        Jet<T, N> jet_f;
        jet_f.a = f;
        jet_f.v = vector_dfdx.template cast<T>() * dxdz; // Also known as dfdz.
        return jet_f;
    }
};

} // namespace ceres

namespace h2o {

static bool warpAffine(const cv::Mat &src, cv::Mat &dst, const cv::Size &size, const std::vector<double> &param) {
    //这里值对第一个波段的影像进行重采样
    dst = cv::Mat::zeros(size, src.type());

    cv::Mat mapx(size, CV_32F);
    cv::Mat mapy(size, CV_32F);
    float *mapx_data = (float *)mapx.data;
    float *mapy_data = (float *)mapy.data;

    int half_lsm = size.width / 2;
    int window_size = size.width;
    int g_half_cols = src.cols / 2;
    int g_half_rows = src.rows / 2;

    double a11(param[0]), a12(param[1]), a13(param[2]), a21(param[3]), a22(param[4]), a23(param[5]);

    for (int r = -half_lsm; r <= half_lsm; ++r) {
        int r_step = (r + half_lsm) * window_size;
        for (int c = -half_lsm; c <= half_lsm; ++c) {
            *mapx_data++ = a11 * c + a12 * r + a13 + g_half_cols;
            *mapy_data++ = a21 * c + a22 * r + a23 + g_half_rows;
        }
    }
    cv::Size dsize(dst.cols, dst.rows);
    dst = h2o::remap_ipp(src, mapx, mapy);

    return true;
}

class PixelDifferenceCostFunctorEx {
public:
    PixelDifferenceCostFunctorEx(const cv::Ptr<cv::Mat> &_image1, const cv::Ptr<cv::Mat> &_image2_with_gradient,
                                 const cv::Ptr<cv::Mat> &_image2)
        : image1(_image1), image2_with_gradient(_image2_with_gradient), image2(_image2) {}
    ~PixelDifferenceCostFunctorEx() {}
    template <typename T> bool operator()(const T *param, T *residuals) const {

        int half_lsm = image1->cols / 2;
        int window_size = half_lsm * 2 + 1;
        int g_half_cols = image2_with_gradient->cols / 2;
        int g_half_rows = image2_with_gradient->rows / 2;

        /**利用畸变参数对影像进行纠正，为了使得后续计算速度更快
         *对于T为double的时候，只存一个channel，反之存3个channel
         */
        cv::Mat image2_remap;
        //如果此时不是Jet类型，是double，那么只计算第一个channel的remap
        {
            std::vector<double> scalar_param(8);
            for (int i = 0; i < 8; ++i)
                scalar_param[i] = JetOps<T>::GetScalar(param[i]);

            if (JetOps<T>::IsScalar()) {
                h2o::warpAffine(*image2, image2_remap, image1->size(), scalar_param);
                int pos = 0;
                float *image1_data = (float *)image1->data;
                float *image2_data = (float *)image2_remap.data;
                for (int i = 0; i < window_size * window_size; ++i) {
                    double f_value = *image1_data++;
                    double g_value = *image2_data++;
                    T cost = param[6] * g_value + param[7] - f_value;
                    residuals[pos++] = cost;
                }
            } else {
                h2o::warpAffine(*image2_with_gradient, image2_remap, image1->size(), scalar_param);
                int pos = 0;
                float *image1_data = (float *)image1->data;
                cv::Vec3f *image2_data = (cv::Vec3f *)image2_remap.data;

                Eigen::Matrix<T, 2, 1> pt2d, shift;
                Eigen::Matrix<T, 2, 2> A;
                // 				T xy[2];
                Eigen::Matrix<T, 2, 1> image2_pos;
                T image2_sample;
                T cost;

                for (int r = -half_lsm; r <= half_lsm; ++r) {
                    for (int c = -half_lsm; c <= half_lsm; ++c) {
                        double f_value = *image1_data++;
                        cv::Vec3f g_value = *image2_data++;

                        pt2d(0) = T(c);
                        pt2d(1) = T(r);
                        shift(0) = param[2];
                        shift(1) = param[5];

                        A(0, 0) = param[0];
                        A(0, 1) = param[1];
                        A(1, 0) = param[3];
                        A(1, 1) = param[4];

                        image2_pos = A * pt2d + shift;

                        image2_sample = Chain<float, 2, T>::Rule(g_value[0], &g_value[1], image2_pos.data());
                        cost = param[6] * image2_sample + param[7] - T(f_value);
                        residuals[pos++] = cost;
                    }
                }
            }
        }
        return true;
    }

private:
    ///\brief template 影像，他的窗口大小决定LSM匹配的窗口大小
    cv::Ptr<cv::Mat> image1;

    ///\brief 搜索影像，中心和template是对应上的
    cv::Ptr<cv::Mat> image2_with_gradient;

    cv::Ptr<cv::Mat> image2;
};
} // namespace h2o

namespace ceres {
class LsmPixelFunctor : public SizedCostFunction<ceres::DYNAMIC, 6> {
public:
    // I - 单波段 I
    // I' - 单波段
    LsmPixelFunctor(cv::Mat *I, cv::Mat *gx, cv::Mat *gy, cv::Mat *Is)
        : SizedCostFunction(), I(I), Is(Is), gx(gx), gy(gy) {
        int lsm_window = I->rows - 2;
        set_num_residuals(lsm_window * lsm_window);
    }

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override {
        double a, b, c, d, e, f;
        a = parameters[0][0];
        b = parameters[0][1];
        c = parameters[0][2];
        d = parameters[0][3];
        e = parameters[0][4];
        f = parameters[0][5];

        int rows = I->rows;
        int cols = I->cols;

        int half_rows_Is = Is->rows / 2;
        int half_cols_Is = Is->cols / 2;

        // left and right gradient is interpolated
        int lsm_window = rows / 2 - 1;
        int half_size = rows / 2;

        int pos = 0;
        // F = I'(ax+by+c,dx+ey+f) - I(x,y)
        for (int row = -lsm_window; row <= lsm_window; ++row) {
            float *ptr_I = I->ptr<float>(row + half_size);
            float y = row;

            for (int col = -lsm_window; col <= lsm_window; ++col) {
                float x = col;
                float xs = a * x + b * y + c + half_cols_Is;
                float ys = d * x + e * y + f + half_rows_Is;

                if (xs < 0 || xs > Is->cols - 1 || ys < 0 || ys > Is->rows - 1) {
                    return false;
                }

                float Is_value = h2o::image_interpolate<float>(*Is, Eigen::Vector2f(xs, ys), cv::BORDER_REPLICATE);
                float I_value = ptr_I[col + half_size];
                residuals[pos] = Is_value - I_value;
                pos++;
            }
        }

        // we have only one parameter block
        if (!jacobians)
            return true;
        double *jacobian = jacobians[0];
        // J[r * 6 + c] = dF[r]/d[c]
        if (!jacobian)
            return true;

        // we use dIdx to approximate dI'dx'

        // dFda = dI'dx' * dx'da = dIdx * x
        // dFdb = dIdx * y
        // dFdc = dIdx
        // dFdd = dIdy * x
        // dFde = dIdy * y
        // dFdf = dIdy
        pos = 0;
        for (int row = -lsm_window; row <= lsm_window; ++row) {
            float *ptr_I = I->ptr<float>(row + half_size);
            float *ptr_gx = gx->ptr<float>(row + half_size);
            float *ptr_gy = gy->ptr<float>(row + half_size);
            float y = row;

            for (int col = -lsm_window; col <= lsm_window; ++col) {
                float x = col;
                jacobian[pos * 6 + 0] = ptr_gx[col + half_size] * x;
                jacobian[pos * 6 + 1] = ptr_gx[col + half_size] * y;
                jacobian[pos * 6 + 2] = ptr_gx[col + half_size];
                jacobian[pos * 6 + 3] = ptr_gy[col + half_size] * x;
                jacobian[pos * 6 + 4] = ptr_gy[col + half_size] * y;
                jacobian[pos * 6 + 5] = ptr_gy[col + half_size];
                pos++;
            }
        }

        return true;
    };

private:
    cv::Mat *gx;
    cv::Mat *gy;
    cv::Mat *I;
    cv::Mat *Is;
};

} // namespace ceres

namespace h2o {
LSMSummary lsm_match(const cv::Mat &mat, const cv::Mat &search, const std::vector<double> &init) {
    cv::Mat I32f, gx, gy;
    mat.convertTo(I32f, CV_32F);
    I32f = I32f - cv::mean(I32f);

    //-1 0 1
    //-2 0 2
    //-1 0 1
    cv::Sobel(I32f, gx, CV_32F, 1, 0, 3, 0.125);
    cv::Sobel(I32f, gy, CV_32F, 0, 1, 3, 0.125);

    cv::Mat Is32f;
    search.convertTo(Is32f, CV_32F);
    Is32f = Is32f - cv::mean(Is32f);

    std::vector<double> param = init;
    ceres::Problem problem;
    ceres::CostFunction *cost = new ceres::LsmPixelFunctor(&I32f, &gx, &gy, &Is32f);
    problem.AddResidualBlock(cost, nullptr, param.data());

    ceres::Solver::Options options;
    options.max_num_iterations = 5;
    options.minimizer_type = ceres::TRUST_REGION;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-2;
    options.gradient_tolerance = 1e-2;
    options.parameter_tolerance = 1e-2;

    ceres::Solver::Summary ceres_summary;
    ceres::Solve(options, &problem, &ceres_summary);

    LSMSummary summary;
    summary.iterations = ceres_summary.iterations.size();
    summary.initial_cost = ceres_summary.initial_cost;
    summary.final_cost = ceres_summary.final_cost;
    summary.runtime = ceres_summary.total_time_in_seconds * 1000;
    summary.x = param;
    if (ceres_summary.IsSolutionUsable()) {
        summary.is_converge = true;
    } else {
        summary.is_converge = false;
    }

    return summary;
}

//
// LSMSummary lsm_match(const cv::Mat &lsm_templ, const cv::Mat &lsm_match, const std::vector<double> &init) {
//     LSMSummary summary;
//
//     ceres::Problem problem;
//
//     // f = k1 * g(a1x + a2y + a3, b1x + b2y + b3) + k2;
//     // X : a1 a2 a3 b1 b2 b3 k1 k2
//     // X : 1 0 0 0 1 0 1 0
//     std::vector<double> params = init;
//
//     cv::Mat lsm_image2_channel[3];
//     lsm_match.convertTo(lsm_image2_channel[0], CV_32F);
//
//     //-1 0 1
//     //-2 0 2
//     //-1 0 1
//     cv::Sobel(lsm_image2_channel[0], lsm_image2_channel[1], CV_32F, 1, 0, 3, 0.125);
//     cv::Sobel(lsm_image2_channel[0], lsm_image2_channel[2], CV_32F, 0, 1, 3, 0.125);
//
//     cv::Ptr<cv::Mat> ptr_lsm_image1(new cv::Mat());
//     cv::Ptr<cv::Mat> ptr_lsm_image2_with_gradient(new cv::Mat());
//     cv::Ptr<cv::Mat> ptr_lsm_image2(new cv::Mat());
//     *ptr_lsm_image2 = lsm_image2_channel[0];
//
//     cv::Mat lsm_image2_with_gradient;
//     cv::merge(lsm_image2_channel, 3, *ptr_lsm_image2_with_gradient);
//
//     int half_lsm = lsm_templ.rows / 2;
//     int lsm_window = 2 * half_lsm + 1;
//     *ptr_lsm_image1 = lsm_templ;
//     lsm_templ.convertTo(*ptr_lsm_image1, CV_32F);
//
//     const int residual_size = lsm_window * lsm_window;
//     ceres::CostFunction *cost_function =
//         new ceres::AutoDiffCostFunction<PixelDifferenceCostFunctorEx, ceres::DYNAMIC, 8>(
//             new PixelDifferenceCostFunctorEx(ptr_lsm_image1, ptr_lsm_image2_with_gradient, ptr_lsm_image2),
//             lsm_window * lsm_window);
//     problem.AddResidualBlock(cost_function, NULL, params.data());
//
//     ceres::Solver::Options options;
//     options.max_num_iterations = 5;
//     options.minimizer_type = ceres::TRUST_REGION;
//     options.linear_solver_type = ceres::DENSE_QR;
//     options.function_tolerance = 1e-2;
//     options.gradient_tolerance = 1e-2;
//     options.parameter_tolerance = 1e-2;
//
//     ceres::Solver::Summary ceres_summary;
//     ceres::Solve(options, &problem, &ceres_summary);
//
//     summary.iterations = ceres_summary.iterations.size();
//     summary.initial_cost = ceres_summary.initial_cost;
//     summary.final_cost = ceres_summary.final_cost;
//     summary.runtime = ceres_summary.total_time_in_seconds * 1000;
//     summary.x = params;
//     if (ceres_summary.IsSolutionUsable()) {
//         summary.is_converge = true;
//     } else {
//         summary.is_converge = false;
//     }
//     return summary;
// }

} // namespace h2o

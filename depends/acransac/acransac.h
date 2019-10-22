#include <opencv2/core.hpp>
#include <vector>

namespace acransac {

/**
 * \brief		acransac 采用ACRANSAC进行单应矩阵估计，剔除粗差
 * \param[in]	const vector<cv::DMatch> & matches 匹配点
 * \param[in]	const vector<cv::KeyPoint> & kpts_train
 * \param[in]	const vector<cv::KeyPoint> & kpts_query
 * \param[in]	vector<cv::DMatch> & matchout 输入，可以inplace
 *
 * http://www.ipol.im/pub/art/2012/mmm-oh/
 */
void acransac_homography(const std::vector<cv::DMatch> &matches, const std::vector<cv::KeyPoint> &kpts_train,
                         const std::vector<cv::KeyPoint> &kpts_query, std::vector<cv::DMatch> &matchout);

std::vector<cv::DMatch> acransac_homography(const std::vector<cv::DMatch> &matches,
                                            const std::vector<cv::KeyPoint> &kpts_train,
                                            const std::vector<cv::KeyPoint> &kpts_query, double threshold);

void acransac_fundamental(const std::vector<cv::DMatch> &matches, const std::vector<cv::KeyPoint> &kpts_train,
                          const std::vector<cv::KeyPoint> &kpts_query, std::vector<cv::DMatch> &matchout);

std::vector<cv::DMatch> acransac_fundamental(const std::vector<cv::DMatch> &matches,
                                             const std::vector<cv::KeyPoint> &kpts_train,
                                             const std::vector<cv::KeyPoint> &kpts_query, double threshold);

std::vector<cv::DMatch> acransac_translation(const std::vector<cv::DMatch> &matches,
                                             const std::vector<cv::KeyPoint> &kpts_train,
                                             const std::vector<cv::KeyPoint> &kpts_query, double threshold);

} // namespace acransac

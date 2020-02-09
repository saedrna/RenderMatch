/*
 * @Author: Han
 * @Date: 2019-11-15 20:32:43
 * Match aerial and ground images, with rendered images as delegates
 */
#include "render_matcher.h"

#include <match/lsm.h>
#include <match/sift_matcher.h>

#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include<opencv2/calib3d/calib3d.hpp>

#include <Eigen/Jacobi>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include <stdio.h>

void imgSubt(cv::Mat &imgRac, cv::Mat &imgOur) {
	for (int i=0;i<imgRac.rows;i++)
	{
		for (int j=0;j<imgRac.cols;j++)
		{
			if (imgRac.at<cv::Vec3b>(i,j)!=imgOur.at<cv::Vec3b>(i,j))
			{
				if (imgRac.at<cv::Vec3b>(i, j)== cv::Vec3b(255,0 , 0))
				{
					imgOur.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
				}
				if (imgOur.at<cv::Vec3b>(i, j) == cv::Vec3b(255,0, 0))
				{
					imgOur.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 255, 0);
					
				}				
			}
		}
	}
}
cv::Mat drawRenderMatch(cv::Mat mat_ground, cv::Mat mat_aerial, std::vector<cv::DMatch> matches, std::vector<cv::KeyPoint> keys_render, std::vector<cv::KeyPoint> keys_ground,cv::Scalar &color=cv::Scalar(0,255,0,0),int thickness=7)
{
	
	int crosssize = 180;
	auto mat_ground_size = mat_ground.size();

	if (mat_ground.type()!=mat_aerial.type())
	{
		cv::cvtColor(mat_ground, mat_ground, cv::COLOR_RGBA2RGB);
	}
	

    std::cout << mat_ground.type() << "   " << mat_aerial.type() << std::endl;

	if (mat_ground.size != mat_aerial.size)
		cv::resize(mat_ground, mat_ground, cv::Size(mat_aerial.cols, mat_aerial.rows), 0, 0);

	cv::Mat ter_aer_mat;
	ter_aer_mat.create(std::max<int>(mat_ground.rows, mat_ground.rows), mat_ground.cols + mat_ground.cols,
		mat_aerial.type()); // des.create()
	cv::Mat r1 = ter_aer_mat(cv::Rect(0, 0, mat_ground.cols, mat_ground.rows));
	mat_ground.copyTo(r1);
	cv::Mat r2 = ter_aer_mat(cv::Rect(mat_ground.cols, 0, mat_ground.cols, mat_ground.rows));
	mat_aerial.copyTo(r2);

	cv::Mat ter_aer_mat_vertical;
	ter_aer_mat_vertical.create(mat_aerial.rows + mat_aerial.rows, mat_aerial.cols,
		mat_aerial.type()); // des.create()
	cv::Mat r11 = ter_aer_mat_vertical(cv::Rect(0, 0, mat_aerial.cols, mat_aerial.rows));
	mat_ground.copyTo(r11);
	cv::Mat r22 = ter_aer_mat_vertical(cv::Rect(0, mat_aerial.rows, mat_aerial.cols, mat_aerial.rows));
	mat_aerial.copyTo(r22);	

	int countKey = 0;
	for (const auto &match : matches) {


		// cv::line(mat_aerial, cv::Point(key_aerial[i].x() - crosssize / 2, key_aerial[i].y()),
		//         cv::Point(key_aerial[i].x() + crosssize / 2, key_aerial[i].y()), color, thickness, 8, 0);
		////绘制竖线
		// cv::line(mat_aerial, cv::Point(key_aerial[i].x(), key_aerial[i].y() - crosssize / 2),
		//         cv::Point(key_aerial[i].x(), key_aerial[i].y() + crosssize / 2), color, thickness, 8, 0);

		line(ter_aer_mat,
			cv::Point(keys_render[match.queryIdx].pt.x * mat_aerial.cols / mat_ground_size.width,
				keys_render[match.queryIdx].pt.y * mat_aerial.rows / mat_ground_size.height),
			cv::Point(keys_ground[match.trainIdx].pt.x + mat_aerial.cols, keys_ground[match.trainIdx].pt.y), color,thickness);

		line(ter_aer_mat_vertical,
			cv::Point(keys_render[match.queryIdx].pt.x * mat_aerial.cols / mat_ground_size.width,
				keys_render[match.queryIdx].pt.y * mat_aerial.rows / mat_ground_size.height),
			cv::Point(keys_ground[match.trainIdx].pt.x, keys_ground[match.trainIdx].pt.y + mat_aerial.rows), color,thickness);

		/*  circle(mat_ground,
				 cv::Point(key_ground.x() * mat_aerial.cols / mat_ground_size.width,
						   key_ground.y() * mat_aerial.rows / mat_ground_size.height),
				 5, color, 5);
		  circle(mat_aerial, cv::Point(key_aerial[i].x(), key_aerial[i].y()), 2, color, 3);*/




	}
	return ter_aer_mat;
}

RenderMeshMatchConfig load_config(const std::string &path) {
    std::ifstream ifile(path);
    nlohmann::json j;
    ifile >> j;

    RenderMeshMatchConfig config;
    config.use_occlusion = j["UseOcclusion"];
    config.angle_difference = j["AngleDifference"];
    config.ncc_window = j["NccWindow"];
    config.ncc_threshold = j["NccSearch"];
    config.ncc_threshold = j["NccThreshold"];
    config.use_lsm = j["UseLsm"];
    return config;
}

RenderMatcher::RenderMatcher() { sift_ = std::make_shared<SiftDetector>(); }

void RenderMatcher::set_param(const RenderMeshMatchConfig &param) { param_ = param; }

void RenderMatcher::set_block(const h2o::Block &aerial, const h2o::Block &ground) {
    block_ground_ = ground;
    block_aerial_ = aerial;

    // initialize images
    for (const auto &pair : ground.photos) {
        DiskImagePtr image = std::make_shared<DiskImage>(pair.second.path, true);
        images_ground_.emplace(pair.first, image);
    }
    for (const auto &pair : aerial.photos) {
        DiskImagePtr image = std::make_shared<DiskImage>(pair.second.path, true);
        images_aerial_.emplace(pair.first, image);
    }
}

void RenderMatcher::set_ogl_matrices(const Matrix4f &view, const MatrixXf &proj) {
    mvp_inverse_ = (proj * view).inverse();
}

RenderMatchResults RenderMatcher::match(uint32_t iid, const cv::Mat &mat_rgb, const cv::Mat &mat_dep) {

     //std::string folderPath = std::to_string(iid);
     //std::string command;
     //command = "mkdir  " /*+std::to_string(iid)*/ + folderPath;
     //system(command.c_str());

    // initialize viewpoint
    viewport_.x() = mat_dep.cols;
    viewport_.y() = mat_dep.rows;

    cv::Mat render_img = mat_rgb.clone();

    // load the sift descriptors of the ground image
    std::vector<cv::KeyPoint> keys_ground;
    cv::Mat desc_ground;

    std::string path = block_ground_.photos.at(iid).path;
    std::string name = get_filename_noext(path);
    std::string dir = get_directory(path);
    path = join_paths(dir, name + ".sift");
    if (file_exist(path)) {
        sift_read(path, keys_ground, desc_ground);
    } else {
        cv::Mat mat = cv::imread(images_ground_.at(iid)->get_path(), cv::IMREAD_GRAYSCALE);
		
        mat = image_percent_scale_8u(mat);
        std::tie(keys_ground, desc_ground) = sift_->detect_and_compute(mat);
    }
	cv::Mat mat_ground =cv::imread(images_ground_.at(iid)->get_path());
    // extract sift from rendered images
    std::vector<cv::KeyPoint> keys_render;
    cv::Mat desc_render;
    {
        cv::Mat mat = mat_rgb.clone();
        cv::Mat alpha;
        {
            std::vector<cv::Mat> channels;
            cv::split(mat_rgb, channels);
            alpha = channels[3];
        }

        cv::cvtColor(mat, mat, cv::COLOR_BGRA2GRAY);
        mat = image_percent_scale_8u(mat, alpha != 0);
        std::tie(keys_render, desc_render) = sift_->detect_and_compute(mat);
    }

    // match sift
    SiftMatcherParam sift_param;
    SiftMatcher sift_matcher;
    sift_matcher.set_match_param(sift_param);
    sift_matcher.set_train_data(keys_ground, desc_ground);

	/*SIFT*/
	std::vector<cv::DMatch> matchesSift = sift_matcher.matchSift(keys_render, desc_render);
	cv::Mat ground_render_sift;
	//drawMatches(render_img, keys_render, mat_ground, keys_ground, matchesSift, ground_render_sift, cv::Scalar::all(-1));
   // cv::imwrite(name+"+sift+" + std::to_string(matchesSift.size())+".png", ground_render_sift);
    cv::Mat ground_renderSift = drawRenderMatch(render_img, mat_ground, matchesSift, keys_render, keys_ground,cv::Scalar(0,std::rand()%255, std::rand() % 255, std::rand() % 255),1);
	cv::imwrite(name + "sift" + std::to_string(matchesSift.size()) + ".png", ground_renderSift);

	/*SIFT+ransac*/
     std::vector<cv::DMatch> matchesNoc = sift_matcher.match(keys_render, desc_render);
	 cv::Mat &ground_render1= drawRenderMatch(render_img, mat_ground, matchesNoc, keys_render, keys_ground,cv::Scalar(255,0,0,0));
    // cv::imwrite(name + "+sift+ransac+" + std::to_string(matchesNoc.size())+ ".png", ground_render1);

	  /*SIFT+Proposed+ransac*/
     std::vector<cv::DMatch> matches = sift_matcher.matchp(keys_render, desc_render, keys_ground, keys_render);
     cv::Mat &ground_render3 =
         drawRenderMatch(render_img, mat_ground, matches, keys_render, keys_ground, cv::Scalar(255, 0, 0, 0));


     
	 imgSubt(ground_render1, ground_render3);//overlapping filter results between proposed and ransac only
	 //cv::imwrite(name + "+sift+ransac+" + std::to_string(matchesNoc.size()) + ".png", ground_render1);
	 cv::imwrite(name + "+proposed+" + std::to_string(matches.size()) +"+"+std::to_string(matchesNoc.size())+ ".png", ground_render3);

	 {
		 cv::Mat ground_img = cv::imread(join_paths(dir, name + ".tif"), 1);
		 cv::Mat ground_img_clone = ground_img.clone();
		 for (const auto &match : matches) {

			 auto key_ground = keys_ground.at(match.trainIdx);
			 auto key_render = keys_render.at(match.queryIdx);

			 // draw arrowedLine in images
             cv::arrowedLine(ground_img, key_render.pt, key_ground.pt, cv::Scalar(255, 0, 255), 3, 8, 0, 0.3);
		
			 for (const auto &match : matches) {				
				 auto key_ground = keys_ground.at(match.trainIdx);
                 auto key_render = keys_render.at(match.queryIdx);
                 // draw cross feature in images
                 {
					 int crosssize = 5,thickness=2;
					 cv::Scalar color(0, 255, 0,255);
                      cv::line(ground_img, cv::Point(key_ground.pt.x - crosssize / 2, key_ground.pt.y),
                              cv::Point(key_ground.pt.x + crosssize / 2, key_ground.pt.y), color, thickness, 8, 0);                     
                      cv::line(ground_img, cv::Point(key_ground.pt.x, key_ground.pt.y - crosssize / 2),
                              cv::Point(key_ground.pt.x, key_ground.pt.y + crosssize / 2), color, thickness, 8, 0);
                      cv::line(mat_rgb, cv::Point(key_render.pt.x - crosssize / 2, key_render.pt.y),
                              cv::Point(key_render.pt.x + crosssize / 2, key_render.pt.y), color, thickness, 8, 0);                   
                      cv::line(mat_rgb, cv::Point(key_render.pt.x, key_render.pt.y - crosssize / 2),
                              cv::Point(key_render.pt.x, key_render.pt.y + crosssize / 2), color, thickness, 8, 0);
                     cv::arrowedLine(ground_img_clone, key_render.pt, key_ground.pt, cv::Scalar(255, 0, 255), 3, 8, 0,0.3);                    
                 }
			 }
		 }
		 std::string r2gName = name + "+r2g+lineintersect.png", siftG2RName = name +"+"+std::to_string(matches.size())+ "+LRCF.png";

		 cv::imwrite(siftG2RName, ground_img_clone);
		
		 auto svgName = (char*)r2gName.data();

		 cv::Mat match_mat, match_mat_noc;
	 }

	//do Lmeds test
	/*{
		using namespace std;
		using namespace cv;
		std::vector<std::vector<cv::DMatch>> Lmatches_;
		cv::BFMatcher Lmatcher;
		Lmatcher.knnMatch(desc_render, desc_ground, Lmatches_,2);
        vector<DMatch> goodMatches;

        for (int i = 0; i < Lmatches_.size(); i++) {

            const DMatch &bestmatch = Lmatches_[i][0];
            const DMatch &bettermatch = Lmatches_[i][1];
            float ratio = bestmatch.distance / bettermatch.distance;
            if (ratio < 0.95) {
                goodMatches.push_back(bestmatch);
            }
        }
		int matchNum = Lmatches_.size();
		vector <KeyPoint> RAN_KP1, RAN_KP2;
        for (size_t i = 0; i < goodMatches.size(); i++) {
           
            RAN_KP1.push_back(keys_render[goodMatches[i].queryIdx]);
           
            RAN_KP2.push_back(keys_ground[goodMatches[i].trainIdx]);           
        }
        vector<Point2f> p01, p02;
        for (size_t i = 0; i < Lmatches_.size(); i++) {
            p01.push_back(RAN_KP1[i].pt);
            p02.push_back(RAN_KP2[i].pt);
        }
        vector<uchar> RansacStatus,lMedsStatus;
        vector<uchar> RansacHomography;
        Mat mat_Fundamental = cv::findFundamentalMat(p01, p02, lMedsStatus, LMEDS);
        Mat mat_Fundamental_ransac = cv::findFundamentalMat(p01, p02, RansacStatus, RANSAC);
        vector<KeyPoint> RR_KP1, RR_KP2, RR_KP3, RR_KP4;
        vector<DMatch> RR_matches1, RR_matches2;
        int index = 0;
        for (size_t i = 0; i < goodMatches.size(); i++) {
            if (lMedsStatus[i] != 0) {
                RR_KP1.push_back(RAN_KP1[i]);
                RR_KP2.push_back(RAN_KP2[i]);
				goodMatches[i].queryIdx = index;
				goodMatches[i].trainIdx = index;
                RR_matches1.push_back(goodMatches[i]);
                index++;
            }
        }
        for (size_t i = 0; i < goodMatches.size(); i++) {
            if (RansacStatus[i] != 0) {
                RR_KP3.push_back(RAN_KP1[i]);
                RR_KP4.push_back(RAN_KP2[i]);
                goodMatches[i].queryIdx = index;
                goodMatches[i].trainIdx = index;
                RR_matches2.push_back(goodMatches[i]);
                index++;
            }
        }
        Mat img_RR_matches, img_LM_matches;
        
        drawMatches(render_img, RR_KP1, mat_ground, RR_KP2, RR_matches1, img_LM_matches);
		drawMatches(render_img, RR_KP3, mat_ground, RR_KP4, RR_matches2, img_RR_matches);
	}*/
     

    // too few matches
    if (matches.size() < 10) {
        return RenderMatchResults();
    }

	/*draw matches if maches number is more than 10*/

 // for each match expand to aerial views with patch match
	RenderMatchResults results;

    for (const auto &match : matches) {
        int count = 0;

        auto key_ground = keys_ground.at(match.trainIdx);
        auto key_render = keys_render.at(match.queryIdx);

        int r = int(key_render.pt.y + 0.5);
        int c = int(key_render.pt.x + 0.5);

        // extract ground patch
        cv::Mat paper_sub_ter_mat, mat_ground_template;
        std::tie(paper_sub_ter_mat, mat_ground_template) =get_patch_on_ground_image(iid, Vector2d(key_ground.pt.x, key_ground.pt.y));
        if (mat_ground_template.empty()) {
            continue;
        }

        // debug
        //         cv::Mat mat_patch_ren = debug_patch_on_render_image(mat_rgb, Vector2d(key_render.pt.x,
        //         key_render.pt.y));

        // extract a planar (a point and normal) on the rendered depth image
        MatrixXf corners_ren;
        MatrixXf point_ren;
        Vector3f normal_ren;
        std::tie(corners_ren, point_ren, normal_ren) = get_patch_on_rendered_image(iid, Vector2i(c, r), mat_dep);
        if (corners_ren.rows() == 0) {
            continue;
        }

        RenderMatchResult result;
        result.xyz = point_ren;
        result.iid_ground = iid;
        result.pt_ground = Vector2f(key_ground.pt.x, key_ground.pt.y);

        // search visible aerial images
        std::vector<uint32_t> iids_aerial = search_visible_aerial_images(corners_ren, normal_ren);
		
        if (iids_aerial.empty()) {
            continue;
        }

        std::vector<cv::Mat> mats_patch;
        std::vector<Matrix3f> Hs_pat_aer;
        std::vector<uint32_t> iids_aerial_valid;
        for (uint32_t iid_aerial : iids_aerial) {
            int iid_back = 0;
            cv::Mat mat_pat, paper_sub_pat;
            Matrix3f H_pat_aer;
            std::tie(paper_sub_pat, mat_pat, H_pat_aer) =
                get_patch_on_aerial_image(iid, iid_aerial, corners_ren, normal_ren);
            if (mat_pat.empty()) {
                continue;
            }
            cv::Mat aer_mat_pat = mat_pat.clone();
            if (mat_pat.channels() == 3) {
                cv::cvtColor(mat_pat, mat_pat, cv::COLOR_RGB2GRAY);
            }
            // do template match
            cv::Mat ncc;
            cv::matchTemplate(mat_pat, mat_ground_template, ncc, cv::TM_CCOEFF_NORMED);
           
            double ncc_max;
            cv::Point pos;
            cv::minMaxLoc(ncc, nullptr, &ncc_max, nullptr, &pos);

            if (ncc_max < param_.ncc_threshold) {
                continue;
            }
            int ncc_size = mat_pat.rows - mat_ground_template.rows + 1;
            std::vector<double> affine = {
                1.0, 0.0, (double)pos.x - ncc_size / 2, 0.0, 1.0, (double)pos.y - ncc_size / 2, 1.0, 0.0};

            if (param_.use_lsm) {
                LSMSummary summary = lsm_match(mat_ground_template, mat_pat, affine);
                if (!summary.is_converge) {
                    continue;
                }
            }

            Vector2f point_pat(affine[2] + mat_pat.cols / 2, affine[5] + mat_pat.rows / 2);
            Vector2f point_aer = (H_pat_aer * point_pat.homogeneous()).hnormalized();

            result.iid_aerial.push_back(iid_aerial);
            iid_back++;
            result.pt_aerial.push_back(point_aer);
        }

        if (result.iid_aerial.empty()) {
            continue;
        }
        results.push_back(result);
    }
   // std::cout << path << std::endl;
    return results;
}

cv::Mat RenderMatcher::draw_matches(uint32_t iid_ground, uint32_t iid_aerial, const RenderMatchResults &matches) {
    std::vector<cv::KeyPoint> keys_ground, keys_aerial;
    std::vector<cv::DMatch> dmatches;

    int pos = 0;
    for (const auto &match : matches) {
        uint32_t iid = match.iid_ground;
        if (iid != iid_ground) {
            continue;
        }

        for (int i = 0; i < match.iid_aerial.size(); ++i) {
            uint32_t iid2 = match.iid_aerial[i];
            if (iid2 != iid_aerial) {
                continue;
            }

            Vector2f pt_aerial = match.pt_aerial[i];

            dmatches.push_back(cv::DMatch(pos, pos, 0.0f));
            keys_ground.push_back(cv::KeyPoint(match.pt_ground.x(), match.pt_ground.y(), 32.0f));
            keys_aerial.push_back(cv::KeyPoint(pt_aerial.x(), pt_aerial.y(), 32.0f));
            pos++;
        }
    }

    if (dmatches.size() == 0) {
        return cv::Mat();
    }

    std::string path_ground = block_ground_.photos.at(iid_ground).path;
    std::string path_aerial = block_aerial_.photos.at(iid_aerial).path;

    cv::Mat mat_ground = cv::imread(path_ground, cv::IMREAD_UNCHANGED);
    cv::Mat mat_aerial = cv::imread(path_aerial, cv::IMREAD_UNCHANGED);

	
    cv::Mat mat;
    cv::drawMatches(mat_ground, keys_ground, mat_aerial, keys_aerial, dmatches, mat, cv::Scalar(0, 255, 0));
	

	 int crosssize = 180, thickness = 30;
    cv::Scalar color(0, 255, 255, 255);

	auto mat_ground_size = mat_ground.size();

	if (mat_ground.size != mat_aerial.size) cv::resize(mat_ground, mat_ground, cv::Size(mat_aerial.cols, mat_aerial.rows), 0, 0);
	
     cv::Mat ter_aer_mat;
    ter_aer_mat.create(std::max<int>(mat_ground.rows, mat_ground.rows), mat_ground.cols + mat_ground.cols,
                        mat_ground.type()); // des.create()	
	cv::Mat r1 = ter_aer_mat(cv::Rect(0, 0, mat_ground.cols, mat_ground.rows));
    mat_ground.copyTo(r1);
    cv::Mat r2 = ter_aer_mat(cv::Rect(mat_ground.cols, 0, mat_ground.cols, mat_ground.rows));
    mat_aerial.copyTo(r2);

	cv::Mat ter_aer_mat_vertical;
	ter_aer_mat_vertical.create(mat_aerial.rows+ mat_aerial.rows, mat_aerial.cols,
		mat_aerial.type()); // des.create()
    cv::Mat r11 = ter_aer_mat_vertical(cv::Rect(0, 0, mat_aerial.cols, mat_aerial.rows));
    mat_ground.copyTo(r11);
    cv::Mat r22 = ter_aer_mat_vertical(cv::Rect(0, mat_aerial.rows, mat_aerial.cols, mat_aerial.rows));
    mat_aerial.copyTo(r22);

	int countKey = 0;

    return mat;
}

std::tuple<MatrixXf, Vector3f, Vector3f> RenderMatcher::get_patch_on_rendered_image(uint32_t iid, const Vector2i &pt,
                                                                                    const cv::Mat &mat_dep) {

    int patch_size = (param_.ncc_window / 2 + param_.ncc_search / 2) * 2 + 1;
    int patch_half = patch_size / 2;

    int rows = mat_dep.rows;
    int cols = mat_dep.cols;

    BoundingBox2i bounds_image;
    bounds_image.extend(Vector2i(0, 0));
    bounds_image.extend(Vector2i(cols - 1, rows - 1));
    BoundingBox2i bounds_patch;
    bounds_patch.extend(Vector2i(pt.x() - patch_half, pt.y() - patch_half));
    bounds_patch.extend(Vector2i(pt.x() + patch_half + 1, pt.y() + patch_half + 1));

    if (!bounds_image.contains(bounds_patch)) {
        return std::make_tuple(MatrixXf(), Vector3f::Zero(), Vector3f::Zero());
    }

    // compute the 3d point of the four corners and center
    MatrixXf corners(3, 4);
    for (int i = 0; i < 4; ++i) {
        Vector2i corner = bounds_patch.corner(BoundingBox2i::CornerType(i));
        float depth = mat_dep.at<float>(corner.y(), corner.x());
        if (std::abs(depth - 1.0) < 1e-4) {
            // invalid depth value
            return std::make_tuple(MatrixXf(), Vector3f::Zero(), Vector3f::Zero());
        }

        corners.col(i) = depth_to_xyz(depth, corner);
    }

    // compute the eigen vectors
    Vector3f nor;
    {
        Vector3f mean = corners.rowwise().mean();
        MatrixXf centered = corners.colwise() - mean;
        Eigen::JacobiSVD<MatrixXf> jacobi(centered, Eigen::ComputeFullU | Eigen::ComputeFullV);
        nor = jacobi.matrixU().col(2);
    }

    // reoriente the normal
    Vector3f point;
    {
        float depth = mat_dep.at<float>(pt.y(), pt.x());
        if (std::abs(depth - 1.0) < 1e-4) {
            // invalid depth value
            return std::make_tuple(MatrixXf(), Vector3f::Zero(), Vector3f::Zero());
        }
        point = depth_to_xyz(depth, pt);
        Vector3f C = block_ground_.photos.at(iid).C.cast<float>();

        if (nor.dot(C - point) < 0) {
            nor = -nor;
        }
    }

    return std::tie(corners, point, nor);
}

cv::Mat RenderMatcher::debug_patch_on_render_image(const cv::Mat &mat_rgb, const Vector2d &point) {
    int patch_size = (param_.ncc_window / 2) * 2 + 1;
    int patch_half = patch_size / 2;

    int rows = mat_rgb.rows;
    int cols = mat_rgb.cols;

    BoundingBox2i bounds_image;
    bounds_image.extend(Vector2i(0, 0));
    bounds_image.extend(Vector2i(cols, rows));

    BoundingBox2i bounds_patch;
    bounds_patch.extend(Vector2i(point.x() - 1.5 - patch_half, point.y() - 1.5 - patch_half));
    bounds_patch.extend(Vector2i(point.x() + 2.5 + patch_half, point.y() + 2.5 + patch_half));

    if (!bounds_image.contains(bounds_patch)) {
        return cv::Mat();
    }

    cv::Mat sub;
    cv::Mat mat = mat_rgb.clone();
    cv::cvtColor(mat, mat, cv::COLOR_BGRA2GRAY);
    cv::getRectSubPix(mat, cv::Size(patch_size, patch_size), cv::Point2f(point.x(), point.y()), sub);

    return sub;
}

std::tuple<cv::Mat, cv::Mat, Matrix3f> RenderMatcher::get_patch_on_aerial_image(uint32_t iid_ground,
                                                                                uint32_t iid_aerial,
                                                                                const MatrixXf &corners,
                                                                                const Vector3f &normal) {
    // the corners corresponding to the 3D points of the four 2d corners of the final warped patch
    // we have four coordinate systems
    // - patch (as pat): the final patch warped to the ground view
    // - rendered image (as ren): assumes to have the no deformation with the patch and corners are input
    // - aerial (as aer): the coordinate of the aerial images
    // - aerial subset (as sub): a subset region, which is loaded and used for warp to the final patch
    Photo photo_aerial = block_aerial_.photos.at(iid_aerial);
    BoundingBox2i bounds_aer;
    bounds_aer.extend(Vector2i(0, 0));
    bounds_aer.extend(
        Vector2i(block_aerial_.groups.at(photo_aerial.cid).width, block_aerial_.groups.at(photo_aerial.cid).height));

    // project the four corners to the aerial images
    BoundingBox2i bounds_sub;

    // the two sets of corners are used to estimate the homography for the transformation between patch and original
    // aerial image
    std::vector<Vector2d> corners_pat(4), corners_aer(4);
    for (int i = 0; i < 4; ++i) {
        Vector3d point = corners.col(i).cast<double>();
        Vector2d point2d = block_aerial_.project(point, iid_aerial);
        corners_aer[i] = point2d;

        bounds_sub.extend(Vector2i(point2d.x() + 1.5, point2d.y() + 1.5));
        bounds_sub.extend(Vector2i(point2d.x() - 0.5, point2d.y() - 0.5));
    }

    if (!bounds_aer.contains(bounds_sub)) {
        return std::tuple<cv::Mat, cv::Mat, Matrix3f>();
    }

    cv::Mat mat_sub;
    // reads M * sub = aer
    Matrix23d M_sub_aer;
    std::tie(M_sub_aer, mat_sub) = images_aerial_.at(iid_aerial)->get_patch(bounds_sub, bounds_sub.sizes());
    cv::Mat paper_mat_sub, paper_mat;
    paper_mat_sub = mat_sub.clone(); //experiments figure making

    if (mat_sub.channels() == 3) {
        cv::cvtColor(mat_sub, mat_sub, cv::COLOR_RGB2GRAY);
    }

    Matrix23d M_aer_sub = revers_affine(M_sub_aer);

    int patch_size = (param_.ncc_window / 2 + param_.ncc_search / 2) * 2 + 1;
    cv::Mat mat_pat = cv::Mat::zeros(patch_size, patch_size, CV_8U);

    BoundingBox2i bounds_pat;
    bounds_pat.extend(Vector2i(0, 0));
    bounds_pat.extend(Vector2i(patch_size, patch_size));
    for (int i = 0; i < 4; ++i) {
        corners_pat[i] = bounds_pat.corner(BoundingBox2i::CornerType(i)).cast<double>();
    }

    Matrix3d H_pat_aer = compute_homography(corners_pat, corners_aer);

    // warp images from the subset patch
    {
        std::vector<Vector2d> corners_sub(4);
        for (int i = 0; i < 4; ++i) {
            corners_sub[i] = M_aer_sub * corners_aer[i].homogeneous();
        }
        Matrix3d H_sub_pat = compute_homography(corners_sub, corners_pat);
       
        cv::Mat H;
        cv::eigen2cv(H_sub_pat, H);
        cv::warpPerspective(mat_sub, mat_pat, H, mat_pat.size(), cv::INTER_LINEAR);
        cv::warpPerspective(paper_mat_sub, paper_mat, H, mat_pat.size(), cv::INTER_LINEAR); 
    }

    return std::make_tuple(paper_mat_sub, paper_mat, H_pat_aer.cast<float>());
}

std::tuple<cv::Mat, cv::Mat> RenderMatcher::get_patch_on_ground_image(uint32_t iid, const Vector2d &point) {

    int patch_size = (param_.ncc_window / 2) * 2 + 1;
    int patch_half = patch_size / 2;

    uint32_t cid = block_ground_.photos.at(iid).cid;
    int rows = block_ground_.groups.at(cid).height;
    int cols = block_ground_.groups.at(cid).width;

    BoundingBox2i bounds_image;
    bounds_image.extend(Vector2i(0, 0));
    bounds_image.extend(Vector2i(cols, rows));

    BoundingBox2i bounds_patch;
    bounds_patch.extend(Vector2i(point.x() - 1.5 - patch_half, point.y() - 1.5 - patch_half));
    bounds_patch.extend(Vector2i(point.x() + 2.5 + patch_half, point.y() + 2.5 + patch_half));

    if (!bounds_image.contains(bounds_patch)) {
        return std::tie(cv::Mat(), cv::Mat());
    }

    Matrix23d M_sub_ima;
    cv::Mat sub, paper_sub;
    std::tie(M_sub_ima, sub) = images_ground_.at(iid)->get_patch(bounds_patch, bounds_patch.sizes());

    /*experiments figure making*/
    paper_sub = sub.clone();
    // cv::imwrite("result\\patch\\"+std::to_string(iid)+"\\ter" + std::to_string(iid) +"-"+
    // std::to_string(int(point.x()))+ ".jpg", paper_sub);

    if (sub.channels() == 3) {
        cv::cvtColor(sub, sub, cv::COLOR_RGB2GRAY);
    }
    Matrix23d M_ima_sub = revers_affine(M_sub_ima);
    Vector2d center = M_ima_sub * point.homogeneous();

    cv::Mat sub_extract;
    cv::getRectSubPix(sub, cv::Size(patch_size, patch_size), cv::Point2f(center.x(), center.y()), sub_extract);

    return std::make_tuple(paper_sub, sub_extract);
}

std::vector<uint32_t> RenderMatcher::search_visible_aerial_images(const MatrixXf &corners, const Vector3f &normal) {
    std::vector<uint32_t> vis_angles;

    Vector3f center = corners.rowwise().mean();

    double threshold = std::cos(param_.angle_difference * DEG2RAD);

    for (const auto &pair : block_aerial_.photos) {
        Vector3f C = pair.second.C.cast<float>();
        Vector3f dir = (C - center).normalized();
        if (normal.dot(dir) < threshold) {
            continue;
        }
        vis_angles.push_back(pair.first);
    }

    // TODO: do occlusion detection

    return vis_angles;
}

Vector3f RenderMatcher::depth_to_xyz(float depth, const Vector2i &point) {
    if (std::abs(depth - 1.0) < 1e-4) {
        return Vector3f::Constant(FLT_MAX);
    }

    Vector4f screen;
    screen(0) = (float)(point.x() + 0.5f) / viewport_.x();
    screen(1) = (float)(viewport_.y() - point.y() - 0.5) / viewport_.y();
    screen(2) = depth;
    screen(3) = 1.0;

    screen = screen.array() * 2.0f - 1.0f;
    Vector4f object = mvp_inverse_ * screen;
    Vector3f coord = object.hnormalized();

    return coord;
}



cv::Mat RenderMatcher::draw_matches(uint32_t iid_ground, uint32_t iid_aerial, const RenderMatchResults &matches, cv::Scalar color)
{
    std::vector<cv::KeyPoint> keys_ground, keys_aerial;
    std::vector<cv::DMatch> dmatches;

    int pos = 0;
    for (const auto &match : matches) {
        uint32_t iid = match.iid_ground;
        if (iid != iid_ground) {
            continue;
        }

        for (int i = 0; i < match.iid_aerial.size(); ++i) {
            uint32_t iid2 = match.iid_aerial[i];
            if (iid2 != iid_aerial) {
                continue;
            }

            Vector2f pt_aerial = match.pt_aerial[i];

            dmatches.push_back(cv::DMatch(pos, pos, 0.0f));
            keys_ground.push_back(cv::KeyPoint(match.pt_ground.x(), match.pt_ground.y(), 32.0f));
            keys_aerial.push_back(cv::KeyPoint(pt_aerial.x(), pt_aerial.y(), 32.0f));
            pos++;
        }
    }

    if (dmatches.size() == 0) {
        return cv::Mat();
    }

    std::string path_ground = block_ground_.photos.at(iid_ground).path;
    std::string path_aerial = block_aerial_.photos.at(iid_aerial).path;

    cv::Mat mat_ground = cv::imread(path_ground, cv::IMREAD_UNCHANGED);
    cv::Mat mat_aerial = cv::imread(path_aerial, cv::IMREAD_UNCHANGED);


    int crosssize = 180, thickness = 30;

	auto mat_ground_size = mat_ground.size();

    if (mat_ground.size != mat_aerial.size)
        cv::resize(mat_ground, mat_ground, cv::Size(mat_aerial.cols, mat_aerial.rows), 0, 0);

    cv::Mat ter_aer_mat;
    ter_aer_mat.create(std::max<int>(mat_ground.rows, mat_ground.rows), mat_ground.cols + mat_ground.cols,
                       mat_ground.type()); // des.create()
    cv::Mat r1 = ter_aer_mat(cv::Rect(0, 0, mat_ground.cols, mat_ground.rows));
    mat_ground.copyTo(r1);
    cv::Mat r2 = ter_aer_mat(cv::Rect(mat_ground.cols, 0, mat_ground.cols, mat_ground.rows));
    mat_aerial.copyTo(r2);

    cv::Mat ter_aer_mat_vertical;
    ter_aer_mat_vertical.create(mat_aerial.rows + mat_aerial.rows, mat_aerial.cols,
                                mat_aerial.type()); // des.create()
    cv::Mat r11 = ter_aer_mat_vertical(cv::Rect(0, 0, mat_aerial.cols, mat_aerial.rows));
    mat_ground.copyTo(r11);
    cv::Mat r22 = ter_aer_mat_vertical(cv::Rect(0, mat_aerial.rows, mat_aerial.cols, mat_aerial.rows));
    mat_aerial.copyTo(r22);

    int countKey = 0;
    for (const auto &match : matches) {
        auto key_aerial = match.pt_aerial;
        auto key_ground = match.pt_ground;

        for (int i = 0; i < key_aerial.size(); i++) {
            if (match.iid_aerial[i] == iid_aerial) {
                // cv::line(mat_aerial, cv::Point(key_aerial[i].x() - crosssize / 2, key_aerial[i].y()),
                //         cv::Point(key_aerial[i].x() + crosssize / 2, key_aerial[i].y()), color, thickness, 8, 0);
                ////绘制竖线
                // cv::line(mat_aerial, cv::Point(key_aerial[i].x(), key_aerial[i].y() - crosssize / 2),
                //         cv::Point(key_aerial[i].x(), key_aerial[i].y() + crosssize / 2), color, thickness, 8, 0);

                line(ter_aer_mat,
                     cv::Point(key_ground.x() * mat_aerial.cols / mat_ground_size.width,
                               key_ground.y() * mat_aerial.rows / mat_ground_size.height),
                     cv::Point(key_aerial[i].x() + mat_aerial.cols, key_aerial[i].y()), color, 7);

                line(ter_aer_mat_vertical,
                     cv::Point(key_ground.x() * mat_aerial.cols / mat_ground_size.width,
                               key_ground.y() * mat_aerial.rows / mat_ground_size.height),
                     cv::Point(key_aerial[i].x(), key_aerial[i].y() + mat_aerial.rows), color, 7);

                circle(mat_ground,
                       cv::Point(key_ground.x() * mat_aerial.cols / mat_ground_size.width,
                                 key_ground.y() * mat_aerial.rows / mat_ground_size.height),
                       5, color, 5);
                circle(mat_aerial, cv::Point(key_aerial[i].x(), key_aerial[i].y()), 2, color, 3);

                countKey++;
            }
        }
    }
	std::cout << "matches number：" << countKey << std::endl;
    return ter_aer_mat;
}

#include <windows.h>
#include <math.h>
#include <time.h>

#include <iostream>

using namespace std;
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
using namespace cv;
#include "my_function.h"
#include "sift.h"

int main() {
    //加载两幅图片
    Mat src1 = imread("F:\\ylab\\image database\\camera\\obj01_001.jpg");
    Mat src2 = imread("F:\\ylab\\image database\\imagesTest2\\test01_.jpg");

    //这四个坐标是模板图像中绿色方框的四个顶点
    Point2f m1(173.0, 0.0), m2(168.0, 464.0), m3(507.0, 464.0), m4(499.0, 0.0);
    std::vector<Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(173.0, 0.0);
    obj_corners[1] = cvPoint(168.0, 464.0);
    obj_corners[2] = cvPoint(507.0, 464.0);
    obj_corners[3] = cvPoint(499.0, 0.0);

    //原始图片比较大，我这里将图片同一处理成了640*480的大小
    Size certainsize = Size(640, 480);
    Mat src_1;
    Mat src_2;
    resize(src1, src_1, certainsize);
    resize(src2, src_2, certainsize);

    //两个图像的特征点序列
    Vector<Keypoint> feature_1, feature_2;

    //采用sift算法，计算特征点序列，这个SIFT函数是在另外的文件中写好的
    Sift(src_1, feature_1, 1.6);
    Sift(src_2, feature_2, 1.6);

    // feature_dis为带有距离的特征点结构体序列
    Vector<Key_point> feature_dis_1;
    Vector<Key_point> feature_dis_2;
    Vector<Key_point> result;

    //对特征点进行匹配，这个Match_feature是我自己写的，就是采用最近比次近小于0.8即为合适的匹配，这种匹配方式
    // openCV中并没有，所以我就自己写了
    Match_feature(feature_1, feature_2, feature_dis_1, feature_dis_2);

    printf("The number of features is %d\n", feature_1.size());
    printf("The number of the match features is %d\n", feature_dis_1.size());

    //从这里开始使用RANSAC方法进行运算
    //下面的程序都好无奈，所有的结构都只能转化成openCV的类型才能用openCV的函数。。
    Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create("BruteForce"); //创建特征匹配器
    int count = feature_dis_1.size();

    //把特征点序列转化成openCV能够使用的类型
    vector<KeyPoint> keypoints1, keypoints2;
    KeyPoint keyp;
    for (int i = 0; i < count; i++) {
        keyp.pt.x = feature_dis_1[i].dx;
        keyp.pt.y = feature_dis_1[i].dy;
        keypoints1.push_back(keyp);
        keyp.pt.x = feature_dis_2[i].dx;
        keyp.pt.y = feature_dis_2[i].dy;
        keypoints2.push_back(keyp);
    }

    Mat descriptors1(count, FEATURE_ELEMENT_LENGTH, CV_32F);
    Mat descriptors2(count, FEATURE_ELEMENT_LENGTH, CV_32F);

    for (int i = 0; i < count; i++) {
        for (int j = 0; j < FEATURE_ELEMENT_LENGTH; j++) {
            descriptors1.at<float>(i, j) = feature_dis_1[i].descriptor[j];
            descriptors2.at<float>(i, j) = feature_dis_2[i].descriptor[j];
        }
    }

    Mat img_match;
    vector<DMatch> matches;
    descriptor_matcher->match(descriptors1, descriptors2, matches);
    Mat img_matches;
    drawMatches(src_1, keypoints1, src_2, keypoints2, matches, img_matches);
    //其实我前面已经完成匹配了，到这里，用openCV自带的方式重新匹配了一遍，并且显示了一下
    imshow("SIFT", img_matches);
    // imwrite("F:\\ylab\\CSDN_image\\3.jpg",img_matches);

    Mat p1(feature_dis_1.size(), 2, CV_32F);
    Mat p2(feature_dis_1.size(), 2, CV_32F);
    for (int i = 0; i < feature_dis_1.size(); i++) {
        p1.at<float>(i, 0) = feature_dis_1[i].dx;
        p1.at<float>(i, 1) = feature_dis_1[i].dy;
        p2.at<float>(i, 0) = feature_dis_2[i].dx;
        p2.at<float>(i, 1) = feature_dis_2[i].dy;
    }
    // 用RANSAC方法计算F
    Mat m_Fundamental;
    // 上面这个变量是基本矩阵
    vector<uchar> m_RANSACStatus;
    // 上面这个变量已经定义过，用于存储RANSAC后每个点的状态

    //一开始使用findFundamentalMat函数，发现可以消除错误匹配，实现很好的效果，但是
    //就是函数返回值不是变换矩阵，而是没有什么用的基础矩阵
    m_Fundamental = findFundamentalMat(p1, p2, m_RANSACStatus, CV_FM_RANSAC);

    //这里使用findHomography函数，这个函数的返回值才是真正的变换矩阵
    Mat m_homography;
    vector<uchar> m;
    m_homography = findHomography(p1, p2, CV_RANSAC, 3, m);
	findHomography()

    //由变换矩阵，求得变换后的物体边界四个点
    std::vector<Point2f> scene_corners(4);
    perspectiveTransform(obj_corners, scene_corners, m_homography);
    line(src_2, scene_corners[0], scene_corners[1], Scalar(0, 0, 255), 2);
    line(src_2, scene_corners[1], scene_corners[2], Scalar(0, 0, 255), 2);
    line(src_2, scene_corners[2], scene_corners[3], Scalar(0, 0, 255), 2);
    line(src_2, scene_corners[3], scene_corners[0], Scalar(0, 0, 255), 2);

    int nr = m_Fundamental.rows;                            // number of rows
    int nc = m_Fundamental.cols * m_Fundamental.channels(); // total number of elements per line

    // 计算野点个数
    int OutlinerCount = 0;
    for (int i = 0; i < Count; i++) {
        if (m_RANSACStatus[i] == 0) // 状态为0表示野点
        {
            OutlinerCount++;
        }
    }

    // 计算内点
    vector<Point2f> m_LeftInlier;
    vector<Point2f> m_RightInlier;
    vector<DMatch> m_InlierMatches;
    // 上面三个变量用于保存内点和匹配关系
    int ptCount = (int)matches.size();
    int InlinerCount = ptCount - OutlinerCount;
    m_InlierMatches.resize(InlinerCount);
    m_LeftInlier.resize(InlinerCount);
    m_RightInlier.resize(InlinerCount);
    InlinerCount = 0;
    for (int i = 0; i < ptCount; i++) {
        if (m_RANSACStatus[i] != 0) {
            m_LeftInlier[InlinerCount].x = p1.at<float>(i, 0);
            m_LeftInlier[InlinerCount].y = p1.at<float>(i, 1);
            m_RightInlier[InlinerCount].x = p2.at<float>(i, 0);
            m_RightInlier[InlinerCount].y = p2.at<float>(i, 1);
            m_InlierMatches[InlinerCount].queryIdx = InlinerCount;
            m_InlierMatches[InlinerCount].trainIdx = InlinerCount;
            InlinerCount++;
        }
    }
    //   //printf("最终的匹配点个数为：%d\n",InlinerCount);
    //// 把内点转换为drawMatches可以使用的格式
    vector<KeyPoint> key1(InlinerCount);
    vector<KeyPoint> key2(InlinerCount);
    KeyPoint::convert(m_LeftInlier, key1);
    KeyPoint::convert(m_RightInlier, key2);

    // 显示计算F过后的内点匹配
    // Mat m_matLeftImage;
    // Mat m_matRightImage;
    // 以上两个变量保存的是左右两幅图像

    line(src_1, m1, m2, Scalar(0, 255, 0), 2);
    line(src_1, m2, m3, Scalar(0, 255, 0), 2);
    line(src_1, m3, m4, Scalar(0, 255, 0), 2);
    line(src_1, m4, m1, Scalar(0, 255, 0), 2);

    Mat OutImage;
    drawMatches(src_1, key1, src_2, key2, m_InlierMatches, OutImage);
    imshow("SIFT_RANSAC", OutImage);
    // imwrite("F:\\ylab\\CSDN_image\\5.jpg",OutImage);
    cvWaitKey(0);
    return 0;
}


#ifndef FRAME_H
#define FRAME_H

//eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
//opencv
#include <opencv2/core.hpp>
#include <opencv2/features2d/features2d.hpp>

//DBOW3
#include <DBoW3/DBoW3.h>
#include <sophus/se3.h>
#include <vector>

class Frame
{
public:
    Frame(){}
    Frame(cv::Mat img,cv::Mat depth);
    void detectKeyPoints();
    void computeDescriptors();
    void ComputekeyPtsPw();
    void setPose(Sophus::SE3 mTwc);
    Sophus::SE3 getPose();

    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    Sophus::SE3 mTwc;
    cv::Mat frame;
    cv::Mat depth_;
    std::vector<Eigen::Vector3d> pts_3d;
    std::vector<Eigen::Vector3d> pts_3d_cam; //相机坐标系下的点P_c,P_w = T_wc*P_c
    static cv::Ptr<cv::ORB> detector_ORB;//
    static cv::FlannBasedMatcher matcher;// = cv::flann::LshIndexParams(5,10,2);// = DescriptorMatcher::create("BruteForce-Hamming")
    DBoW3::BowVector BowVec;
};

#endif 
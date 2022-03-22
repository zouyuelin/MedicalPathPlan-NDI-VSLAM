#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

class camera
{
public:
    typedef std::shared_ptr<camera> Ptr;
    camera(){}

    static cv::FileStorage configfile;
    static cv::Mat K;
    static Eigen::Matrix3d K_Eigen;
    static cv::Mat distCoeffs;
    static float depthScale;

    static void loadYaml(std::string path);
    static void undistort(cv::Mat &frame,cv::Mat &output);

    static cv::Point3f pixel2cam(const cv::Point2d &p, const cv::Mat &K);
    static cv::Point2f cam2pixel(const cv::Point3f &p, const cv::Mat &K);
    static Eigen::Vector3d ToEulerAngles(Eigen::Quaterniond q);

};

#endif 
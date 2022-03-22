#include "camera.h"
#include <iostream>
#include <math.h>

cv::FileStorage camera::configfile = cv::FileStorage();
//cv::Mat camera::K = (cv::Mat_<double>(3, 3) << 603.47, 0, 314.912, 0, 600.49, 234.941, 0, 0, 1);
cv::Mat camera::K = (cv::Mat_<double>(3, 3) << 517.3, 0, 325.1, 0, 516.5, 249.7, 0, 0, 1); //fr1
//K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);//fr2
Eigen::Matrix3d camera::K_Eigen = Eigen::Matrix3d::Identity(3,3);
cv::Mat camera::distCoeffs = cv::Mat::zeros(5,1,CV_64F);
float camera::depthScale = 1000.0;

void camera::loadYaml(std::string path)
{
    configfile.open(path,cv::FileStorage::READ);
    if(!configfile.isOpened())
    {
        std::cout<<"please confirm the path of yaml!\n";
        return ;
    }

    double fx = configfile["Camera.fx"];
    double fy = configfile["Camera.fy"];
    double cx = configfile["Camera.cx"];
    double cy = configfile["Camera.cy"];

    double k1 = configfile["Camera.k1"];
    double k2 = configfile["Camera.k2"];
    double p1 = configfile["Camera.p1"];
    double p2 = configfile["Camera.p2"];
    double k3 = configfile["Camera.k3"];

    K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    K_Eigen<<fx,   0,   cx,
              0,  fy,   cy,
              0,   0,    1;

    distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
}

inline void camera::undistort(cv::Mat &frame,cv::Mat &output)
{
    cv::undistort(frame ,output ,camera::K,camera::distCoeffs);
}

cv::Point3f camera::pixel2cam(const cv::Point2d &p, const cv::Mat &K)
{
  return cv::Point3f
    (
      (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
      (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1),
              1
    );
}

cv::Point2f camera::cam2pixel(const cv::Point3f &p, const cv::Mat &K)
{
    float d = p.z;
    return cv::Point2f( K.at<double>(0, 0) * p.x / d + K.at<double>(0, 2),
                    K.at<double>(1, 1) * p.x / d + K.at<double>(1, 2));
}

Eigen::Vector3d camera::ToEulerAngles(Eigen::Quaterniond q) {
    ///
    /// roll = atan2( 2(q0*q1+q2*q3),1-2(q1^2 + q2^2))
    /// pitch = asin(2(q0*q2 - q3*q1))
    /// yaw = atan2(2(q0*q3+q1*q2),1-2(q2^2 + q3^2))
    ///
    double x,y,z;//roll pitch yaw

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        y = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    z = std::atan2(siny_cosp, cosy_cosp);

    return Eigen::Vector3d(x,y,z);
}
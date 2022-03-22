#ifndef VOSLAM_H
#define VOSLAM_H

#include "camera.h"
#include "frame.h"
#include "map.h"
#include "loopclosing.h"
#include "optimizer.h"
#include "visualmap.h"
#include "pointcloudmapping.h"

using namespace cv;

class VO_slam
{
public:
    typedef std::shared_ptr<VO_slam> Ptr;
    enum VOstate{
        INITIALIZING = 0,
        OK = 1,
        LOST = -1
    };
    enum UPDATEPOSEMETHOD{
        UPDATATCW = 0,
        OPTIMIZERTWC = 1
    };
    VOstate state_;

    VO_slam(const std::string vocabPath);

    Sophus::SE3 tracking(Mat img,Mat depth);

    void featureMatch(Mat descriptors_ref,Mat descriptors_cur);

    void SiftMapPoints(Frame frame_ref, Frame frame_cur,
                           std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &pts_3d_eigen,
                           std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &pts_2d_eigen);

    //非线性优化
    void PoseOptimizer(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &pts_3d_eigen,
                                std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &pts_2d_eigen,
                                Mat &K, Sophus::SE3 &pose);
    void BAOptimizer(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &pts_3d_eigen,
                        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &pts_2d_eigen,
                        Mat &K, Sophus::SE3 &pose);
    //利用opencv求解pnp
    bool solvePnPOpencv(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &pts_3d_eigen,
                        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &pts_2d_eigen,
                        Mat &K, Sophus::SE3 &pose);

    bool pnpSolver(Frame frame_ref, Frame frame_cur, Sophus::SE3 &Curpose);
    bool checkEstimatedPose(Sophus::SE3 Twc_ref, Sophus::SE3 Twc_cur);
    bool checkKeyFrame();

    Map::Ptr map_;
    visualMap::Ptr visual_;
    LoopClosing::Ptr loopclose_;
    std::shared_ptr<PointCloudMapping>  PointCloudMapping_;
    std::vector<DMatch> featureMatches;
    double pose_max_angle;  //keyFrame 和 pose的筛选机制差不多，只是pose的为了抑制较大的误差，条件较小，keyFrame 尽量位移大一些；
    double pose_max_trans;
    double keyFrame_min_trans;
    double KeyFrame_max_trans;
    int num_trackingTry;
    UPDATEPOSEMETHOD updatePoseMethod;

    std::shared_ptr<std::thread> viewerThread;
    std::shared_ptr<std::thread> loopThread;
    std::mutex mMutex;

    DBoW3::Vocabulary vocab;
};


#endif 

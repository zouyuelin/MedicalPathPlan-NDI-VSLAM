#ifndef VISUALMAP_H
#define VISUALMAP_H

#include <memory>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Eigen>
//pangolin
#include "camera.h"
#include <pangolin/pangolin.h>

class visualMap
{
public:
    typedef std::shared_ptr<visualMap> Ptr;
    visualMap();

    void run();
    void GetPoses(Eigen::Isometry3d Twc);
    void GetCurrentTwc(Eigen::Isometry3d CurrentTwc);
    void GetCurrentPointsInCamera(std::vector<Eigen::Vector3d> mpts);
    void clearPoses();

protected:
    void view();
    //画当前位姿
    void drawCurrentFrame(Eigen::Isometry3d poses);
    //画关键帧
    void drawKeyFrame(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> keyframs);
    //画动态坐标轴
    void drawAxis(Eigen::Isometry3d poses);
    //画原点坐标系
    void drawCoordinate(float scale);
    //画轨迹线
    void drawLine(size_t i, std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses, bool drawLine);
    //求解移动速度
    void getVelocity(Eigen::Isometry3d &pose_last,Eigen::Isometry3d &pose_next,double &time_used,Eigen::Vector4d &trans_velocity,Eigen::Vector3d &angluar_velocity);

    void drawing();
    void drawPoints();

public:
    const float baseSize = 0.4;
    const float w = baseSize*0.06;
    const float h = w*0.75;
    const float z = w*0.6;
    const bool drawline = true;
    const long int drawGaps = 5*100;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> keyposes;
    Eigen::Isometry3d mTwc;
    std::vector<Eigen::Vector3d> currentPointsInCamera;
    std::mutex mMutex;
};

#endif 
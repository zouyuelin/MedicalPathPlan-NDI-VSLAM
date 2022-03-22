#ifndef POINTCLOUD_H
#define POINTCLOUD_H

//PCL
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

#include <mutex>
#include <memory>
#include <thread>
#include <condition_variable>

#include "camera.h"
#include "frame.h"


class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping( double resolution_ );

    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame(Frame kf);
    void shutdown();
    void viewer();

protected:
    PointCloud::Ptr generatePointCloud(Frame kf, cv::Mat& color, cv::Mat& depth);

    PointCloud::Ptr globalMap;
    std::shared_ptr<std::thread>  viewerThread;

    bool    shutDownFlag    =false;
    std::mutex   shutDownMutex;

    std::condition_variable  keyFrameUpdated;
    std::mutex               keyFrameUpdateMutex;

    // data to generate point clouds
    std::vector<Frame>       keyframes;
    std::vector<cv::Mat>         colorImgs;
    std::vector<cv::Mat>         depthImgs;
    std::mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;

    double resolution = 0.04;
    pcl::VoxelGrid<PointT>  voxel;
};

#endif 
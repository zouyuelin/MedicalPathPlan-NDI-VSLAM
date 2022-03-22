#ifndef MAP_H
#define MAP_H

#include <thread>
#include <memory>
#include <vector>
#include <unordered_map>
#include <sophus/se3.h>

#include "frame.h"

class Map
{
public:
    Map(){}
    typedef std::shared_ptr<Map> Ptr; //定义智能指针，简化了实例化过程
    void addKeyFrames(Frame frame);
    void add3Dpts_world(std::vector<Eigen::Vector3d> pts_3d_Pw);

    std::unordered_map<unsigned long,std::vector<Eigen::Vector3d> > map_points;
    std::vector<Frame> keyFrames;
    Frame frame_ref;
    Sophus::SE3 pose_cur_Twc;
};

#endif 
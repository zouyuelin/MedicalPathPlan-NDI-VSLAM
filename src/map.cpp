#include "map.h"

void Map::addKeyFrames(Frame frame)
{
    keyFrames.push_back(frame);
}


void Map::add3Dpts_world(std::vector<Eigen::Vector3d> pts_3d_Pw)
{
    map_points.insert(std::make_pair((unsigned long)(keyFrames.size()-1),pts_3d_Pw));
}
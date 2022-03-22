#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <sophus/se3.h>
#include <vector>

#include "frame.h"

class Optimizer
{
public:
    static void PoseOptimization(Sophus::SE3 pose,std::vector<Frame> keyFrames);
    static void BAOptimization(std::vector<Frame> keyFrames);
};


#endif 
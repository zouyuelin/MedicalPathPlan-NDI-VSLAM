#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include <vector>
//DBOW3
#include <DBoW3/DBoW3.h>
#include <memory>

#include "frame.h"

class LoopClosing
{
public:
    typedef std::shared_ptr<LoopClosing> Ptr;
    LoopClosing(DBoW3::Vocabulary vocabulary);

    void getKeyFrames(std::vector<Frame> mkeyframe);
    void addDataset(DBoW3::BowVector vectORB);
    void DetectLoop();
    void run();

protected:
    std::vector<Frame> mkeyFrames;
    Frame currentFrame;
    std::mutex mMutex;
    DBoW3::Database morbDatabase;
};

#endif 
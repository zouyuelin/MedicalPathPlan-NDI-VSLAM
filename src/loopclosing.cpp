#include "loopclosing.h"

LoopClosing::LoopClosing(DBoW3::Vocabulary vocabulary)
{
    morbDatabase = DBoW3::Database(vocabulary,false,0);
}

void LoopClosing::run()
{
    while(true)
    {
        std::unique_lock<std::mutex> lock(this->mMutex);
        DetectLoop();

    }
}

void LoopClosing::DetectLoop()
{
    std::unique_lock<std::mutex> lock(this->mMutex);
    Frame currentKeyFrame = mkeyFrames.back();

    DBoW3::QueryResults ret;
    morbDatabase.query(currentFrame.BowVec, ret, 4);

    addDataset(currentKeyFrame.BowVec);
}

void LoopClosing::addDataset(DBoW3::BowVector vectORB)
{
    std::unique_lock<std::mutex> lock(this->mMutex);
    morbDatabase.add(vectORB);
}

void LoopClosing::getKeyFrames(std::vector<Frame> mkeyframe)
{
//    unique_lock<mutex> lock(this->mMutex);
    mkeyFrames = mkeyframe;
}
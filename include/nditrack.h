#ifndef NDITRACK_H
#define NDITRACK_H

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ToolData.h"
#include <ndicapi.h>

#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <functional>
#include <sophus/se3.h>

using namespace std;

class NDItracking
{
public:
    typedef shared_ptr<NDItracking> Ptr;
    NDItracking(const std::string portname);
    bool FinishTrac();
    Sophus::SE3 getPoseTcw();
    enum STATE{
        RUNNING = 1,
        STOP = -1,
        PREPRING = 0
    };

protected:
    void checkTheUSBPortName(string &name);
    void run();
    void getThePostion(const ToolData& toolData);

private:
    Sophus::SE3 mTcw;
    std::string PortName;
    std::shared_ptr<std::thread> tracking;
    std::mutex mMutex;

    bool finished = false;
    Eigen::Matrix3d transformMatrix;
    STATE state;


};


#endif 

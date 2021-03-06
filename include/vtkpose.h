#ifndef NDITRACKING_H
#define NDITRACKING_H

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include "pathSetting.h"
#include "cameraRender.h"

class vtkPose
{
public:
    typedef std::shared_ptr<vtkPose> Ptr;
    vtkPose(string stlPath_);
    ~vtkPose();
    void run();
    void getPose(Sophus::SE3 mTcw_);
    void getTheStyle( MouseInteractorStyleCenterline* style_);
    void getTheVTKcommand( vtkMyCommand *vtkCommand_ );

    bool IsFinished();

private:
    std::mutex mMutex;
    std::mutex mMutexF;

    Eigen::Isometry3d mTcw;
    Eigen::Matrix3d Rotation;
    Eigen::Vector3d Translation;

    std::shared_ptr<std::thread> vtkRunning;
    std::shared_ptr<std::thread> pathRender;
    pathSetting* modelPathSet;
    MouseInteractorStyleCenterline* m_style = nullptr;
    vtkMyCommand *m_vtkCommand = nullptr;

};

#endif

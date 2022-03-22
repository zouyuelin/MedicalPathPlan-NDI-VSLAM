#include "vtkpose.h"

#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>

#include <vtkJPEGReader.h>
#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkAxesActor.h>
#include <vtkSTLReader.h>

#include <vtkDiscreteMarchingCubes.h>
#include <vtkWindowedSincPolyDataFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkIdList.h>
#include <vtkDecimatePro.h>
#include <vtkDoubleArray.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkLODActor.h>
#include <vtkKochanekSpline.h>

#include <vtkvmtkCapPolyData.h>
#include <vtkvmtkPolyDataCenterlines.h>

#include <unistd.h>

vtkPose::vtkPose(string stlPath_):mTcw(Eigen::Isometry3d::Identity())
{
    Rotation = Eigen::Matrix3d::Identity();
    modelPathSet = new pathSetting(stlPath_);

    vtkRunning = std::make_shared<std::thread>(std::bind(&vtkPose::run,this));
    pathRender = std::make_shared<std::thread>(std::bind(&pathSetting::run,modelPathSet));

    getTheStyle(modelPathSet->style);
    getTheVTKcommand(modelPathSet->mycommand);
}

vtkPose::~vtkPose()
{
    delete modelPathSet;
    delete m_vtkCommand;
    delete m_style;
}

void vtkPose::getPose(Sophus::SE3 mTcw_)
{
    std::unique_lock<std::mutex> lc(mMutex);
    Rotation = mTcw_.rotation_matrix();
    Translation = mTcw_.translation();
    mTcw.prerotate(Rotation);
    mTcw.pretranslate(Translation);
}

bool vtkPose::IsFinished()
{
    std::unique_lock<std::mutex> lc(mMutexF);
    return m_style->IsFinished();
}

void vtkPose::getTheStyle( MouseInteractorStyleCenterline* style_)
{
    m_style = style_;
}

void vtkPose::getTheVTKcommand( vtkMyCommand *vtkCommand_ )
{
    m_vtkCommand = vtkCommand_;
}

void vtkPose::run()
{

    Eigen::Vector3d InitPostion(0,0,-300);
    double ratio = 1000;

    while (!m_style->IsFinished()) {

        if(m_style->IsReady())
        {
            while( m_style->IsRenderring() == false)
            {
                Eigen::Vector3d position,focalPoint,up;
                {
                    std::unique_lock<std::mutex> lc(mMutex);
                    position = InitPostion - ratio * Rotation.transpose() * Translation;
                    focalPoint = position + Rotation.row(2).transpose();
                    up = -Rotation.row(1);
                }
                m_vtkCommand->setCameraInfo(position,focalPoint,up);
            }
        }
    }

    std::cout<<"The vtk pose of ndi is done !"<<std::endl;
}


#include "vtkpose.h"
#include <unistd.h>

vtkPose::vtkPose(string stlPath_, string texture)
{
    Rotation = Eigen::Matrix3d::Identity();
    modelPathSet = new pathSetting(stlPath_,texture);

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

void vtkPose::getPoseTcw(Sophus::SE3 mTcw_)
{
    std::unique_lock<std::mutex> lc(mMutex);
    Rotation = mTcw_.rotation_matrix();
    Translation = ratio * mTcw_.translation();
    mTcw = Sophus::SE3(Rotation,Translation);
}

bool vtkPose::IsFinished()
{
    std::unique_lock<std::mutex> lc(mMutexF);
    return m_style->IsFinished();
}

bool vtkPose::RequireReset()
{
    return m_style->RequireReset();
}

void vtkPose::SetResetStatus(bool status)
{
    m_style->SetResetStatus(status);
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

    Sophus::SE3 InitPos;

    while (true) {

        if(IsFinished())
        {
            break;
        }

        if(m_style->IsReady())
        {
            while( m_style->IsRenderring() == false)
            {
                Eigen::Vector3d position,focalPoint,up;
                Eigen::Matrix4d se3;
                {
                    {
                        std::unique_lock<std::mutex> lc(mMutex);
                        InitPos = m_style->GetInitPosition() * mTcw.inverse();//Twvcv*Tcrcv.inverse()=Twvcr
                    }

//                    position = - InitPos.rotation_matrix().transpose() * InitPos.translation();
//                    focalPoint = position + InitPos.rotation_matrix().row(2).transpose();
//                    up = -InitPos.rotation_matrix().row(1);
                    position = InitPos.translation();
                    focalPoint = position + InitPos.rotation_matrix().col(2);
                    up = -InitPos.rotation_matrix().col(1);
                }
//                std::cout<<InitPos.matrix()<<std::endl;
//                std::cout<<"mTcw"<<endl<<mTcw.matrix()<<endl;
//                std::cout<<"mTwcv"<<endl<<m_style->GetInitPosition().matrix()<<endl;
                m_vtkCommand->setCameraInfo(position,focalPoint,up);
                usleep(20 * 1000);
            }
        }
    }

    std::cout<<"The vtk pose of ndi is done !"<<std::endl;
}


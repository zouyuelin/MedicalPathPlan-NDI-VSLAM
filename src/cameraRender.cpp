#include "cameraRender.h"

void vtkMyCommand::cameraRenderThread(Eigen::Vector3d position, Eigen::Vector3d focalPoint, Eigen::Vector3d up)
{
    if(CenterlinePathSet != nullptr)
    {
        CenterlinePathSet->castCamera(position,focalPoint,up);
    }
}

void vtkMyCommand::setTheStyle(MouseInteractorStyleCenterline *CenterlinePathSet_)
{
    CenterlinePathSet = CenterlinePathSet_;
}

void vtkMyCommand::Execute(vtkObject *caller, unsigned long eventId, void* callData)
{
    if(CenterlinePathSet->rendering == false)
    {
        Eigen::Vector3d position,focalpoint,up;
        {
            std::unique_lock<std::mutex> lc(mMutexRender);
            position = m_position;
            focalpoint = m_focalpoint;
            up = m_up;
        }
        cameraRenderThread(position,focalpoint,up);
    }
}

void vtkMyCommand::setCameraInfo(Eigen::Vector3d position,Eigen::Vector3d focalPoint,Eigen::Vector3d up)
{
    std::unique_lock<std::mutex> lc(mMutexRender);
    m_position = position;
    m_focalpoint = focalPoint;
    m_up = up;
}

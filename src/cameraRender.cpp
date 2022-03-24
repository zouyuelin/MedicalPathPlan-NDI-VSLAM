#include "cameraRender.h"
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>

void vtkMyCommand::cameraRenderThread(Eigen::Vector3d position, Eigen::Vector3d focalPoint, Eigen::Vector3d up)
{
    if(CenterlinePathSet != nullptr)
    {
        if(CenterlinePathSet->camType == MouseInteractorStyleCenterline::CAMERA_TYPE::IN_HAND)
        {
            CenterlinePathSet->castCamera(position,focalPoint,up);
        }
        else if (CenterlinePathSet->camType == MouseInteractorStyleCenterline::CAMERA_TYPE::NOT_IN_HAND)
        {
            CenterlinePathSet->cameraSensor->SetCenter(position[0],position[1],position[2]);
            CenterlinePathSet->GetInteractor()->Render();
        }
    }
}

void vtkMyCommand::setTheStyle(MouseInteractorStyleCenterline *CenterlinePathSet_)
{
    CenterlinePathSet = CenterlinePathSet_;
}

void vtkMyCommand::Execute(vtkObject *caller, unsigned long eventId, void* callData)
{
    if(CenterlinePathSet->IsRenderring() == false)
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

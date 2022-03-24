#ifndef CAMERARENDER_H
#define CAMERARENDER_H

#include <Eigen/Core>
#include <Eigen/Eigen>
#include "interactorStyleCenterline.h"

class MouseInteractorStyleCenterline;

class vtkMyCommand : public vtkCommand
{
public:
    vtkTypeMacro(vtkMyCommand,vtkCommand);
    static vtkMyCommand *New()
    {
        return new vtkMyCommand;
    }
    virtual void Execute(vtkObject *caller, unsigned long eventId, void* callData);
    void setTheStyle(MouseInteractorStyleCenterline * CenterlinePathSet_);
    void setCameraInfo(Eigen::Vector3d position, Eigen::Vector3d focalPoint, Eigen::Vector3d up);

protected:
    void cameraRenderThread(Eigen::Vector3d position,Eigen::Vector3d focalPoint,Eigen::Vector3d up);

private:
    MouseInteractorStyleCenterline * CenterlinePathSet = nullptr;
    Eigen::Vector3d m_position;
    Eigen::Vector3d m_focalpoint;
    Eigen::Vector3d m_up;
    std::mutex mMutexRender;
};



#endif // CAMERARENDER_H

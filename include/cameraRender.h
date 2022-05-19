#ifndef CAMERARENDER_H
#define CAMERARENDER_H

#include <Eigen/Core>
#include <Eigen/Eigen>
#include "interactorStyleCenterline.h"

#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkPolyDataNormals.h>
#include <vtkKdTree.h>

class MouseInteractorStyleCenterline;

class vtkMyCommand : public vtkCommand
{
public:
    vtkMyCommand();
    vtkTypeMacro(vtkMyCommand,vtkCommand);
    static vtkMyCommand *New()
    {
        return new vtkMyCommand;
    }
    virtual void Execute(vtkObject *caller, unsigned long eventId, void* callData);
    void setTheStyle(MouseInteractorStyleCenterline * CenterlinePathSet_);
    void setThesurfaceForCollDetection(vtkPolyData* surface_);
    void setThenormals(vtkPolyDataNormals* normals_);
    void setCameraInfo(Eigen::Vector3d position, Eigen::Vector3d focalPoint, Eigen::Vector3d up);

protected:
    void cameraRenderThread(Eigen::Vector3d position,Eigen::Vector3d focalPoint,Eigen::Vector3d up);
    void collisionDetection(Eigen::Vector3d &position,Eigen::Vector3d &focalPoint,Eigen::Vector3d &up);
    Eigen::Vector3d getTheClosetPointNorm(double x,double y,double z);
    Eigen::Vector3d getTheClosetPoint(double x,double y,double z);
    void recordThePath(Eigen::Vector3d position,Eigen::Vector3d lastpoint);

private:
    MouseInteractorStyleCenterline * CenterlinePathSet = nullptr;
    Eigen::Vector3d m_position;
    Eigen::Vector3d m_focalpoint;
    Eigen::Vector3d m_up;
    Eigen::Vector3d m_lastPosition;
    Eigen::Vector3d m_lastPosition_change;
    std::mutex mMutexRender;

    vtkSmartPointer<vtkSelectEnclosedPoints> selectEnclosedPoints;
    vtkPolyData* surface;
    vtkPolyDataNormals* normals = nullptr;
    vtkKdTree * kDTreePoints;

    std::ofstream trackNDI;
    std::ofstream collD;
    std::ofstream collDAndNDI;

    Eigen::Vector3d n_lastPosition_pre;
    Eigen::Vector3d n_lastPosition_colld;

    bool collision = false;
    bool lastPositionInside = true;
};



#endif // CAMERARENDER_H

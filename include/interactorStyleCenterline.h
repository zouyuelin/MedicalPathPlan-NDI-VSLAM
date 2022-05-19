#ifndef INTERACTORSTYLECENTERLINE_H
#define INTERACTORSTYLECENTERLINE_H

#include <vtkPolyData.h>
#include <vtkIdList.h>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPointData.h>
#include <vtkObjectFactory.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkCoordinate.h>
#include <vtkKdTreePointLocator.h>
#include <vtkSphereSource.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkActorCollection.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkKochanekSpline.h>
#include <vtkCamera.h>
#include <map>

#include "Centerline.h"
#include <vtkLODActor.h>
#include <thread>
#include <memory>
#include <functional>
#include <mutex>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <sophus/se3.h>

class MouseInteractorStyleCenterline : public vtkInteractorStyleTrackballCamera
{
public:
	static MouseInteractorStyleCenterline* New();
    MouseInteractorStyleCenterline();
	vtkTypeMacro(MouseInteractorStyleCenterline, vtkInteractorStyleTrackballCamera);

    enum CAMERA_TYPE{
        IN_HAND = 0,
        NOT_IN_HAND = 1
    };
    CAMERA_TYPE camType;

	void OnKeyPress();
    void OnLeftButtonDown();
    void OnChar();

	void SetSurface(vtkPolyData* surface);
	void SetCenterline(vtkPolyData* centerline);
	void SetAppendEndPoints(bool appendFlag);
    void castCamera(Eigen::Vector3d position,Eigen::Vector3d focalPoint,Eigen::Vector3d up);

    bool IsReady();
    bool IsRenderring();
    bool IsFinished();
    void RequireFinish();
    bool RequireReset();
    void SetResetStatus(bool status);
    vtkPolyData* GetSurface();
    Sophus::SE3 GetInitPosition();

    vtkSmartPointer<vtkSphereSource> cameraSensor;
    vtkSmartPointer<vtkSphereSource> cameraSensorBefore;
    int fileName = 0;
    int saveCmd = 0;

protected:
    void SetThePoints();
    void SetTheInitPostion(double x,double y,double z);
    void Roaming();
    void GetTheGuidePoints(vtkSmartPointer<vtkParametricSpline> spline, unsigned int numPoints);

private:
    vtkPolyData* m_surface = nullptr;
    vtkSphereSource* m_sphere = nullptr;
    vtkPolyData* m_centerline = nullptr;
	bool m_currentSeedType = 0; //0 and 1 are source and target seed type respectively
	bool m_appendFlag = 0;
	unsigned int m_numOfSourceSeed = 0;
	unsigned int m_numOfTargetSeed = 0;
	unsigned int m_numOfSeeds = 0;
    map<unsigned int, vtkSphereSource*> seedList;

    map<unsigned int, vtkActor*> seedActorList;
    map<unsigned int, bool> seedTypeList;
	double m_currentSeedPosition[3] = { 0, 0, 0 };
    std::mutex mMutex;
    std::mutex mMutexRender;

    vector<vtkLODActor*> pointActors;
    vector<vtkPoints*> mPoints;
    int pathSelect = 0;

    Eigen::Vector3d mposition;
    Eigen::Vector3d mfocalPoint;
    Eigen::Vector3d mup;

    Eigen::Vector3d mInitPostion;
    Eigen::Matrix3d mInitPosture;
    Sophus::SE3 mTwc;

    vtkSmartPointer<vtkActor> cameraSensorActor;
    vtkSmartPointer<vtkActor> cameraSensorActorBefore;

    bool rendering = true;
    bool ready = false;
    bool reset = true;

    static bool finished;
};


#endif

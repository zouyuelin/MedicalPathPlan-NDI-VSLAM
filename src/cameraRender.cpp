#include "cameraRender.h"

vtkMyCommand::vtkMyCommand()
{
    selectEnclosedPoints = vtkSmartPointer<vtkSelectEnclosedPoints>::New();
    kDTreePoints = vtkKdTree::New();
}

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
            CenterlinePathSet->cameraSensorBefore->SetCenter(m_lastPosition[0],m_lastPosition[1],m_lastPosition[2]);
            CenterlinePathSet->GetInteractor()->Render();
        }
    }
}

void vtkMyCommand::setTheStyle(MouseInteractorStyleCenterline *CenterlinePathSet_)
{
    CenterlinePathSet = CenterlinePathSet_;
}

void vtkMyCommand::setThesurfaceForCollDetection(vtkPolyData *surface_)
{
    surface = surface_;
    selectEnclosedPoints->SetTolerance(0.00001);
    selectEnclosedPoints->Initialize(surface);
    selectEnclosedPoints->Update();
}

void vtkMyCommand::setThenormals(vtkPolyDataNormals* normals_)
{
    normals = normals_;
    kDTreePoints->BuildLocatorFromPoints(normals->GetOutput()->GetPoints());
}

Eigen::Vector3d vtkMyCommand::getTheClosetPointNorm(double x,double y,double z)
{
    double closetPointDist;
    vtkIdType id = kDTreePoints->FindClosestPoint(x,y,z,closetPointDist);

    double normValue[3];
    normals->GetOutput()->GetPointData()->GetNormals()->GetTuple(id,normValue);
    return Eigen::Vector3d(normValue[0],normValue[1],normValue[2]);
}

Eigen::Vector3d vtkMyCommand::getTheClosetPoint(double x,double y,double z)
{
    double closetPointDist;
    vtkIdType id = kDTreePoints->FindClosestPoint(x,y,z,closetPointDist);
    double temp[3];
    normals->GetOutput()->GetPoints()->GetPoint(id,temp);

    return Eigen::Vector3d(temp[0],temp[1],temp[2]);
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
        //碰撞测试
        collisionDetection(position,focalpoint,up);

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

void vtkMyCommand::collisionDetection(Eigen::Vector3d &position,Eigen::Vector3d &focalPoint,Eigen::Vector3d &up)
{
    static bool initTheLastPoint = false;
    if(!initTheLastPoint)
    {
        m_lastPosition = position;
        initTheLastPoint = true;
        return;
    }

    Eigen::Vector3d sourceP = position;

    if(selectEnclosedPoints->IsInsideSurface(position[0],position[1],position[2])) //position[0],position[1],position[2]
    {
//        cout<<"Fucking inside ! "<<++i << endl;
        lastPositionInside = true;
    }
    else
    {
        Eigen::Vector3d positionDirection = position - m_lastPosition;
        Eigen::Vector3d cameraDirection = focalPoint - position;

        if(m_lastPosition_change.norm() == 0 || lastPositionInside == true)
        {
            Eigen::Vector3d norm = getTheClosetPointNorm(m_lastPosition[0],m_lastPosition[1],m_lastPosition[2]); //position[0],position[1],position[2]
            Eigen::Vector3d a_ = position - m_lastPosition;
            Eigen::Vector3d b_ ;
            b_ = a_ - a_.transpose() * norm * norm / 1.0;
            m_lastPosition_change = m_lastPosition + b_;
        }
        else if(lastPositionInside == false)
        {
            Eigen::Vector3d newDirection = m_lastPosition_change + positionDirection;
            if (!selectEnclosedPoints->IsInsideSurface(newDirection[0],newDirection[1],newDirection[2]))
            {
                Eigen::Vector3d theBorderPt = getTheClosetPoint(newDirection[0],newDirection[1],newDirection[2]);
                m_lastPosition_change = theBorderPt;
            }
            else{
                m_lastPosition_change = newDirection;
            }

        }

        position = m_lastPosition_change;
        focalPoint = position + cameraDirection;

        collision = true;
        lastPositionInside = false;
    }

    recordThePath(sourceP,position);
    //磁导航上一位置
    m_lastPosition = sourceP;//上个点

    collision = false;
}

void vtkMyCommand::recordThePath(Eigen::Vector3d position, Eigen::Vector3d lastpoint)
{
    static long int id = 0;

    if(!trackNDI.is_open() && CenterlinePathSet->saveCmd == 1)
    {
        trackNDI.open("data/" + to_string(CenterlinePathSet->fileName) + "_NDI.txt");
        collD.open("data/" + to_string(CenterlinePathSet->fileName) + "_colld.txt");
        collDAndNDI.open("data/" + to_string(CenterlinePathSet->fileName) + "_colldAndNDI.txt");
        std::cout<<"open the file"<<std::endl;
    }

    if(CenterlinePathSet->saveCmd == 1 && trackNDI.is_open())
    {
        Eigen::Vector3d temp =n_lastPosition_colld - lastpoint;
        if(temp.norm() > 0.5)
        {
            trackNDI<<id<<" "<< position[0] <<" "<<position[1]<<" "<<position[2]<<std::endl;
            collD<<id<<" "<< lastpoint[0] <<" "<<lastpoint[1]<<" "<<lastpoint[2]<<std::endl;

            if(collision == true)
            {
                collDAndNDI<<id<<" "<< position[0] <<" "<<position[1]<<" "<<position[2]<<" "<< lastpoint[0] <<" "<<lastpoint[1]<<" "<<lastpoint[2]<<std::endl;
            }

            id++;
        }

        n_lastPosition_pre = position;
        n_lastPosition_colld = lastpoint;
    }

    if(CenterlinePathSet->saveCmd == 2)
    {
        trackNDI.close();
        collD.close();
        collDAndNDI.close();
        CenterlinePathSet->saveCmd = 0;
        id = 0;
        std::cout<<"stop save path data"<<std::endl;
    }
}

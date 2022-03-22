#include "visualmap.h"

visualMap::visualMap()
{
    mTwc = Eigen::Isometry3d::Identity();
}

void visualMap::run()
{
    //---------------------------------------------------------------map init----------------------------------------------------//
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST | GL_COLOR_BUFFER_BIT);//深度测试
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    pangolin::OpenGlRenderState s_cam = pangolin::OpenGlRenderState(                                                  //摆放一个相机
            pangolin::ProjectionMatrix(1024, 768, 1000, 1000, 512, 384, 0.1, 500),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, pangolin::AxisZ)
    );
    pangolin::View &d_cam = pangolin::CreateDisplay()//创建一个窗口
            .SetBounds(0.0, 1.0, 0, 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while(true)
    {
//        unique_lock<mutex> lock(this->mMutex);
        //消除颜色缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        s_cam.Follow(mTwc.matrix());

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        d_cam.Activate(s_cam);
        view();

        pangolin::FinishFrame();
    }
}

void visualMap::clearPoses()
{
    std::unique_lock<std::mutex> lock(this->mMutex);
    keyposes.clear();
}

void visualMap::GetPoses(Eigen::Isometry3d Twc)
{
    std::unique_lock<std::mutex> lock(this->mMutex);
    this->keyposes.push_back(Twc);
}

void visualMap::GetCurrentTwc(Eigen::Isometry3d CurrentTwc)
{
    std::unique_lock<std::mutex> lock(this->mMutex);
    this->mTwc = CurrentTwc;
}

void visualMap::GetCurrentPointsInCamera(std::vector<Eigen::Vector3d> mpts)
{
    std::unique_lock<std::mutex> lock(this->mMutex);
    this->currentPointsInCamera = mpts;
}

void visualMap::view()
{
    this->drawing();
    if(!currentPointsInCamera.empty())
    {
        this->drawPoints();
    }
}

void visualMap::drawAxis(Eigen::Isometry3d poses)
{
    //画出坐标轴
    Eigen::Vector3d Ow = poses.translation();
    Eigen::Vector3d Xw = poses * (0.1 * Eigen::Vector3d(1, 0, 0));
    Eigen::Vector3d Yw = poses * (0.1 * Eigen::Vector3d(0, 1, 0));
    Eigen::Vector3d Zw = poses * (0.1 * Eigen::Vector3d(0, 0, 1));
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Xw[0], Xw[1], Xw[2]);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Yw[0], Yw[1], Yw[2]);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3d(Ow[0], Ow[1], Ow[2]);
    glVertex3d(Zw[0], Zw[1], Zw[2]);
    glEnd();
}

void visualMap::getVelocity(Eigen::Isometry3d &pose_last,Eigen::Isometry3d &pose_next,double &time_used,Eigen::Vector4d &trans_velocity,Eigen::Vector3d &angluar_velocity)
{
    //平移速度 x y z v_r(合速度)
    double dx = pose_next.translation()[0]-pose_last.translation()[0];
    double dy = pose_next.translation()[1]-pose_last.translation()[1];
    double dz = pose_next.translation()[2]-pose_last.translation()[2];
    double distance_ = sqrt(dx*dx+dy*dy+dz*dz);
    trans_velocity <<dx/time_used,dy/time_used,dz/time_used,distance_/time_used;

    //角速度：绕 z y x--->x y z
    Eigen::AngleAxisd rotation_vector_last(pose_last.rotation());
    Eigen::AngleAxisd rotation_vector_next(pose_next.rotation());
    Eigen::Vector3d dtheta_zyx = camera::ToEulerAngles(Eigen::Quaterniond(rotation_vector_next.matrix())) - camera::ToEulerAngles(Eigen::Quaterniond(rotation_vector_last.matrix()));
    Eigen::Vector3d angluar_zyx = dtheta_zyx/time_used;
    angluar_velocity <<angluar_zyx[2],angluar_zyx[1],angluar_zyx[0];
}

void visualMap::drawCurrentFrame(Eigen::Isometry3d poses)
{
    //变换位姿
    glPushMatrix();
    pangolin::OpenGlMatrix Twc_(poses.matrix());
    glMultMatrixd(Twc_.m);

    glColor3f(229/255.f, 113/255.f, 8/255.f);
    glLineWidth(2);
    glBegin(GL_LINES);
    //画相机模型
    float scale_=1.5;
    glVertex3f(0, 0, 0);
    glVertex3f(w*scale_,h*scale_,z*scale_);
    glVertex3f(0, 0, 0);
    glVertex3f(w*scale_,-h*scale_,z*scale_);
    glVertex3f(0, 0, 0);
    glVertex3f(-w*scale_,-h*scale_,z*scale_);
    glVertex3f(0, 0, 0);
    glVertex3f(-w*scale_,h*scale_,z*scale_);
    glVertex3f(w*scale_,h*scale_,z*scale_);
    glVertex3f(w*scale_,-h*scale_,z*scale_);
    glVertex3f(-w*scale_,h*scale_,z*scale_);
    glVertex3f(-w*scale_,-h*scale_,z*scale_);
    glVertex3f(-w*scale_,h*scale_,z*scale_);
    glVertex3f(w*scale_,h*scale_,z*scale_);
    glVertex3f(-w*scale_,-h*scale_,z*scale_);
    glVertex3f(w*scale_,-h*scale_,z*scale_);

    glEnd();
    glPopMatrix();
}

void visualMap::drawLine(size_t i,std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses,bool drawLine)
{
    glLineWidth(2);
    if(drawLine)
    {
      for (size_t j = 1; j < i; j++) {
          glColor3f(1.0, 0.0, 0.0);
          glBegin(GL_LINES);
          Eigen::Isometry3d p1 = poses[j-1], p2 = poses[j];
          glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
          glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
          glEnd();
          }
    }
}

void visualMap::drawKeyFrame(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> keyframs)
{
    for(auto Twc:keyframs)
    {
        glPushMatrix();
        pangolin::OpenGlMatrix Twc_(Twc.matrix());
        glMultMatrixd(Twc_.m);

        glLineWidth(2);
        glColor3f(20/255.f, 68/255.f, 106/255.f);
        glBegin(GL_LINES);
        //画相机模型
        glVertex3f(0, 0, 0);
        glVertex3f(w,h,z);
        glVertex3f(0, 0, 0);
        glVertex3f(w,-h,z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w,-h,z);
        glVertex3f(0, 0, 0);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);
        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);
        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);
        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);

        glEnd();
        glPopMatrix();
    }
}

void visualMap::drawCoordinate(float scale)
{
    glLineWidth(1.2);
    glBegin(GL_LINES);
    //x
    glColor3f(1.0, 0.0, 0.0);
    glVertex3d(0,0,0);
    glVertex3d(scale,0,0);
    //y
    glColor3f(0.0, 1.0, 0.0);
    glVertex3d(0,0,0);
    glVertex3d(0,scale,0);
    //z
    glColor3f(0.0, 0.0, 1.0);
    glVertex3d(0,0,0);
    glVertex3d(0,0,scale);
    glEnd();
}


void visualMap::drawing()
{

      if(pangolin::ShouldQuit()==false)
      {
          //画相机模型
          drawCurrentFrame(mTwc);
          //画出动态坐标轴
          drawAxis(mTwc);
          //画坐标系
          drawCoordinate(0.5);
          //绘制关键帧
          {
              std::unique_lock<std::mutex> lc(mMutex);
              drawKeyFrame(keyposes);
          }
      }
}

void visualMap::drawPoints()
{
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_POINTS);
//    #pragma omp parallel for
    for(int i=0;i<currentPointsInCamera.size();i++ )
    {
        glVertex3d(currentPointsInCamera[i][0],currentPointsInCamera[i][1],currentPointsInCamera[i][2]);
    }
    glEnd(); //加上glEnd()可以加快绘点的速度
}
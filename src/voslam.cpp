#include "voslam.h"

//g2o
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>

#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>

#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include <g2o/types/sim3/sim3.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/se3quat.h>

#include <opencv2/core/eigen.hpp>


VO_slam::VO_slam(const std::string vocabPath):state_(VOstate::INITIALIZING),
    pose_max_trans(0.5),pose_max_angle(0.5),
    keyFrame_min_trans(0.02),KeyFrame_max_trans(0.2),
    num_trackingTry(0),updatePoseMethod(OPTIMIZERTWC) //UPDATATCW OPTIMIZERTWC
{
    //加载词典
    std::cout << std::endl << "Loading ORB Vocabulary. This could take a while..." << std::endl;
    vocab.load(vocabPath);
    //保存词典
//    vocab.save("Vocabulary/ORBvoc.bin",true);
    map_ = std::make_shared<Map>();
    visual_ = std::make_shared<visualMap>();
    loopclose_ = std::make_shared<LoopClosing>(vocab);
    PointCloudMapping_ = std::make_shared<PointCloudMapping>( 0.02 );

//    omp_set_num_threads(15);
    viewerThread = std::make_shared<std::thread>(bind(&visualMap::run,visual_));
    loopThread = std::make_shared<std::thread>(bind(&LoopClosing::run,loopclose_));
}

Sophus::SE3 VO_slam::tracking(Mat img, Mat depth)
{
    depth.convertTo(depth,CV_32F,1.0/camera::depthScale);
    Frame frame_cur(img,depth);
    frame_cur.detectKeyPoints();
    frame_cur.computeDescriptors();
        vocab.transform(frame_cur.descriptors,frame_cur.BowVec);
    if(state_ != VOstate::OK)
    {
        //特征点超过50才能完成初始化
        if(frame_cur.keypoints.size()>50)
        {
            //clear the key pose and frames
            if(!map_->keyFrames.empty())
            {
                map_->keyFrames.clear();
                {
//                    unique_lock<mutex> lock(this->mMutex);
                    visual_->clearPoses();
                }
            }
            frame_cur.setPose(Sophus::SE3());
            frame_cur.ComputekeyPtsPw();

            map_->pose_cur_Twc = frame_cur.getPose(); //Twc1
            map_->addKeyFrames(frame_cur);
            map_->add3Dpts_world(frame_cur.pts_3d);
            //ref frame
            map_->frame_ref = frame_cur;

            PointCloudMapping_->insertKeyFrame(frame_cur);

            state_ = VOstate::OK;

            std::cout<<"Initlization *************\n";
        }

        return map_->pose_cur_Twc;
    }

    if(frame_cur.keypoints.size()<50)
        return map_->pose_cur_Twc;

    featureMatch(map_->frame_ref.descriptors,frame_cur.descriptors);

    if(featureMatches.size()<=20 && num_trackingTry<3)
    {
        num_trackingTry++;
        std::cout<<"trying tracking again..."<<num_trackingTry<<std::endl;
        return map_->pose_cur_Twc;
    }
    else if(num_trackingTry>=3)
    {
        num_trackingTry = 0;
        state_ = VOstate::LOST;
        std::cout<<"Tracking lost *************\n";
        return map_->pose_cur_Twc;
    }
    else
    {
        num_trackingTry = 0;
    }

        //pose estimate
        bool matchCondition;

        //init the pose
        // 最小化重投影误差
        // e = (u,v) - project(Tcw*Pw)
        Sophus::SE3 Tcw = map_->frame_ref.mTwc.inverse();
        matchCondition = pnpSolver(map_->frame_ref,frame_cur,Tcw); //Twc
        frame_cur.setPose(Tcw.inverse());

        if(matchCondition==false)
            return map_->pose_cur_Twc;

        if(checkEstimatedPose(map_->frame_ref.mTwc,frame_cur.getPose())== true)
        {
            //updata current pose
            frame_cur.ComputekeyPtsPw();
            map_->pose_cur_Twc = frame_cur.getPose(); //Twc1

            map_->frame_ref = frame_cur;//map_->keyFrames.back()

            if(checkKeyFrame() == true)
            {
                //ref frame
                PointCloudMapping_->insertKeyFrame(frame_cur);

                map_->addKeyFrames(frame_cur);
                map_->add3Dpts_world(frame_cur.pts_3d);
                //insert the keyFrames
                loopclose_->getKeyFrames(map_->keyFrames);

                visual_->GetPoses(Eigen::Isometry3d(frame_cur.getPose().matrix()));
            }
            //    map_->map_points.find((unsigned long)(map_->KeyPoses.size()-1));
            visual_->GetCurrentTwc(Eigen::Isometry3d(frame_cur.getPose().matrix()));
            visual_->GetCurrentPointsInCamera(frame_cur.pts_3d);
        }

        return map_->pose_cur_Twc;
}

void VO_slam::SiftMapPoints(Frame frame_ref,Frame frame_cur,
                       std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &pts_3d_eigen,
                       std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &pts_2d_eigen)
{
    for(auto m:featureMatches)
    {
        float d = frame_ref.pts_3d_cam[m.queryIdx][2];
        if(d>0 && d<10)
        {
            Eigen::Vector3d ref_pts3d;

            if(updatePoseMethod==UPDATATCW)
                ref_pts3d = frame_ref.pts_3d_cam[m.queryIdx];
            else
                ref_pts3d = frame_ref.pts_3d[m.queryIdx];

            cv::Point2f cur_pts2d = frame_cur.keypoints[m.trainIdx].pt;

            pts_3d_eigen.push_back(ref_pts3d);
            pts_2d_eigen.push_back(Eigen::Vector2d(cur_pts2d.x,cur_pts2d.y));
        }

    }
}

void VO_slam::PoseOptimizer(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &pts_3d_eigen,
                                     std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &pts_2d_eigen,
              Mat &K,Sophus::SE3 &pose)
{
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    //构造求解器
    g2o::SparseOptimizer optimizer;
    //线性方程求解器
    //g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
    //矩阵块求解器
    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3(linearSolver);
    //L-M优化算法
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
    //
    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(true);

    //顶点
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(g2o::SE3Quat(pose.rotation_matrix(),pose.translation()));//
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    //边
    for(size_t i = 0;i<pts_2d_eigen.size();i++)
    {
//        int index = inliers.at<int> ( i,0 );
        g2o::EdgeSE3ProjectXYZOnlyPose* edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setMeasurement(pts_2d_eigen[i]);
        edge->fx = fx;
        edge->fy = fy;
        edge->cx = cx;
        edge->cy = cy;
        edge->Xw = pts_3d_eigen[i];
        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        edge->setRobustKernel(rk);
        optimizer.addEdge(edge);
    }

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    pose = Sophus::SE3(vSE3->estimate().rotation(),vSE3->estimate().translation());
}

void VO_slam::BAOptimizer(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &pts_3d_eigen,
                          std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &pts_2d_eigen,
                          Mat &K, Sophus::SE3 &pose)
{
    //构造求解器
    g2o::SparseOptimizer optimizer;
    //线性方程求解器
    //g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
    //矩阵块求解器
    g2o::BlockSolver_6_3* block_solver = new g2o::BlockSolver_6_3(linearSolver);
    //L-M优化算法
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);
    //
    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(true);

    //添加位姿顶点
    g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap;
    v->setId(0);
    v->setFixed(false);
    v->setEstimate(g2o::SE3Quat(pose.rotation_matrix(),pose.translation()));
    optimizer.addVertex(v);

    //添加特征点顶点
    for(int i=1;i<pts_3d_eigen.size();i++)
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId(i); //已经添加过两个位姿的顶点了
        v->setEstimate(pts_3d_eigen[i]);
        v->setFixed(false); //优化
        v->setMarginalized(true);//把矩阵块分成两个部分，分别求解微量
        optimizer.addVertex(v);
    }

    //添加相机参数
    g2o::CameraParameters* camera = new g2o::CameraParameters(K.at<double>(0, 0),Eigen::Vector2d(K.at<double>(0, 2),K.at<double>(1, 2)),0);
    camera->setId(0);
    optimizer.addParameter(camera);

    //添加边,第一帧和第二帧
    for(size_t i = 1;i<pts_3d_eigen.size();i++)
    {

        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0,dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i)));
        edge->setVertex(1,dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(0)));
        edge->setMeasurement(pts_2d_eigen[i]);
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0,0);//这句必要
        optimizer.addEdge(edge);

    }

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(20);
    pose = Sophus::SE3(v->estimate().rotation(),v->estimate().translation());

    //****************************BA优化过程*********************
}

bool VO_slam::solvePnPOpencv(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &pts_3d_eigen,
                              std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &pts_2d_eigen,
                              Mat &K, Sophus::SE3 &pose)
{
    Mat r, t, R;
    std::vector<Point2f> pts_2d_cur;
    std::vector<Point3f> pts_3d_ref;

    Mat inliers;
//    #pragma omp parallel for
    for (size_t i = 0; i < pts_3d_eigen.size(); ++i) {
        pts_3d_ref.push_back(cv::Point3f(pts_3d_eigen[i][0], pts_3d_eigen[i][1], pts_3d_eigen[i][2]));
        pts_2d_cur.push_back(cv::Point2f(pts_2d_eigen[i][0], pts_2d_eigen[i][1]));
      }

    try
    {
        cv::solvePnPRansac(pts_3d_ref, pts_2d_cur, K, Mat(), r, t, false, 100, 8.0, 0.99,inliers);     // 调用OpenCV 的 PnPRansac 求解，可选择EPNP，DLS等方法
    }
    catch(cv::Exception e)
    {
        std::cout<<"throwing a error :"<<e.what()<<std::endl;
        return false;
    }
    catch(...)
    {
        std::cout<<"throwing a unknown error"<<std::endl;
        return false;
    }
    cv::Rodrigues(r, R);                                            // r为旋转向量形式，用Rodrigues公式转换为矩阵
    Eigen::Matrix3d R_eigen;
    Eigen::Vector3d t_eigen;
    cv::cv2eigen(R,R_eigen);
    cv::cv2eigen(t,t_eigen);
    pose = Sophus::SE3(R_eigen,t_eigen);
    return true;
}

bool VO_slam::checkEstimatedPose(Sophus::SE3 Twc_ref,Sophus::SE3 Twc_cur)
{
    Sophus::SE3 pose = Twc_cur.inverse() * Twc_ref; //Tc2c1
    if(pose.translation().norm()>pose_max_trans)
    {
        return false;
    }
    Eigen::Vector3d euler = camera::ToEulerAngles(Eigen::Quaterniond(pose.rotation_matrix()));
    double deltaFai = euler.norm();
    if(deltaFai > pose_max_angle)
    {
        return false;
    }
    return true;
}

bool VO_slam::checkKeyFrame()
{
    Sophus::SE3 lastKeyFramePosTwc1 = map_->keyFrames.back().mTwc;
    Sophus::SE3 currentFramePosTwc2 = map_->pose_cur_Twc;
    Eigen::Vector3d TransE = (lastKeyFramePosTwc1.translation()-currentFramePosTwc2.translation());
    Eigen::Matrix3d eurE_ = currentFramePosTwc2.rotation_matrix().transpose() * lastKeyFramePosTwc1.rotation_matrix();
    Eigen::Vector3d Eur = camera::ToEulerAngles(Eigen::Quaterniond(eurE_));
    if(Eur.norm()>M_PI/12 || TransE.norm()>keyFrame_min_trans)
    {
        return true;
    }
    if(TransE.norm()>KeyFrame_max_trans)
    {
        return false;
    }

    return false;
}

void VO_slam::featureMatch(Mat descriptors_ref,Mat descriptors_cur)
{
    std::vector<DMatch> match;
    featureMatches.clear();
    // BFMatcher matcher ( NORM_HAMMING );
    Frame::matcher.match(descriptors_ref, descriptors_cur, match);

    double min_dist = 10000, max_dist = 0;

    for (int i = 0; i < match.size(); i++)
    {
      double dist = match[i].distance;
      if (dist < min_dist) min_dist = dist;
      if (dist > max_dist) max_dist = dist;
    }

  for (int i = 0; i < match.size(); i++)
  {
    if (match[i].distance <=  max<float> ( min_dist*2, 30.0 ))
    {
      featureMatches.push_back(match[i]);
    }
  }
}

bool VO_slam::pnpSolver(Frame frame_ref, Frame frame_cur,Sophus::SE3 &Curpose)
{
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> pts_3d;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> pts_2d;
        SiftMapPoints(frame_ref,frame_cur,pts_3d,pts_2d);
    if(pts_2d.size()<10)
        return false;

//    if(!solvePnPOpencv(pts_3d,pts_2d,camera::K,Curpose))
//        return false;

    PoseOptimizer(pts_3d,pts_2d,camera::K,Curpose);

    return true;
}

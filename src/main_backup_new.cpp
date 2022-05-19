﻿#include "vtkpose.h"
#include "nditrack.h"

//opencv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "onnxrun.h"

#include <boost/timer.hpp>
#include <iostream>


using namespace std;

int main(int argc, char**argv)
{

    if(argc<2)
    {
        cout<<"The usage is: NDI-VSLAM STL_MODEL_PATH DL{NDI}\n";
        return -1;
    }

    string RunType = argv[2];

    cout<<"---------------starting------------------------\n";

    cv::Mat cameraMatrix(cv::Mat::zeros(3,3,CV_64F));
    cv::Mat distCoeffs(cv::Mat::zeros(5,1,CV_64F));
    cameraMatrix.at<double>(0,0) = 399.23;
    cameraMatrix.at<double>(1,1) = 396.99;
    cameraMatrix.at<double>(0,2) = 301.175;
    cameraMatrix.at<double>(1,2) = 306.12;
    cameraMatrix.at<double>(2,2) = 1.00;

    distCoeffs.at<double>(0,0) = -0.0876;
    distCoeffs.at<double>(0,1) = -0.1972;
    distCoeffs.at<double>(0,4) = 0.1358;

    //vtk 显示交互程序
    vtkPose::Ptr vtkRender = std::make_shared<vtkPose>(string(argv[1]));
    //电磁导航开启
    NDItracking::Ptr nditrack = std::make_shared<NDItracking>(false);
    //深度学习
    DLrun *dlonnx = new DLrun("model_ssd_resnet101.onnx");
    //model_faster_rcnn_resnet101.onnx   model_ssd_resnet101.onnx  model_ssd_resnet50.onnx model_faster_rcnn_inceptionv2.onnx

    //初始化参数
    cv::VideoCapture cap;
        cap.open(0);
        cap.set(cv::CAP_PROP_FRAME_WIDTH,1920);//1920*1080
        cap.set(cv::CAP_PROP_FRAME_HEIGHT,1080);
    ofstream txt;
        txt.open("images/pose.txt");

    Sophus::SE3 LastTwc,TwcI,currentTcw;
    cv::Mat lastFrame;
    //距离补偿
    double compensate = 1.0;

    //等待1秒
    sleep(1);// wait a moment
    while(nditrack->IsRunning())
    {
    //---------------------------------------------------------------tracing------------------------------------------------------//
        while(!vtkRender->IsFinished())
        {
            //calculate the time comsuption
//            boost::timer timer;
            Sophus::SE3 Twc = nditrack->getPoseTwc();
            if(vtkRender->RequireReset())
            {
                TwcI = Twc;
                currentTcw = Sophus::SE3();
                vtkRender->SetResetStatus(false);
                std::cout<<"Has reset the posture !"<<std::endl;
                continue;
            }
               //利用NDI
            if(RunType == "NDI")
            {
                /*
                 * Twrce 起始状态(E twrci)  Twrcr 真实状态
                 * Twrce 与wr坐标系仅差平移
                 * Tcecr = Twrce.inverse() * Twrcr ([E -twrci] * [Rwrcr twrcr] = [Rwrcr twrcr-twrci] )
                 * Tcvcr = [Rwrcr twrcr-twrci]
                */
                Sophus::SE3 Tcvcr( Twc.rotation_matrix(),(Twc.translation()-TwcI.translation()) * compensate );  //这里乘以compensate是因为重建的误差导致的，相当于一个补偿
                //Sophus::SE3 Tcicr = TwcI.inverse()*Twc;  //Tc2c1 绝对增量，但位置不容易对准
                vtkRender->getPoseTcw(Tcvcr.inverse());

                cv::Mat frame,distortImage;
                if(cap.isOpened())
                {
                    cap>>frame;
                    cv::Rect rect(frame.cols/2-355,frame.rows/2-355,750,750);
                    frame=frame(rect);
                    cv::resize(frame,frame,cv::Size(600,600));
                    cv::undistort(frame,distortImage,cameraMatrix,distCoeffs);
                }
                dlonnx->objectPositionONNX(distortImage);

                cv::imshow("currentframe",distortImage);
                cv::waitKey(1);
            }
              //利用Deeplearning
            else if(RunType == "DL")
            {
                //内窥镜数据采集
                //----------------------------初始化 \begin-------------------
                cv::Mat frame,distortImage;
                if(cap.isOpened())
                {
                    cap>>frame;
                    cv::Rect rect(frame.cols/2-355,frame.rows/2-355,750,750);
                    frame=frame(rect);
                    cv::resize(frame,frame,cv::Size(600,600));
                    cv::undistort(frame,distortImage,cameraMatrix,distCoeffs);
                }

                static long int id = 0;
                if(id == 0)
                {
                    lastFrame = frame;
                    id++;
                    continue;
                }
                //----------------------------初始化 \end-------------------

                Sophus::SE3 Tc21 = dlonnx->computePoseDNN(lastFrame,frame);

                currentTcw = Tc21 * currentTcw;

                //get the pose
                vtkRender->getPoseTcw(currentTcw);

                lastFrame = frame;//updata the last frame
                cv::imshow("currentframe",distortImage);

                if((char)cv::waitKey(10)=='l')
                {
                    Sophus::SE3 Tc2c1;
                    Tc2c1 = Twc.inverse() * LastTwc;

                    cv::imwrite("images/"+std::to_string(id)+".jpg",frame);
                    LastTwc = Twc;

                    if(id != 1)
                    {
                    Eigen::Vector3d eular = DLrun::ToEulerAngles(Tc2c1.unit_quaternion());
                    //x y z roll-rx pitch-ry yaw-rz
                    txt<<id <<" "<<"images/"+std::to_string(id-1)+".jpg"<<" "<<"images/"+std::to_string(id)+".jpg"<<" "<<Tc2c1.translation()[0]
                             <<" "<<Tc2c1.translation()[1]<<" "<<Tc2c1.translation()[2]<<" "<<eular[0]<<" "<<eular[1]<<" "<<eular[2]<<" "<< std::endl;

                    Sophus::SE3 Tc1c2 = Tc2c1.inverse();
                    Eigen::Vector3d eular_c1c2 = DLrun::ToEulerAngles(Tc1c2.unit_quaternion());
                    //x y z roll-rx pitch-ry yaw-rz
                    txt<<id <<" "<<"images/"+std::to_string(id)+".jpg"<<" "<<"images/"+std::to_string(id-1)+".jpg"<<" "<<Tc1c2.translation()[0]
                             <<" "<<Tc1c2.translation()[1]<<" "<<Tc1c2.translation()[2]<<" "<<eular_c1c2[0]<<" "<<eular_c1c2[1]<<" "<<eular_c1c2[2]<<" "<< std::endl;
                    }
                    id++;
                }
            }
            //std::cout<<timer.elapsed()<<" s"<<std::endl;
        }
        txt.close();

        //finished the ndi
        nditrack->FinishTracking();
        cv::destroyAllWindows();
    }

    std::cout<<"The main thread is done !"<<std::endl;

    return 0;
}

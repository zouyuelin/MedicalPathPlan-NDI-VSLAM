#include "voslam.h"
#include "vtkpose.h"
#include <rs.hpp>
#include "nditrack.h"

////opencv
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/calib3d.hpp>
//#include <opencv2/features2d/features2d.hpp>

#include <boost/timer.hpp>
#include <iostream>


using namespace std;

int main(int argc, char**argv)
{

    if(argc<5)
    {
        cout<<"The usage is: NDI-VSLAM config/intel_RGBD.yaml Vocabulary/ORBvoc.bin <NDI or VSLAM> STL_MODEL_PATH\n";
        return -1;
    }

    //是否使用NDI导航
    string SLAMMethod = argv[3];
    bool useNDITracking;
    if(SLAMMethod == "NDI")
        useNDITracking = true;
    else if(SLAMMethod == "VSLAM")
        useNDITracking = false;
    else
    {
        cout<<"please input NDI or VSLAM"<<endl;
        return -1;
    }

    const std::string PortName = "/dev/ttyUSB0";

    cout<<"---------------starting------------------------\n";

    vtkPose::Ptr vtkRender = std::make_shared<vtkPose>(string(argv[4]));


    if(useNDITracking)
    {
        NDItracking::Ptr nditrack = std::make_shared<NDItracking>(PortName);
        //等待NDI响应完成
        sleep(10);
        bool hasInit = false;
        Sophus::SE3 TcwI;
        while(!vtkRender->IsFinished())
        {
        //---------------------------------------------------------------tracing------------------------------------------------------//
            boost::timer timer;
            Sophus::SE3 Tcw = nditrack->getPoseTcw();
            if(!hasInit)
            {
                TcwI = Tcw;
                hasInit = true;
                continue;
            }

            Sophus::SE3 TcwP = Tcw * TcwI.inverse();  //Tc2c1

//            cout<<TcwP.matrix()<<endl;

            vtkRender->getPose(Tcw);
            cv::waitKey(10);
        }

        if(nditrack->FinishTrac())
        {
            std::cout<<"The main thread is done !"<<std::endl;
        }
    }

    else
    {
        rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

        rs2::config cfg;
        ///设置从设备管道获取的深度图和彩色图的配置对象
        ///配置彩色图像流：分辨率640*480，图像格式：BGR， 帧率：30帧/秒
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        ///配置深度图像流：分辨率640*480，图像格式：Z16， 帧率：30帧/秒
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

        ///生成Realsense管道，用来封装实际的相机设备
        rs2::pipeline pipe;
        ///根据给定的配置启动相机管道
        pipe.start(cfg);

        rs2::frameset data;

        camera::loadYaml(argv[1]);
        const std::string vocabPath = std::string(argv[2]);

        VO_slam::Ptr slam = std::make_shared<VO_slam>(vocabPath);

        while(true)
        {
        //---------------------------------------------------------------tracing------------------------------------------------------//
            boost::timer timer;
            ///等待一帧数据，默认等待5s
            data = pipe.wait_for_frames();

            rs2::frame depth  = data.get_depth_frame(); ///获取深度图像数据
            rs2::frame color = data.get_color_frame();  ///获取彩色图像数据
            if (!color || !depth) break;

            cv::Mat image(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depthmat(cv::Size(640, 480), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);


            Sophus::SE3 Twc = slam->tracking(image,depthmat);

            vtkRender->getPose(Twc.inverse());
            //cout<<"VO cost time is:"<<timer.elapsed()<<endl;
            cv::imshow("frame",image);
            cv::waitKey(10);

         }
    }

    return 0;
}

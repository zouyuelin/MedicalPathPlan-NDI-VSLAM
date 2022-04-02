#include "vtkpose.h"
#include "nditrack.h"

////opencv
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

    if(argc<3)
    {
        cout<<"The usage is: NDI-VSLAM STL_MODEL_PATH Textpicture\n";
        return -1;
    }

    const std::string PortName = "/dev/ttyUSB0";

    cout<<"---------------starting------------------------\n";

    vtkPose::Ptr vtkRender = std::make_shared<vtkPose>(string(argv[1]),string(argv[2]));
    NDItracking::Ptr nditrack = std::make_shared<NDItracking>(PortName);

    Sophus::SE3 TwcI;
    //距离补偿
    double compensate = 1.0;
    //等待3秒
    sleep(3);// wait a moment

    while(nditrack->IsRunning())
    {
    //---------------------------------------------------------------tracing------------------------------------------------------//
        while(!vtkRender->IsFinished())
        {
            boost::timer timer;
            Sophus::SE3 Twc = nditrack->getPoseTwc();
            if(vtkRender->RequireReset())
            {
                TwcI = Twc;
                vtkRender->SetResetStatus(false);
                std::cout<<"Has reset the posture !"<<std::endl;
                continue;
            }

            /*
             * Twrce 起始状态(E twrci)  Twrcr 真实状态
             * Twrce 与wr坐标系仅差平移
             * Tcecr = Twrce.inverse() * Twrcr ([E -twrci] * [Rwrcr twrcr] = [Rwrcr twrcr-twrci] )
             * Tcvcr = [Rwrcr twrcr-twrci]
            */
            Sophus::SE3 Tcvcr( Twc.rotation_matrix(),(Twc.translation()-TwcI.translation()) * compensate );  //这里乘以compensate是因为重建的误差导致的，相当于一个补偿
//            Sophus::SE3 Tcicr = TwcI.inverse()*Twc;  //Tc2c1 绝对增量，但位置不容易对准


            vtkRender->getPoseTcw(Tcvcr.inverse());


            //std::cout<<timer.elapsed()<<" s"<<std::endl;
        }

        //finished the ndi
        nditrack->FinishTracking();
    }

    std::cout<<"The main thread is done !"<<std::endl;

    return 0;
}

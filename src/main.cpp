#include "vtkpose.h"
#include "nditrack.h"

////opencv
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

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
    //等待NDI响应完成
    sleep(10);
    Sophus::SE3 TwcI;
    while(!vtkRender->IsFinished())
    {
    //---------------------------------------------------------------tracing------------------------------------------------------//
        boost::timer timer;
        Sophus::SE3 Twc = nditrack->getPoseTwc();
        if(!vtkRender->RequireReset())
        {
            TwcI = Twc;
            vtkRender->SetResetStatus(true);
            std::cout<<"Has reset the posture !"<<std::endl;
            continue;
        }

//            Sophus::SE3 TcwP = Tcw * TcwI.inverse();  //Tc2c1
        Sophus::SE3 TwcP(Twc.rotation_matrix(),Twc.translation()-TwcI.translation());  //Tc2c1
//            cout<<TwcP.matrix()<<endl;

        vtkRender->getPose(TwcP.inverse());
        cv::waitKey(10);
    }

    if(nditrack->FinishTrac())
    {
        std::cout<<"The main thread is done !"<<std::endl;
    }


    return 0;
}

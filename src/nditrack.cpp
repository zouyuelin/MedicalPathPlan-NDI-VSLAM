#include "nditrack.h"

NDItracking::NDItracking(bool checkTheRightPort, const string portname):PortName(portname),finished(false),state(PREPRING)
{
    tracking = std::make_shared<std::thread>(std::bind(&NDItracking::run,this));
    findThePortAuto = checkTheRightPort;
    transformMatrix << 0, -1 , 0,
                       1,  0 , 0,
                       0,  0  ,1;
//        transformMatrix << 1,  0, 0,
//                           0,  1 , 0,
//                           0,  0  ,1;
    cout<<"the transform matrix is:"<<endl<<transformMatrix<<endl;
}


void NDItracking::run()
{
    static CombinedApi capi = CombinedApi();

    checkTheUSBPortName(PortName);

    cout<<"The usb port name is:"<<PortName<<endl;

    ///connect
    if(capi.connect(PortName)!=0);
        cout<<"Please check the port name\n";
    sleep(1);

    ///initialize
    capi.initialize();

    std::cout << capi.getTrackingDataTX() << std::endl;

    int portHandle = capi.portHandleRequest();

    std::vector<PortHandleInfo> portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
    for (int i = 0; i < portHandles.size(); i++)
    {
        capi.portHandleInitialize(portHandles[i].getPortHandle());
        capi.portHandleEnable(portHandles[i].getPortHandle());
    }
    portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);

    ///start tracking
    capi.startTracking();

    {
        std::unique_lock<mutex> lc(mMutex);
        state = RUNNING;
    }
    while(true)
    {
        ///get the tool data
        std::vector<ToolData> toolData =  capi.getTrackingDataBX();

        for(auto &tl:toolData)
        {
            getThePostion(tl);
        }

        if(finished)
            break;
    }

    ///stop tracking

    capi.stopTracking();

    {
        std::unique_lock<mutex> lc(mMutex);
        state = STOP;
    }

}

void NDItracking::FinishTracking()
{
    std::unique_lock<mutex> lc(mMutex);
    finished = true;
}

void NDItracking::checkTheUSBPortName(string &name)
{
    //Initialize used by old api--------This is a bug

    ndicapi* device(nullptr);
    device = ndiOpenSerial(name.c_str());

    //Find the right ports

    if(device == nullptr)
    {
        if(findThePortAuto == false)
        {
            std::cout<<"Please check the port /dev/ttyUSB0 is connected !"<<std::endl;
            return;
        }

        cout<<"Trying find the right ports"<<endl;
        const int MAX_SERIAL_PORTS = 30;
        for (int i = 0; i < MAX_SERIAL_PORTS; ++i)
        {
          name = ndiSerialDeviceName(i);
          cout<<i<<" "<<name <<endl;
          int result = ndiSerialProbe(name.c_str(),true);
          if (result == NDI_OKAY)
          {
            break;
          }
        }
        device = ndiOpenSerial(name.c_str());
    }

}

void NDItracking::getThePostion(const ToolData& toolData)
{
    Eigen::Quaterniond rotation(toolData.transform.q0,toolData.transform.qx,toolData.transform.qy,toolData.transform.qz);
    Eigen::Vector3d translation(toolData.transform.tx,toolData.transform.ty,toolData.transform.tz);
    //误差 toolData.transform.error
    Sophus::SE3 mTwc_( transformMatrix * rotation.matrix(), transformMatrix * translation);

    if (toolData.transform.isMissing())
    {
        std::cout<< "Missing,,,,,,,,\n";
    }
    else
    {
        std::unique_lock<mutex> lc(mMutex);
        mTwc = mTwc_;
    }
}

Sophus::SE3 NDItracking::getPoseTwc()
{
    if(state != RUNNING)
    {
        return Sophus::SE3();
    }
    else
    {
        std::unique_lock<mutex> lc(mMutex);
        return mTwc;
    }
}

bool NDItracking::IsRunning()
{
    std::unique_lock<mutex> lc(mMutex);
    if (state == RUNNING || state == PREPRING)
        return true;
    else
        return false;
}

#include "frame.h"
#include "camera.h"

cv::FlannBasedMatcher Frame::matcher = cv::FlannBasedMatcher(new cv::flann::LshIndexParams(5,10,2));
cv::Ptr<cv::ORB> Frame::detector_ORB = cv::ORB::create(800,1.2,8);


Frame::Frame(cv::Mat img, cv::Mat depth)
{
    frame=img.clone();
    depth_ = depth.clone();
}

void Frame::detectKeyPoints()
{
    detector_ORB->detect(frame,keypoints);
}

void Frame::computeDescriptors()
{
    detector_ORB->compute(frame,keypoints,descriptors);
}

void Frame::ComputekeyPtsPw()
{
    for(auto keypoint:keypoints)
    {

        int x = cvRound(keypoint.pt.x);
        int y = cvRound(keypoint.pt.y);
        double dd=0;
        float d = depth_.ptr<float>(y)[x];
        if ( d!=0 )
        {
            dd = double(d);
        }
        else
        {
            // check the nearby points
            int dx[4] = {-1,0,1,0};
            int dy[4] = {0,-1,0,1};
            for ( int i=0; i<4; i++ )
            {
                d = depth_.ptr<float>( y+dy[i] )[x+dx[i]];
                if ( d!=0 )
                {
                    dd = double(d);
                }
            }
        }
        cv::Point3f p1_cw = dd * camera::pixel2cam(keypoint.pt, camera::K);
//        Eigen::Vector3d p1_wc = mTwc.rotation_matrix() * Eigen::Vector3d(p1_cw.x,p1_cw.y,p1_cw.z) + mTwc.translation();
        Eigen::Vector4d p1_wc = mTwc.matrix()*Eigen::Vector4d(p1_cw.x,p1_cw.y,p1_cw.z,1) ;
        this->pts_3d.push_back(p1_wc.head(3));
        this->pts_3d_cam.push_back(Eigen::Vector3d(p1_cw.x,p1_cw.y,p1_cw.z));
    }
}

void Frame::setPose(Sophus::SE3 mTwc)
{
    this->mTwc = mTwc;
}

Sophus::SE3 Frame::getPose()
{
    return mTwc;
}

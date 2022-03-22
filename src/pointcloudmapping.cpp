#include "pointcloudmapping.h"

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    //globalMap = boost::shared_ptr< PointCloud >(new PointCloud());
    globalMap =  std::make_shared< PointCloud >(); //PointCloud::Ptr(new PointCloud());
    viewerThread = std::make_shared<std::thread>( std::bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        std::unique_lock<std::mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(Frame kf)
{
    std::unique_lock<std::mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( kf.frame.clone());
    depthImgs.push_back( kf.depth_.clone() );

    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(Frame kf, cv::Mat& color, cv::Mat& depth)
{
    PointCloud::Ptr tmp( new PointCloud() );
    // point cloud is null ptr
    for ( int m=0; m<depth.rows; m++ )
    {
        for ( int n=0; n<depth.cols; n++ )
        {
            float d = (float)depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - camera::K_Eigen(0,0)) * p.z / camera::K_Eigen(0,0);
            p.y = ( m - camera::K_Eigen(1,1)) * p.z / camera::K_Eigen(1,1);

            p.b = color.ptr<uchar>(m)[n*3];
            p.g = color.ptr<uchar>(m)[n*3+1];
            p.r = color.ptr<uchar>(m)[n*3+2];

            tmp->points.push_back(p);
        }
    }

    Eigen::Isometry3d T(kf.mTwc.matrix());
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.matrix());
    cloud->is_dense = false;

//    std::cout<<"cloud size="<<cloud->points.size()<<std::endl;
    return cloud;
}


void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            std::unique_lock<std::mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            std::unique_lock<std::mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            std::unique_lock<std::mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
            *globalMap += *p;
        }
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( globalMap );
        voxel.filter( *tmp );
        globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        std::cout<<"show global map, size="<<globalMap->points.size()<<std::endl;
        lastKeyframeSize = N;
    }
}

#ifndef ONNXRUN_H
#define ONNXRUN_H

//onnxruntime
#include <core/session/onnxruntime_cxx_api.h>
#include <core/providers/cuda/cuda_provider_factory.h>
#include <core/session/onnxruntime_c_api.h>
#include <core/providers/tensorrt/tensorrt_provider_factory.h>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <sophus/se3.h>

using namespace cv;

class DLrun{

public:
    DLrun(std::string model = "./model.onnx");
    Sophus::SE3 computePoseDNN(Mat img_1, Mat img_2);
    void printModelInfo();
    static Eigen::Vector3d ToEulerAngles(Eigen::Quaterniond q);

private:
    Ort::Env env ;
    Ort::SessionOptions session_options;
    Ort::MemoryInfo memory_info;
    Ort::AllocatorWithDefaultOptions allocator;
    Ort::Session *session;
};

#endif // ONNXRUN_H

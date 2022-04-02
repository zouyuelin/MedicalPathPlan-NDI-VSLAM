#include "onnxrun.h"
#include <iostream>

using namespace cv;
using namespace std;
//输入网络的维度
static constexpr const int width = 512;
static constexpr const int height = 512;
static constexpr const int channel = 3;
std::array<int64_t, 4> input_shape_{ 1,height, width,channel};

DLrun::DLrun(std::string model):memory_info(Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault))

{
    env = Ort::Env{ORT_LOGGING_LEVEL_ERROR, "Default"};
    //CUDA加速开启
    //OrtSessionOptionsAppendExecutionProvider_Tensorrt(session_options, 0);
    OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session = new Ort::Session(env, model.c_str(), session_options);

    printModelInfo();
}

Sophus::SE3 DLrun::computePoseDNN(Mat img_1, Mat img_2)
{
    Mat Input_1,Input_2;
    resize(img_1,Input_1,Size(512,512));
    resize(img_2,Input_2,Size(512,512));
    std::vector<const char*> input_node_names = {"input1","input2"};
    std::vector<const char*> output_node_names = {"Output"};

    //将图像存储到uchar数组中，BGR--->RGB
    std::array<float, width * height *channel> input_image_1{};
    std::array<float, width * height *channel> input_image_2{};

    float* input_1 =  input_image_1.data();
    float* input_2 =  input_image_2.data();

    for (int i = 0; i < Input_1.rows; i++) {
        for (int j = 0; j < Input_1.cols; j++) {
            for (int c = 0; c < 3; c++)
            {
                //NHWC 格式
                if(c==0)
                    input_1[i*Input_1.cols*3+j*3+c] = Input_1.ptr<uchar>(i)[j*3+2]/255.0;
                if(c==1)
                    input_1[i*Input_1.cols*3+j*3+c] = Input_1.ptr<uchar>(i)[j*3+1]/255.0;
                if(c==2)
                    input_1[i*Input_1.cols*3+j*3+c] = Input_1.ptr<uchar>(i)[j*3+0]/255.0;
                //NCHW 格式
//                if (c == 0)
//                     input_1[c*imgSource.rows*imgSource.cols + i * imgSource.cols + j] = imgSource.ptr<uchar>(i)[j * 3 + 2];
//                if (c == 1)
//                     input_1[c*imgSource.rows*imgSource.cols + i * imgSource.cols + j] = imgSource.ptr<uchar>(i)[j * 3 + 1];
//                if (c == 2)
//                     input_1[c*imgSource.rows*imgSource.cols + i * imgSource.cols + j] = imgSource.ptr<uchar>(i)[j * 3 + 0];


            }
        }
    }
    for (int i = 0; i < Input_2.rows; i++) {
        for (int j = 0; j < Input_2.cols; j++) {
            for (int c = 0; c < 3; c++)
            {
                //NHWC 格式
                if(c==0)
                    input_2[i*Input_2.cols*3+j*3+c] = Input_2.ptr<uchar>(i)[j*3+2]/255.0;
                if(c==1)
                    input_2[i*Input_2.cols*3+j*3+c] = Input_2.ptr<uchar>(i)[j*3+1]/255.0;
                if(c==2)
                    input_2[i*Input_2.cols*3+j*3+c] = Input_2.ptr<uchar>(i)[j*3+0]/255.0;
            }
        }
    }

    std::vector<Ort::Value> input_tensors;
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, input_1, input_image_1.size(), input_shape_.data(), input_shape_.size()));
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, input_2, input_image_2.size(), input_shape_.data(), input_shape_.size()));
    //不知道输入维度时
    //input_tensors.push_back(Ort::Value::CreateTensor<uchar>(
    //        memory_info, input, input_image_.size(), input_dims.data(), input_dims.size()));

    std::vector<Ort::Value> output_tensors;

    output_tensors = session->Run(Ort::RunOptions { nullptr },
                                    input_node_names.data(), //输入节点名
                                    input_tensors.data(),     //input tensors
                                    input_tensors.size(),     //2
                                    output_node_names.data(), //输出节点名
                                    output_node_names.size()); //1

//    cout<<output_tensors.size()<<endl;//输出的维度
    float* output = output_tensors[0].GetTensorMutableData<float>();
    Eigen::Vector3d t(output[0],output[1],output[2]);
    Eigen::Vector3d r(output[3],output[4],output[5]);

    // 初始化旋转向量，绕z轴旋转，y轴，x轴；
    Eigen::AngleAxisd R_z(r[2], Eigen::Vector3d(0,0,1));
    Eigen::AngleAxisd R_y(r[1], Eigen::Vector3d(0,1,0));
    Eigen::AngleAxisd R_x(r[0], Eigen::Vector3d(1,0,0));
    // 转换为旋转矩阵
    Eigen::Matrix3d R_matrix_xyz  = R_z.toRotationMatrix()*R_y.toRotationMatrix()*R_x.toRotationMatrix();

    return Sophus::SE3(R_matrix_xyz,t);
}

void DLrun::printModelInfo()
{
    //输出模型输入节点的数量
    size_t num_input_nodes = session->GetInputCount();
    size_t num_output_nodes = session->GetOutputCount();
    cout<<"Number of input node is:"<<num_input_nodes<<endl;
    cout<<"Number of output node is:"<<num_output_nodes<<endl;

    //获取输入输出维度
    for(auto i = 0; i<num_input_nodes;i++)
    {
        std::vector<int64_t> input_dims = session->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
        cout<<endl<<"input "<<i<<" dim is: ";
        for(auto j=0; j<input_dims.size();j++)
            cout<<input_dims[j]<<" ";
    }
    for(auto i = 0; i<num_output_nodes;i++)
    {
        std::vector<int64_t> output_dims = session->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
        cout<<endl<<"output "<<i<<" dim is: ";
        for(auto j=0; j<output_dims.size();j++)
            cout<<output_dims[j]<<" ";
    }
    //输入输出的节点名
    cout<<endl;//换行输出
    for(auto i = 0; i<num_input_nodes;i++)
        cout<<"The input op-name "<<i<<" is:"<<session->GetInputName(i, allocator)<<endl;
    for(auto i = 0; i<num_output_nodes;i++)
        cout<<"The output op-name "<<i<<" is:"<<session->GetOutputName(i, allocator)<<endl;

    //input_dims_2[0] = input_dims_1[0] = output_dims[0] = 1;//batch size = 1
}

Eigen::Vector3d DLrun::ToEulerAngles(Eigen::Quaterniond q) {
    ///
    /// roll = atan2( 2(q0*q1+q2*q3),1-2(q1^2 + q2^2))
    /// pitch = asin(2(q0*q2 - q3*q1))
    /// yaw = atan2(2(q0*q3+q1*q2),1-2(q2^2 + q3^2))
    ///
    double x,y,z;//roll pitch yaw

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    x = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        y = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    z = std::atan2(siny_cosp, cosy_cosp);

    return Eigen::Vector3d(x,y,z);
}

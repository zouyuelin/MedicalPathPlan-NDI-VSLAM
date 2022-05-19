cmake_minimum_required(VERSION 2.8)


if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
endif()

project(NDI_AUTO LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

set( CMAKE_BUILD_TYPE "Debug" )

set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")
set(CMAKE_C_COMPILER "/usr/bin/gcc-7")

#******openMp 多线程****
find_package(OpenMP REQUIRED)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -pthread -O3 -Wall") # ${OpenMP_CXX_FLAGS} = -fopenmp
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
#******onnxruntime*****
set(ONNXRUNTIME_ROOT_PATH /home/zyl/ubuntu/tensorRT/onnxruntime-master)
set(ONNXRUNTIME_INCLUDE_DIRS ${ONNXRUNTIME_ROOT_PATH}/include/onnxruntime
                             ${ONNXRUNTIME_ROOT_PATH}/onnxruntime
                             ${ONNXRUNTIME_ROOT_PATH}/include/onnxruntime/core/session/)
set(ONNXRUNTIME_LIB ${ONNXRUNTIME_ROOT_PATH}/build/Linux/Release/libonnxruntime.so)
#NDI
set(NDI_FILE /home/zyl/ubuntu_HDD/NDI/NDI)
INCLUDE_DIRECTORIES(${NDI_FILE}/library/include )
set(NDI_LIB ${NDI_FILE}/build/linux/libndicapi.so)
# ndicapi
find_package(ndicapi REQUIRED)
INCLUDE_DIRECTORIES(${ndicapi_INCLUDE_DIRS})
# Eigen3
find_package( Eigen3 REQUIRED )
#Sophus
find_package( Sophus REQUIRED)
#boost
find_package( Boost REQUIRED COMPONENTS system filesystem)
# OpenCV
find_package( OpenCV 4.5.3 EXACT REQUIRED)
# VMTK
find_package( VMTK REQUIRED)
include(${VMTK_USE_FILE})
# ITK
find_package( ITK REQUIRED)
include(${ITK_USE_FILE})
# VTK
find_package( VTK 8.1.2 EXACT REQUIRED)
include(${VTK_USE_FILE})

INCLUDE_DIRECTORIES(
                ${PROJECT_SOURCE_DIR}//include
                ${OpenCV_INCLUDE_DIRS}
                ${EIGEN3_INCLUDE_DIR}
                ${Sophus_INCLUDE_DIRS}
                ${BOOST_INCLUDE_DIRS}
                ${VTK_INCLUDE_DIRS}
                ${ONNXRUNTIME_INCLUDE_DIRS}
                )

add_executable(${PROJECT_NAME}
src/TrackingSystem.cpp
src/nditrack.cpp
src/vtkpose.cpp
src/Centerline.cpp
src/interactorStyleCenterline.cpp
src/pathSetting.cpp
src/cameraRender.cpp
src/onnxrun.cpp
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
         ${OpenCV_LIBS}
         ${Boost_LIBRARIES}
         ${NDI_LIB}
         ${ndicapi_LIBRARIES}
         ${VTK_LIBRARIES}
         ${Sophus_LIBRARIES}
         ${ONNXRUNTIME_LIB}
         )


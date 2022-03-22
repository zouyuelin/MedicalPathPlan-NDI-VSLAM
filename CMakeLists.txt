cmake_minimum_required(VERSION 2.8)


if(POLICY CMP0020)
  cmake_policy(SET CMP0020 NEW)
endif()

project(MYSLAM LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

set( CMAKE_BUILD_TYPE "Debug" )

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_C_COMPILER "gcc")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -pthread -O3 -Wall")
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# librealsense
file(GLOB librealsense2 /usr/local/lib/librealsense*)
set(Realsense2_INCLUDE_DIR /usr/local/include/librealsense2)
#NDI
set(NDI_FILE /home/zyl/ubuntu_HDD/NDI/NDI)
INCLUDE_DIRECTORIES(${NDI_FILE}/library/include )
set(NDI_LIB ${NDI_FILE}/build/linux/libndicapi.so)
# ndicapi
find_package(ndicapi REQUIRED)
INCLUDE_DIRECTORIES(${ndicapi_INCLUDE_DIRS})
# DBow3
set( DBoW3_INCLUDE_DIRS "/usr/local/include/" )
set( DBoW3_LIBS "/usr/local/lib/libDBoW3.so" )
# Eigen3
find_package( Eigen3 REQUIRED )
# pangoling
find_package( Pangolin REQUIRED)
# G2O
find_package( G2O REQUIRED )
#Sophus
find_package( Sophus REQUIRED)
#Cholmod
find_package( Cholmod REQUIRED)
#boost
find_package( Boost REQUIRED COMPONENTS system filesystem)
# OpenCV
find_package( OpenCV 4.5.1 EXACT REQUIRED)
# VMTK
find_package( VMTK REQUIRED)
include(${VMTK_USE_FILE})
# ITK
find_package( ITK REQUIRED)
include(${ITK_USE_FILE})
# VTK
find_package( VTK 8.1.2 EXACT REQUIRED)
include(${VTK_USE_FILE})
# PCL
#find_package(PCL 1.11.0 EXACT REQUIRED) #1.10.0 EXACT
#add_definitions( ${PCL_DEFINITIONS} )
file(GLOB PCL_LIBRARIES /usr/local/lib/libpcl_*)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.11)

INCLUDE_DIRECTORIES(
                ${PROJECT_SOURCE_DIR}//include
                ${CHOLMOD_INCLUDE_DIR}
                ${OpenCV_INCLUDE_DIRS}
                ${Pangolin_INCLUDE_DIRS}
                ${EIGEN3_INCLUDE_DIR}
                ${Sophus_INCLUDE_DIRS}
                ${Realsense2_INCLUDE_DIR}
                ${DBoW3_INCLUDE_DIRS}
                ${PCL_INCLUDE_DIRS}
                ${BOOST_INCLUDE_DIRS}
                ${VTK_INCLUDE_DIRS}
                )

add_executable(${PROJECT_NAME}
src/main.cpp
src/nditrack.cpp
src/vtkpose.cpp
src/voslam.cpp
src/frame.cpp
src/loopclosing.cpp
src/optimizer.cpp
src/pointcloudmapping.cpp
src/visualmap.cpp
src/map.cpp
src/camera.cpp
src/Centerline.cpp
src/interactorStyleCenterline.cpp
src/pathSetting.cpp
src/cameraRender.cpp
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
         ${OpenCV_LIBS}
         ${Sophus_LIBRARIES}
         ${CHOLMOD_LIBRARIES}
         g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sim3 g2o_types_sba
         ${Pangolin_LIBRARIES}
         ${Boost_LIBRARIES}
         ${librealsense2}
         ${DBoW3_LIBS}
         ${PCL_LIBRARIES}
         ${NDI_LIB}
         ${ndicapi_LIBRARIES}
         ${VTK_LIBRARIES}
         )

 message(STATUS "OPENCV is :${OpenCV_INCLUDE_DIRS}")
 message(STATUS "OPENCV version is :${OpenCV_VERSION}")
 message(STATUS "the cholmod is ${CHOLMOD_INCLUDE_DIR} ${CHOLMOD_LIBRARIES} ")
 message(STATUS "Eigen and boost :${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS}")
 message(STATUS "****: ${PCL_INCLUDE_DIRS} ${PCL_LIBRARIES}")


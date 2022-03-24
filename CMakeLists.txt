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

INCLUDE_DIRECTORIES(
                ${PROJECT_SOURCE_DIR}//include
                ${OpenCV_INCLUDE_DIRS}
                ${EIGEN3_INCLUDE_DIR}
                ${Sophus_INCLUDE_DIRS}
                ${BOOST_INCLUDE_DIRS}
                ${VTK_INCLUDE_DIRS}
                )

add_executable(${PROJECT_NAME}
src/main.cpp
src/nditrack.cpp
src/vtkpose.cpp
src/Centerline.cpp
src/interactorStyleCenterline.cpp
src/pathSetting.cpp
src/cameraRender.cpp
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
         ${OpenCV_LIBS}
         ${Boost_LIBRARIES}
         ${NDI_LIB}
         ${ndicapi_LIBRARIES}
         ${VTK_LIBRARIES}
         ${Sophus_LIBRARIES}
         )


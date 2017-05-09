cmake_minimum_required(VERSION 2.8)
project( RGBD-Recorder )

set(OpenCV_STATIC OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( OpenCV REQUIRED )
#find_package( RealSense REQUIRED)

set( OPENNI2_INCLUDE /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Include)
set( OPENNI2_REDIST  /home/ubuntu/Downloads/2-Linux/OpenNI-Linux-Arm-2.3/Redist)
include_directories( ${OPENNI2_INCLUDE} )
link_directories ( ${OPENNI2_REDIST} )
# find_package( OpenNI2 REQUIRED)

set(libname kinect-cv2 )

#define include
include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${RealSense_INCLUDE_DIR})
# include_directories(${OPENNI2_INCLUDE_DIRS})

set( TumRecorder tumrecorder)
set( sourceFiles
TumRecorder.cpp 
NISensorController.h 
NISensorController.cpp)

add_executable( ${TumRecorder} ${sourceFiles} )
target_link_libraries( ${TumRecorder} ${OpenCV_LIBS} OpenNI2)
#target_link_libraries( TumRecorder ${RealSense_LIBRARY} )
# target_link_libraries( TumRecorder ${OPENNI2_LIBRARY} )

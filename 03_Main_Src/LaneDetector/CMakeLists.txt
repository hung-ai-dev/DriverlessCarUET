cmake_minimum_required(VERSION 2.8)
project( LaneDetector )

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(OpenCV_STATIC OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")
set(CMAKE_SUPPRESS_REGENERATION true) #remove Zerocheck

if(UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	SET(CUDA_PROPAGATE_HOST_FLAGS OFF)
endif()

#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")

find_package( OpenCV REQUIRED )
find_package( Eigen REQUIRED )

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

INCLUDE(CMakeModules/CudaUtils.cmake)
# enable_cuda_support()
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11" )

#define include
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

file(GLOB Utils ./Utilities/DatasetReader.h ./Utilities/DatasetReader.cpp)
file(GLOB ImgProc ./ImgProc/*.h ./ImgProc/*.cpp)
file(GLOB ImgProcCUDA ./ImgProc/cuda/*.cu ./ImgProc/cuda/*.h)

cuda_add_library(libImgProc STATIC ${ImgProc} ${ImgProcCUDA} OPTIONS -std=c++11)
target_link_libraries( libImgProc ${OpenCV_LIBS})
add_library(libDataset STATIC ${Utils})
target_link_libraries( libDataset ${OpenCV_LIBS} )

# Project 01
add_executable( Test_LaneDetection Test_LaneDetection.cpp)
target_link_libraries( Test_LaneDetection ${OpenCV_LIBS} libDataset libImgProc)

if(Build_Test_LaneDetection_CUDA)
	#add_executable( Test_LaneDetection_CUDA Test_LaneDetection_CUDA.cpp)
	#target_link_libraries( Test_LaneDetection_CUDA ${OpenCV_LIBS} libDataset libImgProc )
endif()

#Project 02 Device runnning
set(Build_Test_LaneDetection_Astra false CACHE BOOL "Build testing Astra")
set(Build_Recorder false CACHE BOOL "Build astra recorder")
set(Build_Test_LaneDetection_CUDA false CACHE BOOL "Build testing CUDA")
set(Build_Peripheral_Driver false CACHE BOOL "Build CAR DRIVER")

if(Build_Recorder OR Build_Test_LaneDetection_Astra OR Build_Peripheral_Driver)
	find_package( OpenNI2 REQUIRED )
	include_directories( ${OPENNI2_INCLUDE_DIRS} )
endif()

if(Build_Recorder)
	add_executable( Recorder Recorder.cpp OpenNI2_Helper.h)
	target_link_libraries( Recorder ${OpenCV_LIBS} ${OPENNI2_LIBRARY} libDataset libImgProc)
endif()

if(Build_Test_LaneDetection_Astra)
	add_executable( Test_LaneDetection_Astra Test_LaneDetection_Astra.cpp OpenNI2_Helper.h)
	target_link_libraries( Test_LaneDetection_Astra ${OpenCV_LIBS} ${OPENNI2_LIBRARY} libDataset libImgProc )

	add_executable( LaneCalibration LaneCalibration.cpp OpenNI2_Helper.h)
	target_link_libraries( LaneCalibration ${OpenCV_LIBS} ${OPENNI2_LIBRARY} libDataset libImgProc)
endif()

if(Build_Peripheral_Driver)
# Project 00
    add_subdirectory(peripheral_driver)
    add_executable(Test_Peripheral_Driver Test_RunCar.cpp OpenNI2_Helper.h)
    add_executable(Test_Theta_Manual testSetThetaManual.cpp)
    add_executable(Test_Theta_Auto testSetThetaAuto.cpp)
    target_link_libraries( Test_Peripheral_Driver ${OpenCV_LIBS} ${OPENNI2_LIBRARY} libImgProc i2c-pwm uart)
    target_link_libraries(Test_Theta_Manual i2c-pwm)
    target_link_libraries(Test_Theta_Auto i2c-pwm)
endif()

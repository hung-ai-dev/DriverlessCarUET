cmake_minimum_required(VERSION 2.8)
project( Lane_Center_3D )

set(OpenCV_STATIC OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")
set(CMAKE_SUPPRESS_REGENERATION true) #remove Zerocheck
if(UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true;--prec-div=false;--prec-sqrt=false")

find_package( OpenCV REQUIRED )
find_package( Eigen REQUIRED )
# find_package(PCL 1.8 REQUIRED)
# add_definitions(${PCL_DEFINITIONS})

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

INCLUDE(CMakeModules/CudaUtils.cmake)
enable_cuda_support()
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11" )

#define include
include_directories(${EIGEN_INCLUDE_DIRS})
# include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

file(GLOB Utils ./Utilities/DatasetReader.h ./Utilities/DatasetReader.cpp)
file(GLOB ImgProc ./ImgProc/*.h ./ImgProc/*.cpp)
file(GLOB ImgProcCUDA ./ImgProc/cuda/*.cu ./ImgProc/cuda/*.h)


# source_group("Core"        		FILES ${Core} )
# source_group("ImgProc"        	FILES ${ImgProc} )

# CUDA_COMPILE(imgproc_cuda_objs ${ImgProcCUDA})

cuda_add_library(libImgProc STATIC ${ImgProc} ${ImgProcCUDA} OPTIONS -std=c++11)
target_link_libraries( libImgProc ${OpenCV_LIBS})
add_library(libDataset STATIC ${Utils})
target_link_libraries( libDataset ${OpenCV_LIBS} )

#Project 01 Dataset Reader
add_executable( Viewer3D Viewer3D.cpp)
target_link_libraries( Viewer3D ${OpenCV_LIBS} libDataset)

add_executable( Test_LaneDetection3D Test_LaneDetection3D.cpp)
target_link_libraries( Test_LaneDetection3D ${OpenCV_LIBS} libDataset libImgProc)

#Project 02 Device runnning
set(Build_Test_LaneDetection_Astra false CACHE BOOL "Build testing Astra")
set(Build_Test_LaneDetection_CUDA false CACHE BOOL "Build testing CUDA")

if(Build_Test_LaneDetection_CUDA)
	add_executable( Test_LaneDetection_CUDA Test_LaneDetection_CUDA.cpp)
	target_link_libraries( Test_LaneDetection_CUDA ${OpenCV_LIBS} libDataset libImgProc )
endif()

if(Build_Test_LaneDetection_Astra)

	#FIND OpenNI2 ... 
	find_package( OpenNI2 REQUIRED )
	include_directories( ${OPENNI2_INCLUDE_DIRS} )
	add_executable( Test_LaneDetection_Astra Test_LaneDetection_Astra.cpp)
	target_link_libraries( Test_LaneDetection_Astra ${OpenCV_LIBS} ${OPENNI2_LIBRARY} libDataset libImgProc )
endif()
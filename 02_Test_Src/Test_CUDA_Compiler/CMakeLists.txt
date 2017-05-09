cmake_minimum_required(VERSION 2.8)
project( Lane_Center_3D )

set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

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

file(GLOB ImgProc_src ./ImgProc/*.h ./ImgProc/cuda/*.h ./ImgProc/*.cpp)
file(GLOB ImgProcCUDA_src ./ImgProc/cuda/*.cu ./ImgProc/cuda/*.h  ./ImgProc/types.h)

#CUDA_COMPILE(imgproc_cuda_objs ${ImgProcCUDA_src} OPTION -std=c++11)

cuda_add_library(libImgProcCUDA STATIC ${ImgProc_src} ${ImgProcCUDA_src} OPTIONS -std=c++11)
target_link_libraries( libImgProcCUDA ${OpenCV_LIBS})

#add_library(libImgProc STATIC ${ImgProc_src})
#target_link_libraries( libImgProc ${OpenCV_LIBS})

# add_executable( Test_CUDA Test_CUDA.cpp ./ImgProc/ImgProc.h ./ImgProc/ImgProc.cpp)
add_executable( Test_CUDA Test_CUDA.cpp)
target_link_libraries( Test_CUDA ${OpenCV_LIBS} libImgProcCUDA)

add_executable( Test_Kalman Test_Kalman.cpp)
target_link_libraries( Test_Kalman ${OpenCV_LIBS})

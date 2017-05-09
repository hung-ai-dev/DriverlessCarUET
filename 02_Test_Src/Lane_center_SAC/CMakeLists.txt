cmake_minimum_required(VERSION 2.8)

project( OpenCV-Kinect )
find_package( OpenCV REQUIRED )

set(libname kinect-cv2 )
set(OpenCV_STATIC OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")
set(CMAKE_SUPPRESS_REGENERATION true) #remove Zerocheck
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(source_files
	main.cpp
)

add_library( ${libname} ${source_files})

target_link_libraries(${test_exec_name2} ${libname} ${OpenCV_LIBS})


set (test_exec_name2 test-main)

add_executable( ${test_exec_name2} main.cpp)

target_link_libraries(${test_exec_name2} ${libname})

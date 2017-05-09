cmake_minimum_required(VERSION 2.8)

project( OpenCV-Kinect )
find_package( OpenCV REQUIRED )

set(libname kinect-cv2 )

set(source_files
    polyfit.c
    polyfit.h
    helper.cpp
    helper.hpp
	main.cpp
)

add_library( ${libname} ${source_files})

target_link_libraries(${test_exec_name2} ${libname} ${OpenCV_LIBS})


set (test_exec_name2 test-main)

add_executable( ${test_exec_name2} main.cpp)

target_link_libraries(${test_exec_name2} ${libname})

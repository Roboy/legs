cmake_minimum_required (VERSION 2.8) 

project (raspicam_opencv)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake/")

find_package(raspicam REQUIRED)
find_package(OpenCV REQUIRED)

add_executable (raspicam_opencv main.cpp)  
target_link_libraries (raspicam_opencv ${raspicam_CV_LIBS})

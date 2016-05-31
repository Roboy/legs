cmake_minimum_required(VERSION 2.8.3)
project(acc_gyro_publisher)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

SET(CMAKE_BUILD_TYPE Debug)

## Find catkin and any catkin packages
find_package( PkgConfig )

find_package(catkin REQUIRED COMPONENTS roscpp)

pkg_check_modules(gtkmm-3.0 REQUIRED)
 
## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
	include 
	MPU6050-Pi-Demo
	${catkin_INCLUDE_DIRS}
)

############ POSITION CONTROLLER #############
add_executable(acc_gyro_publisher src/main.cpp MPU6050-Pi-Demo/I2Cdev.cpp MPU6050-Pi-Demo/MPU6050.cpp)
target_link_libraries(acc_gyro_publisher ${catkin_LIBRARIES} ${gtkmm-3.0_LIBRARIES})

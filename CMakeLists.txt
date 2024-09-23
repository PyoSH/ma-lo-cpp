cmake_minimum_required(VERSION 3.5)
project(ma-lo-cpp)

set(CMAKE_CXX_STANDARD 17)
#list(APPEND CMAKE_MODULE_PATH ${CMAKE_PREFIX_PATH})

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        nav_msgs
        sensor_msgs
        cv_bridge
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

find_package(PCL REQUIRED)
find_package(pcl_conversions)
find_package(pcl_ros)

add_executable(mapping_test mapping/main.cpp
        mapping/src/map_RT.cpp
        mapping/src/map_RT.h)
target_link_libraries(mapping_test
        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
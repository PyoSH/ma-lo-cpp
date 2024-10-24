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
include_directories(tools ${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

find_package(PCL REQUIRED)
find_package(pcl_conversions)
find_package(pcl_ros)

set(tool_source
        tools/tool.cpp
        tools/tool.h
        tools/nanoflann.hpp)

set(mcl_source
        localization/src/mcl.cpp
        localization/src/mcl.h)

set(mapping_source
        mapping/src/map_RT.cpp
        mapping/src/map_RT.h)

# MAPPING
add_executable(mapping_test mapping/main.cpp ${mapping_source} ${tool_source})
target_link_libraries(mapping_test
        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# MCL
add_executable(mcl_test localization/main.cpp ${mcl_source} ${tool_source})
target_link_libraries(mcl_test
        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# tool-merge PCs
add_executable(mergePC tools/mergePC.cpp ${tool_source})
target_link_libraries(mergePC
        ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# test-segmentGround
add_executable(segmentGround tools/segmentGround.cpp ${tool_source})
target_link_libraries(segmentGround
        ${catkin_LIBRARIES} ${PCL_LIBRARIES})
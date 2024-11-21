cmake_minimum_required(VERSION 2.8.3)
project(gazebo_set)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
    tf2
    tf2_ros
    tf
    pcl_conversions
    pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror


 
add_executable ( camera_pose
    src/pcl_camera_pose_pointcloud.cpp )

target_link_libraries( camera_pose
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} ) 
    

    

    

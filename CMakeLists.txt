cmake_minimum_required(VERSION 2.8.3)
project(pose_vizualization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  std_msgs
 cv_bridge
)

## Declare a catkin package
catkin_package(
 CATKIN_DEPENDS std_msgs roscpp 
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(vizualize src/vizualize.cpp)
target_link_libraries(vizualize ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(vizualize vizualize_cpp)


add_executable(run_pose_estimation_for_scene_screenshot src/run_pose_estimation_for_scene_screenshot.cpp)
target_link_libraries(run_pose_estimation_for_scene_screenshot ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(run_pose_estimation_for_scene_screenshot run_pose_estimation_for_scene_screenshot_cpp)




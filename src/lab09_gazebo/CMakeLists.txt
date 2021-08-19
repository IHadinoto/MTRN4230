cmake_minimum_required(VERSION 3.0.2)
project(lab09_gazebo)

add_compile_options(-std=c++17)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
      moveit_core
      moveit_ros_planning
      moveit_ros_planning_interface
      roslaunch
)

# Perform checks on launch files too
roslaunch_add_file_check(launch)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
      moveit_core
      moveit_ros_planning_interface
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
cmake_minimum_required(VERSION 3.0.2)
project(nodes)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  roslaunch
  cv_bridge
  rospy
  std_msgs
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
  moveit_core
  moveit_ros_planning_interface
)


roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(user_IO src/user_IO.cpp)
target_link_libraries(user_IO ${catkin_LIBRARIES})

add_executable(demo_IO src/demo_IO.cpp)
target_link_libraries(demo_IO ${catkin_LIBRARIES})

add_executable(test_controller_camera src/test_controller_camera.cpp)
target_link_libraries(test_controller_camera ${catkin_LIBRARIES})

add_executable(test_controller_movement src/test_controller_movement.cpp)
target_link_libraries(test_controller_movement ${catkin_LIBRARIES})

add_executable(test_controller_environment src/test_controller_environment.cpp)
target_link_libraries(test_controller_environment ${catkin_LIBRARIES})

add_executable(environment src/environment.cpp)
target_link_libraries(environment ${catkin_LIBRARIES})

add_executable(movement src/movement.cpp)
target_link_libraries(movement ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(independent_movement src/independent_movement.cpp)
target_link_libraries(independent_movement ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_movement_2 src/test_movement_2.cpp)
target_link_libraries(test_movement_2 ${catkin_LIBRARIES})

add_executable(character_detection src/character_detection.cpp)
target_link_libraries(character_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(character_recognition src/character_recognition.cpp)
target_link_libraries(character_recognition ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries (character_recognition tesseract)


add_executable(test_env src/test_env.cpp)
target_link_libraries(test_env ${catkin_LIBRARIES})

add_executable(positions_listener src/positions_listener.cpp)
target_link_libraries(positions_listener ${catkin_LIBRARIES})

add_executable(image_listener src/image_listener.cpp)
target_link_libraries(image_listener ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

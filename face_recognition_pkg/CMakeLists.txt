cmake_minimum_required(VERSION 3.5)
project(face_recognition_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  roscpp
)

find_package(OpenCV REQUIRED)

## Declare ROS messages
add_message_files(
  FILES
  DetectionInfo.msg
  DetectionInfoArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

# If you have a C++ node, uncomment and adjust this:
# add_executable(face_recognition_node src/face_recognition_node.cpp)

# Link the executable to ROS and OpenCV libraries
# target_link_libraries(face_recognition_node
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )


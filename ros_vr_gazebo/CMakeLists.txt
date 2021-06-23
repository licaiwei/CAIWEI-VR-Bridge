cmake_minimum_required(VERSION 3.0.2)
project(ros_vr_gazebo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  /usr/lib/x86_64-linux-gnu
)

add_executable(vr_device_states_controller src/vr_device_states_controller.cpp)
target_link_libraries(vr_device_states_controller 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
)

add_executable(combine_pic src/combine_pic.cpp)
target_link_libraries(combine_pic 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
)
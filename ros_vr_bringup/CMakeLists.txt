cmake_minimum_required(VERSION 3.0.2)
project(ros_vr_bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  tf
  cv_bridge
  #OpenCV 3.2.0 REQUIRED
)


catkin_package(
#  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ./include
  /usr/include
)

link_directories(
	./lib
  /usr/include
  /usr/lib/x86_64-linux-gnu
)

add_executable(vr_bringup src/vr_bringup.cpp)
target_link_libraries(vr_bringup 
  ${catkin_LIBRARIES} 
  libglut.so
  libGLU.so
  libGL.so
  libopenvr_api.so
  ${OpenCV_LIBS}
)

add_executable(tf_state_publisher src/tf_state_publisher.cpp)
target_link_libraries(tf_state_publisher 
  ${catkin_LIBRARIES} 
  libglut.so
  libGLU.so
  libGL.so
  libopenvr_api.so
  ${OpenCV_LIBS}
)

# add_executable(submit_image_node src/submit_image_node.cpp)
# target_link_libraries(submit_image_node 
#   ${catkin_LIBRARIES} 
#   libglut.so
#   libGLU.so
#   libGL.so
#   libopenvr_api.so
#   ${OpenCV_LIBS}
# )
cmake_minimum_required(VERSION 3.0.2)
project(lidar2odom)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  message_filters
  tf
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs message_filters tf pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/pc_transformer.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

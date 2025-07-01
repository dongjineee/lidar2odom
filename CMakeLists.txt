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

add_executable(pc_transformer_node src/pc_transformer.cpp)
add_executable(map2odom_node src/map2odom_tf_pub.cpp)

target_link_libraries(pc_transformer_node
  ${catkin_LIBRARIES}
)

target_link_libraries(map2odom_node
  ${catkin_LIBRARIES}
)
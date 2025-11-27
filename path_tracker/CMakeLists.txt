cmake_minimum_required(VERSION 3.8)
project(path_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(pid_path_tracker_node src/pid_path_tracker_node.cpp)

ament_target_dependencies(pid_path_tracker_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(tf_to_pose_publisher_node src/tf_to_pose_publisher_node.cpp)
ament_target_dependencies(tf_to_pose_publisher_node rclcpp tf2_ros geometry_msgs)

add_executable(path_publisher_node src/path_publisher_node.cpp)
ament_target_dependencies(path_publisher_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(path_topic_publisher_node src/path_topic_publisher.cpp)
ament_target_dependencies(path_topic_publisher_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

add_executable(path_topic_publisher2_node src/path_topic_publisher2.cpp)
ament_target_dependencies(path_topic_publisher2_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

install(TARGETS
  pid_path_tracker_node
  tf_to_pose_publisher_node
  path_publisher_node
  path_topic_publisher_node
  path_topic_publisher2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  path
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(oaslam_ros2_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(oaslam CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shared_semantic_map_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticObject.msg"
  "msg/SemanticMapSnapshot.msg"
  "msg/SemanticMapDelta.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

add_library(oaslam_ros2_wrapper_common STATIC
  src/session_config_utils.cpp
)
target_include_directories(oaslam_ros2_wrapper_common PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(oaslam_ros2_wrapper_common
  Eigen3::Eigen
  oaslam::oaslam_sdk
)
ament_target_dependencies(oaslam_ros2_wrapper_common
  geometry_msgs
  rclcpp
  sensor_msgs
  shared_semantic_map_interfaces
  std_msgs
  visualization_msgs
)
rosidl_target_interfaces(oaslam_ros2_wrapper_common
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(scout_slam_node
  src/scout_slam_node.cpp
)

target_link_libraries(scout_slam_node
  Eigen3::Eigen
  oaslam::oaslam_sdk
  oaslam_ros2_wrapper_common
)

ament_target_dependencies(scout_slam_node
  cv_bridge
  geometry_msgs
  message_filters
  rclcpp
  sensor_msgs
  shared_semantic_map_interfaces
  std_msgs
  visualization_msgs
)
rosidl_target_interfaces(scout_slam_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(scout_slam_node
  src/scout_slam_node.cpp
)

target_link_libraries(scout_slam_node
  Eigen3::Eigen
  oaslam::oaslam_sdk
  oaslam_ros2_wrapper_common
)

ament_target_dependencies(scout_slam_node
  cv_bridge
  geometry_msgs
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  sensor_msgs
  shared_semantic_map_interfaces
  std_msgs
  visualization_msgs
)
rosidl_target_interfaces(scout_slam_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS scout_slam_node scout_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

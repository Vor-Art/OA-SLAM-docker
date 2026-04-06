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
find_package(oaslam CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(oaslam_wrapper_node
  src/oaslam_wrapper_node.cpp
)

target_link_libraries(oaslam_wrapper_node
  PRIVATE
    Eigen3::Eigen
    oaslam::oaslam_sdk
)

ament_target_dependencies(oaslam_wrapper_node
  cv_bridge
  geometry_msgs
  image_transport
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS oaslam_wrapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
